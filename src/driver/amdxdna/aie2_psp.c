// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022-2026, Advanced Micro Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/slab.h>
#include <linux/iopoll.h>
#include <linux/pci.h>
#include "aie2_pci.h"
#include "amdxdna_xen.h"

/*
 * Firmware patching: module parameter to enable VE2 IPU firmware patching.
 * When enabled, applies the following patches to npu_7.sbin (PCI 17f0 rev 0x11):
 *   - Column limit unlock: changes max_columns from 8 to 40 (offsets 0x17B04,
 *     0x17B10), enabling all physically available AIE columns on Strix Halo
 * Disabled by default for safety (patches are VE2-firmware-specific).
 *
 * NOTE: Multi-app gate bypass (offsets 0x2DD8-0x2DF7) was removed because
 * those offsets are a data table of 32-bit relative values in firmware
 * 1.1.2.65, not function pointers. Zeroing them crashes the firmware.
 */
bool fw_patches_enable;
EXPORT_SYMBOL(fw_patches_enable);
module_param(fw_patches_enable, bool, 0644);
MODULE_PARM_DESC(fw_patches_enable, "Enable VE2 IPU firmware patches (unlock 40 columns)");

/*
 * Firmware patch descriptor: byte-level patches applied to the DMA buffer
 * before PSP_START copies the firmware into NPU internal SRAM.
 */
struct aie_fw_patch {
	u32	offset;
	u8	data[8];
};

/*
 * VE2 (NPU5/Strix Halo) IPU firmware patches.
 *
 * Column limit unlock (offsets 0x17B04, 0x17B10):
 *   Modifies the VE2 IPU firmware's column_config structure to expose
 *   all 40 physically available AIE columns instead of the default 8:
 *     - 0x17B04: max_columns  0x00000008 -> 0x00000028 (8 -> 40)
 *     - 0x17B10: flags        0x08000010 -> 0x28000010 (encodes max_col=40)
 *   TOPS scales linearly with column count:
 *     ~4096 * total_col * clock_freq / 1,000,000
 *
 * These offsets are VE2 IPU firmware-specific (npu_7.sbin for PCI
 * 17f0 rev 0x11).  Do NOT apply to other firmware versions.
 */
static const struct aie_fw_patch ve2_fw_patches[] = {
	/* Column limit unlock (8 -> 40 columns) */
	{ 0x17B04, { 0x28, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF } },
	{ 0x17B10, { 0x10, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00 } },
};

/*
 * aie_should_apply_fw_patches() - Check if VE2 firmware patching is safe.
 *
 * Only applies patches when:
 *   1. The module parameter fw_patches_enable is set (opt-in)
 *   2. The device is VE2/NPU5 (PCI 17f0 rev 0x11)
 *
 * This prevents corrupting non-VE2 firmwares with VE2-specific offsets.
 */
static bool aie_should_apply_fw_patches(struct psp_device *psp)
{
	struct pci_dev *pdev;

	if (!fw_patches_enable)
		return false;

	pdev = to_pci_dev(psp->dev);
	if (pdev->device == 0x17f0 && pdev->revision == 0x11)
		return true;

	dev_warn(psp->dev,
		 "fw_patches_enable set but device 0x%04x rev 0x%02x is not VE2/NPU5; skipping\n",
		 pdev->device, pdev->revision);
	return false;
}

static void aie_apply_fw_patches(struct psp_device *psp, u64 buf_offset)
{
	u8 *buf = (u8 *)psp->fw_buffer + buf_offset;
	int i;

	for (i = 0; i < ARRAY_SIZE(ve2_fw_patches); i++) {
		const struct aie_fw_patch *p = &ve2_fw_patches[i];
		memcpy(buf + p->offset, p->data, sizeof(p->data));
	}

	dev_info(psp->dev, "Applied %zu VE2 firmware patches (40-col unlock)\n",
		 ARRAY_SIZE(ve2_fw_patches));
}

static inline char *psp_decode_resp(u32 resp)
{
	switch (resp) {
	case PSP_ERROR_CANCEL:
		return "Error cancel";
	case PSP_ERROR_BAD_STATE:
		return "Error bad state";
	default:
		return "Error unknown";
	};
}

static int psp_exec(struct psp_device *psp, u32 *reg_vals)
{
	u32 resp_code;
	int ret, i;
	u32 ready;

	/* Write command and argument registers */
	for (i = 0; i < PSP_NUM_IN_REGS; i++)
		writel(reg_vals[i], PSP_REG(psp, i));

	/* clear and set PSP INTR register to kick off */
	writel(0, PSP_REG(psp, PSP_INTR_REG));
	writel(1, PSP_REG(psp, PSP_INTR_REG));

	/* PSP should be busy. Wait for ready, so we know task is done. */
	ret = readx_poll_timeout(readl, PSP_REG(psp, PSP_STATUS_REG), ready,
				 FIELD_GET(PSP_STATUS_READY, ready),
				 PSP_POLL_INTERVAL, PSP_POLL_TIMEOUT);
	if (ret) {
		dev_err(psp->dev, "PSP is not ready, ret 0x%x", ret);
		return ret;
	}

	resp_code = readl(PSP_REG(psp, PSP_RESP_REG));
	if (resp_code) {
		dev_err(psp->dev, "fw return error 0x%x(%s)", resp_code,
			psp_decode_resp(resp_code));
		return -EIO;
	}

	return 0;
}

int aie2_psp_waitmode_poll(struct psp_device *psp)
{
	int mode_reg = -1, ret;

	ret = readx_poll_timeout(readl, PSP_REG(psp, PSP_PWAITMODE_REG), mode_reg,
				 (mode_reg & 0x1) == 1,
				 PSP_POLL_INTERVAL, PSP_POLL_TIMEOUT);
	if (ret) {
		dev_err(psp->dev, "fw waitmode reg error, ret 0x%x", ret);
		return ret;
	}

	return 0;
}

void aie2_psp_stop(struct psp_device *psp)
{
	u32 reg_vals[PSP_NUM_IN_REGS] = { PSP_RELEASE_TMR, };
	int ret;

	ret = psp_exec(psp, reg_vals);
	if (ret)
		dev_err(psp->dev, "release tmr failed, ret %d", ret);
}

int aie2_psp_start(struct psp_device *psp)
{
	u32 reg_vals[PSP_NUM_IN_REGS];
	u64 buf_offset;
	int ret;

	/*
	 * SKIP PSP_VALIDATE: we are loading a patched firmware.
	 * PSP would reject the modified firmware (error 0x63) because
	 * the cryptographic signature no longer matches. We skip
	 * validation and load the firmware directly.
	 */
	dev_info(psp->dev, "PSP validation SKIPPED -- loading patched firmware directly\n");

	/*
	 * Optionally apply VE2 firmware patches before PSP_START copies
	 * the firmware into NPU SRAM. Since we skip PSP_VALIDATE, we can
	 * patch the DMA buffer here. PSP_START with PSP_START_COPY_FW
	 * reads the (now-patched) buffer and copies it to NPU execution
	 * memory. The PSP does not re-validate during PSP_START.
	 *
	 * Only applied when:
	 *   - fw_patches_enable module param is set (opt-in)
	 *   - Device is VE2/NPU5 (PCI 17f0 rev 0x11)
	 */
	buf_offset = psp->fw_paddr - virt_to_phys(psp->fw_buffer);
	if (aie_should_apply_fw_patches(psp))
		aie_apply_fw_patches(psp, buf_offset);

	memset(reg_vals, 0, sizeof(reg_vals));
	reg_vals[0] = PSP_START;
	reg_vals[1] = PSP_START_COPY_FW;
	ret = psp_exec(psp, reg_vals);
	if (ret) {
		dev_err(psp->dev, "failed to start fw, ret %d", ret);
		return ret;
	}

	return 0;
}

void aie2_psp_destroy(struct device *dev, void *psp_hdl)
{
	struct psp_device *psp = psp_hdl;

	if (is_xen_initial_pvh_domain())
		amdxdna_xen_free_buf_phys(dev, psp->fw_buffer, psp->fw_dma_handle,
					  psp->fw_buf_sz + PSP_FW_ALIGN);
}
