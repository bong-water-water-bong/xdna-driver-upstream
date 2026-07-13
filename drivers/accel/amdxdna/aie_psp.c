// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026, Advanced Micro Devices, Inc.
 */

#include <drm/drm_device.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>
#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "aie.h"
#include "amdxdna_xen.h"

#define PSP_STATUS_READY	BIT(31)

/* PSP commands */
#define PSP_START		2
#define PSP_RELEASE_TMR		3

/* PSP special arguments */
#define PSP_START_COPY_FW	1

/* PSP response error code */
#define PSP_ERROR_CANCEL	0xFFFF0002
#define PSP_ERROR_BAD_STATE	0xFFFF0007

#define PSP_FW_ALIGN		0x10000
#define PSP_POLL_INTERVAL	20000	/* us */
#define PSP_POLL_TIMEOUT	1000000	/* us */

/*
 * Firmware patching: module parameter to enable VE2 IPU firmware patching.
 * When enabled, patches the VE2 IPU firmware's column_config to expose
 * all 40 physically available AIE columns instead of the default 8.
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
 * Tracking: whether PSP cmd #3 is actually issued.
 * PSP_RELEASE_TMR (cmd 3)  MUST only be sent if we
 * successfully called PSP_START on the NPU side.  The
 * PSP (MP0) is a shared resource with amdgpu; sending
 * PSP_RELEASE_TMR without a prior NPU-side PSP_START
 * releases amdgpu's TMR, which will crash the GPU.
 */
#define PSP_F_SKIP_TMR_RELEASE	BIT(0)

#define PSP_REG(p, reg) ((p)->conf.psp_regs[reg])
#define PSP_SET_CMD(psp, reg_vals, cmd, arg0, arg1, arg2)		\
({									\
	u32 *_regs = reg_vals;						\
	u32 _cmd = cmd;							\
	_regs[0] = _cmd;						\
	_regs[1] = arg0;						\
	_regs[2] = arg1;						\
	_regs[3] = ((arg2) | ((_cmd) << 24)) & (psp)->conf.arg2_mask;	\
})

struct psp_device {
	struct drm_device	*ddev;
	struct psp_config	conf;
	struct amdxdna_xen_bufs_mgr xen_mgr;
	u32			fw_buf_sz;
	u64			fw_paddr;
	void			*fw_buffer;
	unsigned long		flags;
};

/*
 * Firmware patch descriptor: byte-level patches applied to the DMA buffer
 * before PSP_START copies the firmware into NPU internal SRAM.
 *
 * We skip PSP_VALIDATE (shared PSP with GPU is busy), so patching happens
 * before PSP_START instead of between VALIDATE and START like the DKMS
 * version does.  Since we never called PSP_VALIDATE on the stock firmware,
 * there is no PSP validation to worry about -- we can patch freely and
 * PSP_START will copy the patched firmware directly.
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

	pdev = to_pci_dev(psp->ddev.dev);
	if (pdev->device == 0x17f0 && pdev->revision == 0x11)
		return true;

	drm_warn(psp->ddev,
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

	drm_info(psp->ddev, "Applied %zu VE2 firmware patches (40-col unlock)\n",
		 ARRAY_SIZE(ve2_fw_patches));
}

static int psp_exec(struct psp_device *psp, u32 *reg_vals)
{
	u32 resp_code;
	int ret, i;
	u32 ready;

	/* Check for PSP ready before any write */
	ret = readx_poll_timeout(readl, PSP_REG(psp, PSP_STATUS_REG), ready,
				 FIELD_GET(PSP_STATUS_READY, ready),
				 PSP_POLL_INTERVAL, PSP_POLL_TIMEOUT);
	if (ret) {
		drm_err(psp->ddev, "PSP is not ready, ret 0x%x", ret);
		return ret;
	}

	/* Write command and argument registers */
	for (i = 0; i < PSP_NUM_IN_REGS; i++)
		writel(reg_vals[i], PSP_REG(psp, i));

	/* clear and set PSP INTR register to kick off */
	writel(0, PSP_REG(psp, PSP_INTR_REG));
	writel(psp->conf.notify_val, PSP_REG(psp, PSP_INTR_REG));

	/* PSP should be busy. Wait for ready, so we know task is done. */
	ret = readx_poll_timeout(readl, PSP_REG(psp, PSP_STATUS_REG), ready,
				 FIELD_GET(PSP_STATUS_READY, ready),
				 PSP_POLL_INTERVAL, PSP_POLL_TIMEOUT);
	if (ret) {
		drm_err(psp->ddev, "PSP is not ready, ret 0x%x", ret);
		return ret;
	}

	resp_code = readl(PSP_REG(psp, PSP_RESP_REG));
	if (resp_code) {
		drm_err(psp->ddev, "fw return error 0x%x", resp_code);
		return -EIO;
	}

	return 0;
}

int aie_psp_waitmode_poll(struct psp_device *psp)
{
	struct amdxdna_dev *xdna = to_xdna_dev(psp->ddev);
	u32 mode_reg;
	int ret;

	ret = readx_poll_timeout(readl, PSP_REG(psp, PSP_PWAITMODE_REG), mode_reg,
				 (mode_reg & 0x1) == 1,
				 PSP_POLL_INTERVAL, PSP_POLL_TIMEOUT);
	if (ret)
		XDNA_ERR(xdna, "fw waitmode reg error, ret %d", ret);

	return ret;
}

void aie_psp_stop(struct psp_device *psp)
{
	u32 reg_vals[PSP_NUM_IN_REGS];
	int ret;

	if (psp->flags & PSP_F_SKIP_TMR_RELEASE) {
		drm_info(psp->ddev, "Skipping PSP_RELEASE_TMR (NPU PSP was never started)\n");
		return;
	}

	PSP_SET_CMD(psp, reg_vals, PSP_RELEASE_TMR, 0, 0, 0);

	ret = psp_exec(psp, reg_vals);
	if (ret)
		drm_err(psp->ddev, "release tmr failed, ret %d", ret);
}

static int psp_start(struct psp_device *psp)
{
	u32 reg_vals[PSP_NUM_IN_REGS];
	u64 buf_offset;
	int ret;

	buf_offset = psp->fw_paddr - virt_to_phys(psp->fw_buffer);

	/*
	 * Optionally apply VE2 firmware patches before PSP_START copies
	 * the firmware into NPU SRAM.  Since we skip PSP_VALIDATE (shared
	 * PSP is GPU-owned), we patch the DMA buffer here. PSP_START with
	 * PSP_START_COPY_FW reads the (now-patched) buffer and copies it
	 * to NPU execution memory. The PSP does not re-validate during
	 * PSP_START.
	 *
	 * Only applied when:
	 *   - fw_patches_enable module param is set (opt-in)
	 *   - Device is VE2/NPU5 (PCI 17f0 rev 0x11)
	 */
	if (aie_should_apply_fw_patches(psp))
		aie_apply_fw_patches(psp, buf_offset);

	PSP_SET_CMD(psp, reg_vals, PSP_START, PSP_START_COPY_FW, 0, 0);

	ret = psp_exec(psp, reg_vals);
	if (ret)
		drm_err(psp->ddev, "failed to start fw, ret %d", ret);

	return ret;
}

int aie_psp_start(struct psp_device *psp)
{
	int ret;

	/*
	 * Skip PSP_VALIDATE: the PSP (MP0) is a system-wide resource shared
	 * with the GPU (amdgpu). On Strix Halo, the GPU driver initializes
	 * the PSP first (loading DMUB firmware, reserving TMR). Attempting
	 * PSP_VALIDATE while the GPU's PSP session is active causes a timeout
	 * since the PSP is busy.
	 *
	 * Worse, the cleanup path (aie_psp_stop -> PSP_RELEASE_TMR) can then
	 * release the GPU's Trusted Memory Region, wedging the shared SMU and
	 * crashing the entire system.
	 *
	 * Skipping validation and going straight to PSP_START works because:
	 *   1) PSP_START only triggers DMA to NPU SRAM, not PSP core work.
	 *   2) The NPU firmware was validated by BIOS/UEFI secure boot.
	 *   3) This matches the in-kernel amdxdna driver behavior.
	 *
	 * However, since we skipped PSP_VALIDATE, the NPU firmware was never
	 * associated with the PSP's TMR.  Sending PSP_RELEASE_TMR on cleanup
	 * would release amdgpu's TMR instead, causing a GPU crash.  Mark the
	 * PSP so aie_psp_stop() skips the TMR release.
	 */
	psp->flags |= PSP_F_SKIP_TMR_RELEASE;

	ret = psp_start(psp);
	if (ret)
		return ret;

	/*
	 * PSP_START succeeded -- the NPU firmware is now running in its own
	 * context on the shared PSP.  PSP_RELEASE_TMR is now safe and will
	 * release the NPU's TMR (or no-op if the NPU doesn't use TMR).
	 */
	psp->flags &= ~PSP_F_SKIP_TMR_RELEASE;

	return 0;
}

/*
 * PSP requires host physical address to load firmware.
 * Allocate a buffer, obtain its physical address, align, and copy data in.
 */
static void *psp_alloc_fw_buf(struct psp_device *psp, const void *fw_data,
			      u32 fw_size, u32 align, u32 *buf_sz,
			      u64 *paddr)
{
	u32 alloc_sz;
	void *buffer;
	u64 offset;

	*buf_sz = ALIGN(fw_size, align);
	alloc_sz = *buf_sz + align;

	if (amdxdna_is_xen_initial_pvh_domain()) {
		buffer = amdxdna_xen_bufs_alloc(&psp->xen_mgr, alloc_sz,
						paddr);
		if (!buffer)
			return NULL;
	} else {
		buffer = drmm_kmalloc(psp->ddev, alloc_sz, GFP_KERNEL);
		if (!buffer)
			return NULL;
		*paddr = virt_to_phys(buffer);
	}

	offset = ALIGN(*paddr, align) - *paddr;
	*paddr += offset;
	memcpy(buffer + offset, fw_data, fw_size);

	return buffer;
}

struct psp_device *aiem_psp_create(struct drm_device *ddev, struct psp_config *conf)
{
	struct psp_device *psp;

	psp = drmm_kzalloc(ddev, sizeof(*psp), GFP_KERNEL);
	if (!psp)
		return NULL;

	psp->ddev = ddev;
	amdxdna_xen_bufs_init(&psp->xen_mgr, ddev->dev);
	if (drmm_add_action_or_reset(ddev, amdxdna_xen_bufs_drmm_release,
				     &psp->xen_mgr))
		return NULL;

	psp->fw_buffer = psp_alloc_fw_buf(psp, conf->fw_buf, conf->fw_size,
					  PSP_FW_ALIGN, &psp->fw_buf_sz,
					  &psp->fw_paddr);
	if (!psp->fw_buffer)
		return NULL;


	memcpy(&psp->conf, conf, sizeof(psp->conf));

	return psp;
}
