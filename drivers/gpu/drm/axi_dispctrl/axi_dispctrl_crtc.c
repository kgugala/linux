/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/amba/xilinx_dma.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "axi_dispctrl_crtc.h"
#include "axi_dispctrl_drv.h"
#include "axi_dispctrl_encoder.h"

struct axi_dispctrl_crtc {
	struct drm_crtc drm_crtc;
	struct dma_chan *dma;
	struct xilinx_dma_config dma_config;
	int mode;
};

static inline struct
axi_dispctrl_crtc *to_axi_dispctrl_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct axi_dispctrl_crtc, drm_crtc);
}

static int axi_dispctrl_crtc_update(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->primary->fb;
	struct dma_async_tx_descriptor *desc;
	struct dma_interleaved_template *xt;
	struct drm_gem_cma_object *obj;
	struct xilinx_vdma_config xconf;
	size_t offset;
	int ret;

	axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);
	if (!mode || !fb)
		return -EINVAL;

	dmaengine_terminate_all(axi_dispctrl_crtc->dma);
	if (axi_dispctrl_crtc->mode == DRM_MODE_DPMS_ON) {
		obj = drm_fb_cma_get_gem_obj(fb, 0);
		if (!obj)
			return -EINVAL;

		xt = kzalloc(sizeof(struct dma_async_tx_descriptor) +
			     sizeof(struct data_chunk), GFP_KERNEL);
		if (!xt)
			return -ENOMEM;

		offset = crtc->x * fb->bits_per_pixel / 8 +
			 crtc->y * fb->pitches[0];

		/* configure xilinx spacific DMA */
		xconf.frm_dly = 0;
		xconf.park = 1;
		xconf.park_frm = 0;

		xilinx_vdma_channel_set_config(axi_dispctrl_crtc->dma, &xconf);

		xt->src_start = obj->paddr + offset;
		xt->src_inc = false;
		xt->dst_inc = true;
		xt->src_sgl = false;
		xt->dst_sgl = true;
		xt->frame_size = 1;
		xt->numf = mode->vdisplay;
		xt->sgl[0].size = mode->hdisplay * fb->bits_per_pixel / 8;
		xt->sgl[0].icg = 0;
		xt->dir = DMA_MEM_TO_DEV;

		desc = dmaengine_prep_interleaved_dma(axi_dispctrl_crtc->dma,
						      xt, DMA_PREP_INTERRUPT);
		kfree(xt);
		if (!desc)
			ret = -ENOMEM;
		else {
			dmaengine_submit(desc);
			dma_async_issue_pending(axi_dispctrl_crtc->dma);
			ret = 0;
		}
	}
	return ret;
}

static void axi_dispctrl_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;

	axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);
	if (axi_dispctrl_crtc->mode != mode) {
		axi_dispctrl_crtc->mode = mode;
		axi_dispctrl_crtc_update(crtc);
	}
}

static void axi_dispctrl_crtc_prepare(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;

	axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);
	dmaengine_terminate_all(axi_dispctrl_crtc->dma);
}

static void axi_dispctrl_crtc_commit(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;

	axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);
	axi_dispctrl_crtc->mode = DRM_MODE_DPMS_ON;
	axi_dispctrl_crtc_update(crtc);
}

static bool axi_dispctrl_crtc_mode_fixup(struct drm_crtc *crtc,
					 const struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int axi_dispctrl_crtc_mode_set(struct drm_crtc *crtc,
				      struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode,
				      int x, int y,
				      struct drm_framebuffer *old_fb)
{
	/* We do everything in commit() */
	return 0;
}

static int axi_dispctrl_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					   struct drm_framebuffer *old_fb)
{
	return axi_dispctrl_crtc_update(crtc);
}

static void axi_dispctrl_crtc_load_lut(struct drm_crtc *crtc)
{
}

static struct drm_crtc_helper_funcs axi_dispctrl_crtc_helper_funcs = {
	.dpms		= axi_dispctrl_crtc_dpms,
	.prepare	= axi_dispctrl_crtc_prepare,
	.commit		= axi_dispctrl_crtc_commit,
	.mode_fixup	= axi_dispctrl_crtc_mode_fixup,
	.mode_set	= axi_dispctrl_crtc_mode_set,
	.mode_set_base	= axi_dispctrl_crtc_mode_set_base,
	.load_lut	= axi_dispctrl_crtc_load_lut,
};

static void axi_dispctrl_crtc_destroy(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;

	axi_dispctrl_crtc  = to_axi_dispctrl_crtc(crtc);
	drm_crtc_cleanup(crtc);
	kfree(axi_dispctrl_crtc);
}

static struct drm_crtc_funcs axi_dispctrl_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.destroy	= axi_dispctrl_crtc_destroy,
};

struct drm_crtc *axi_dispctrl_crtc_create(struct drm_device *dev)
{
	struct axi_dispctrl_private *p = dev->dev_private;
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;
	struct drm_crtc *crtc;

	axi_dispctrl_crtc = kzalloc(sizeof(*axi_dispctrl_crtc), GFP_KERNEL);
	if (!axi_dispctrl_crtc)
		return ERR_PTR(-ENOMEM);

	crtc = &axi_dispctrl_crtc->drm_crtc;

	axi_dispctrl_crtc->dma = p->dma;

	drm_crtc_init(dev, crtc, &axi_dispctrl_crtc_funcs);
	drm_crtc_helper_add(crtc, &axi_dispctrl_crtc_helper_funcs);

	return crtc;
}
