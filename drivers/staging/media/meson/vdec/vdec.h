/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_CORE_H_
#define __MESON_VDEC_CORE_H_

#include <linux/irqreturn.h>
#include <linux/wait.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/soc/amlogic/meson-canvas.h>

#include "vdec_platform.h"

/* 32 buffers in 3-plane YUV420 */
#define MAX_CANVAS (32 * 3)
#define MAX_CANVAS_REGS 32

struct amvdec_buffer {
	struct list_head list;
	struct vb2_buffer *vb;
};

/**
 * struct amvdec_timestamp - stores a src timestamp along with a VIFIFO offset
 *
 * @list: used to make lists out of this struct
 * @tc: timecode from the v4l2 buffer
 * @ts: timestamp from the VB2 buffer
 * @offset: offset in the VIFIFO where the associated packet was written
 * @flags: flags from the v4l2 buffer
 * @used_count: times this timestamp was checked for a match with a dst buffer
 */
struct amvdec_timestamp {
	struct list_head list;
	struct v4l2_timecode tc;
	u64 ts;
	u32 offset;
	u32 flags;
	u32 used_count;
};

struct amvdec_timestamp_info {
	struct v4l2_timecode timecode;
	u64 timestamp;
	u32 flags;
};

struct amvdec_session;

enum amvdec_irq {
	AMVDEC_IRQ_MBOX1,
	AMVDEC_IRQ_MBOX0,
	AMVDEC_NUM_IRQS,
};

enum amvdec_hw {
	AMVDEC_HW_VDEC_1,
	AMVDEC_HW_VDEC_HEVC,
	AMVDEC_NUM_HW,
};

/**
 * struct amvdec_core - device parameters, singleton
 *
 * @dos_base: DOS memory base address
 * @esparser_base: PARSER memory base address
 * @regmap_ao: regmap for the AO bus
 * @dev: core device
 * @dev_dec: decoder device
 * @platform: platform-specific data
 * @canvas: canvas provider reference
 * @dos_parser_clk: DOS_PARSER clock
 * @dos_clk: DOS clock
 * @vdec_1_clk: VDEC_1 clock
 * @vdec_hevc_clk: VDEC_HEVC clock
 * @vdec_hevcf_clk: VDEC_HEVCF clock
 * @esparser_reset: RESET for the PARSER
 * @vdev_dec: video device for the decoder
 * @v4l2_dev: v4l2 device
 * @m2m_dev: device-level v4l2 memory-to-memory scheduler
 * @esparser_wq: wait queue for parser fetch completion
 * @esparser_search_done: parser fetch completion flag
 * @cur_sess: session currently receiving decoder interrupts
 * @hw_sess: session retained on each decoder hardware block
 * @exclusive_sess: streaming session for a codec that cannot context switch
 * @context_switching_sessions: number of streaming switchable sessions
 * @lock: video device lock
 * @hw_lock: serializes decoder hardware ownership transitions
 * @irq_lock: protects the current session observed by IRQ handlers
 * @irqs: decoder mailbox IRQs
 */
struct amvdec_core {
	void __iomem *dos_base;
	void __iomem *esparser_base;
	struct regmap *regmap_ao;

	struct device *dev;
	struct device *dev_dec;
	const struct vdec_platform *platform;

	struct meson_canvas *canvas;

	struct clk *dos_parser_clk;
	struct clk *dos_clk;
	struct clk *vdec_1_clk;
	struct clk *vdec_hevc_clk;
	struct clk *vdec_hevcf_clk;

	struct reset_control *esparser_reset;

	struct video_device *vdev_dec;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	wait_queue_head_t esparser_wq;
	struct mutex parser_lock; /* Serializes shared parser and PFIFO access. */
	bool esparser_search_done;

	struct amvdec_session *cur_sess;
	struct amvdec_session *hw_sess[AMVDEC_NUM_HW];
	struct amvdec_session *exclusive_sess;
	unsigned int context_switching_sessions;
	struct mutex lock;
	struct mutex hw_lock; /* Serializes hardware ownership changes. */
	spinlock_t irq_lock; /* Protects cur_sess for IRQ handlers. */
	int irqs[AMVDEC_NUM_IRQS];
};

/**
 * struct amvdec_ops - vdec operations
 *
 * @start: mandatory call when the vdec needs to initialize
 * @stop: mandatory call when the vdec needs to stop
 * @stop_suspended: optional call to power off an already suspended vdec
 * @resume: optional call to restore a switchable hardware context, reloading
 * firmware when requested
 * @suspend: optional call to save a switchable hardware context
 * @hw: decoder hardware block used by these operations
 * @conf_esparser: mandatory call to let the vdec configure the ESPARSER
 * @vififo_level: mandatory call to get the current amount of data
 *		  in the VIFIFO
 */
struct amvdec_ops {
	int (*start)(struct amvdec_session *sess);
	int (*stop)(struct amvdec_session *sess);
	int (*stop_suspended)(struct amvdec_session *sess);
	int (*resume)(struct amvdec_session *sess, bool reload_firmware);
	void (*suspend)(struct amvdec_session *sess);
	enum amvdec_hw hw;
	void (*conf_esparser)(struct amvdec_session *sess);
	u32 (*vififo_level)(struct amvdec_session *sess);
};

/**
 * struct amvdec_codec_ops - codec operations
 *
 * @start: mandatory call when the codec needs to initialize
 * @run: optional call after the decoder and parser hardware have started
 * @input_queued: optional call after compressed input has entered the VIFIFO
 * @input_queued_buf: optional call that retains ownership of a compressed
 *	OUTPUT buffer after it has entered the VIFIFO
 * @can_queue_input: optional call to determine whether input can be queued
 * @capture_queued: optional call after a CAPTURE buffer becomes available
 * @async_drain: finish queued input before completing a decoder stop command
 * @stop: mandatory call when the codec needs to stop
 * @release: optional call to release session resources after hardware stop
 * @irq: mailbox interrupt used by the codec firmware
 * @context_switching: the codec can save and restore its hardware context
 * @canvas_height_align: optional capture canvas height alignment
 * @capture_stride_align: optional capture line alignment, 32 bytes by default
 * @prepare_firmware: optional call to prepare a complete firmware package
 * @load_extended_firmware: optional call to load additional firmware bits
 * @has_pending_job: optional call if the codec has work that can run without
 *		     a new OUTPUT buffer
 * @job_ready: optional call to check whether a pending codec job can run
 * @num_pending_bufs: optional call to get the number of dst buffers on hold
 * @hold_capture_buf: optional call to hold a returned capture buffer while it
 *		     remains referenced by the decoder
 * @can_recycle: optional call to know if the codec is ready to recycle
 *		 a dst buffer
 * @recycle: optional call to tell the codec to recycle a dst buffer. Must go
 *	     in pair with @can_recycle
 * @drain: optional call if the codec has a custom way of draining
 * @resume: optional call to resume after a resolution change
 * @eos_sequence: optional call to get an end sequence to send to esparser
 *		  for flush. Mutually exclusive with @drain.
 * @isr: mandatory call when the ISR triggers
 * @threaded_isr: mandatory call for the threaded ISR
 */
struct amvdec_codec_ops {
	int (*start)(struct amvdec_session *sess);
	int (*run)(struct amvdec_session *sess);
	void (*input_queued)(struct amvdec_session *sess, u32 payload_size);
	void (*input_queued_buf)(struct amvdec_session *sess,
				 struct vb2_v4l2_buffer *vbuf,
				 u32 payload_size, u32 parser_size);
	bool (*can_queue_input)(struct amvdec_session *sess);
	void (*capture_queued)(struct amvdec_session *sess);
	bool async_drain;
	int (*stop)(struct amvdec_session *sess);
	void (*release)(struct amvdec_session *sess);
	enum amvdec_irq irq;
	bool context_switching;
	u32 canvas_height_align;
	u32 capture_stride_align;
	int (*prepare_firmware)(struct amvdec_session *sess,
				const u8 *data, u32 len);
	int (*load_extended_firmware)(struct amvdec_session *sess,
				      const u8 *data, u32 len);
	bool (*has_pending_job)(struct amvdec_session *sess);
	bool (*job_ready)(struct amvdec_session *sess);
	u32 (*num_pending_bufs)(struct amvdec_session *sess);
	bool (*hold_capture_buf)(struct amvdec_session *sess,
				 struct vb2_v4l2_buffer *vbuf);
	int (*can_recycle)(struct amvdec_core *core);
	void (*recycle)(struct amvdec_core *core, u32 buf_idx);
	void (*drain)(struct amvdec_session *sess);
	int (*resume)(struct amvdec_session *sess);
	const u8 * (*eos_sequence)(u32 *len);
	irqreturn_t (*isr)(struct amvdec_session *sess);
	irqreturn_t (*threaded_isr)(struct amvdec_session *sess);
};

/**
 * struct amvdec_format - describes one of the OUTPUT (src) format supported
 *
 * @pixfmt: V4L2 pixel format
 * @min_buffers: minimum amount of CAPTURE (dst) buffers
 * @max_buffers: maximum amount of CAPTURE (dst) buffers
 * @max_width: maximum picture width supported
 * @max_height: maximum picture height supported
 * @flags: enum flags associated with this pixfmt
 * @vdec_ops: the VDEC operations that support this format
 * @codec_ops: the codec operations that support this format
 * @firmware_path: Path to the firmware that supports this format
 * @pixfmts_cap: list of CAPTURE pixel formats available with pixfmt
 */
struct amvdec_format {
	u32 pixfmt;
	u32 min_buffers;
	u32 max_buffers;
	u32 max_width;
	u32 max_height;
	u32 flags;

	struct amvdec_ops *vdec_ops;
	struct amvdec_codec_ops *codec_ops;

	char *firmware_path;
	u32 pixfmts_cap[4];
};

enum amvdec_status {
	STATUS_STOPPED,
	STATUS_INIT,
	STATUS_RUNNING,
	STATUS_NEEDS_RESUME,
};

enum amvdec_m2m_job_state {
	AMVDEC_M2M_JOB_IDLE,
	AMVDEC_M2M_JOB_RUNNING,
	AMVDEC_M2M_JOB_COMPLETING,
};

/**
 * struct amvdec_session - decoding session parameters
 *
 * @core: reference to the vdec core struct
 * @fh: v4l2 file handle
 * @m2m_ctx: v4l2 m2m context
 * @ctrl_handler: V4L2 control handler
 * @ctrl_min_buf_capture: V4L2 control V4L2_CID_MIN_BUFFERS_FOR_CAPTURE
 * @lock: cap & out queues lock
 * @fmt_out: vdec pixel format for the OUTPUT queue
 * @pixfmt_cap: V4L2 pixel format for the CAPTURE queue
 * @src_buffer_size: size in bytes of the OUTPUT buffers' only plane
 * @width: current picture width
 * @height: current picture height
 * @colorspace: current colorspace
 * @bitdepth: current luma and chroma bit depth
 * @ycbcr_enc: current ycbcr_enc
 * @quantization: current quantization
 * @xfer_func: current transfer function
 * @pixelaspect: Pixel Aspect Ratio reported by the decoder
 * @esparser_queued_bufs: number of buffers currently queued into ESPARSER
 * @esparser_queue_work: work struct for the ESPARSER to process src buffers
 * @m2m_job_running: current state of this context in the m2m scheduler
 * @streamon_cap: stream on flag for capture queue
 * @streamon_out: stream on flag for output queue
 * @sequence_cap: capture sequence counter
 * @sequence_out: output sequence counter
 * @should_stop: flag set if userspace signaled EOS via command
 *		 or empty buffer
 * @draining: queued input is being drained before decoder stop
 * @keyframe_found: flag set once a keyframe has been parsed
 * @num_dst_bufs: number of destination buffers
 * @changed_format: the format changed
 * @source_change_pending: capture buffers need renegotiation
 * @canvas_alloc: array of all the canvas IDs allocated
 * @canvas_num: number of canvas IDs allocated
 * @canvas_regs: DOS registers containing the session canvas mappings
 * @canvas_values: values restored to the session canvas registers
 * @canvas_reg_count: number of saved canvas registers
 * @vififo_vaddr: virtual address for the VIFIFO
 * @vififo_paddr: physical address for the VIFIFO
 * @vififo_size: size of the VIFIFO dma alloc
 * @vififo_curr: saved VIFIFO current pointer
 * @vififo_wp: saved VIFIFO write pointer
 * @vififo_rp: saved VIFIFO read pointer
 * @vififo_wrap_count: saved VIFIFO wrap counter
 * @vififo_context_valid: whether the saved VIFIFO registers are valid
 * @bufs_recycle: list of buffers that need to be recycled
 * @bufs_recycle_lock: lock for the bufs_recycle list
 * @recycle_thread: task struct for the recycling thread
 * @timestamps: chronological list of src timestamps
 * @ts_spinlock: spinlock for the timestamps list
 * @last_irq_jiffies: tracks last time the vdec triggered an IRQ
 * @irq_seen: decoder has produced at least one interrupt
 * @hardware_stalled: decoder stopped responding before teardown
 * @last_offset: tracks last offset of vififo
 * @wrap_count: number of times the vififo wrapped around
 * @fw_idx_to_vb2_idx: firmware buffer index to vb2 buffer index
 * @status: current decoding status
 * @priv: codec private data
 */
struct amvdec_session {
	struct amvdec_core *core;

	struct v4l2_fh fh;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *ctrl_min_buf_capture;
	struct mutex lock;

	const struct amvdec_format *fmt_out;
	u32 pixfmt_cap;
	u32 src_buffer_size;

	u32 width;
	u32 height;
	u32 colorspace;
	u8 bitdepth;
	u8 ycbcr_enc;
	u8 quantization;
	u8 xfer_func;

	struct v4l2_fract pixelaspect;

	atomic_t esparser_queued_bufs;
	struct work_struct esparser_queue_work;
	atomic_t m2m_job_running;

	unsigned int streamon_cap, streamon_out;
	unsigned int sequence_cap, sequence_out;
	unsigned int should_stop;
	unsigned int draining;
	unsigned int keyframe_found;
	unsigned int num_dst_bufs;
	unsigned int changed_format;
	bool source_change_pending;

	u8 canvas_alloc[MAX_CANVAS];
	u32 canvas_num;
	u32 canvas_regs[MAX_CANVAS_REGS];
	u32 canvas_values[MAX_CANVAS_REGS];
	u32 canvas_reg_count;

	void *vififo_vaddr;
	dma_addr_t vififo_paddr;
	u32 vififo_size;
	u32 vififo_curr;
	u32 vififo_wp;
	u32 vififo_rp;
	u32 vififo_wrap_count;
	bool vififo_context_valid;

	struct list_head bufs_recycle;
	struct mutex bufs_recycle_lock; /* bufs_recycle list lock */
	struct task_struct *recycle_thread;

	struct list_head timestamps;
	spinlock_t ts_spinlock; /* timestamp list lock */

	u64 last_irq_jiffies;
	bool irq_seen;
	bool hardware_stalled;
	u32 last_offset;
	u32 wrap_count;
	u32 fw_idx_to_vb2_idx[32];

	enum amvdec_status status;
	void *priv;
};

static inline struct amvdec_session *file_to_amvdec_session(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct amvdec_session, fh);
}

u32 amvdec_get_output_size(struct amvdec_session *sess);
bool amvdec_session_is_current(struct amvdec_session *sess);
int amvdec_m2m_job_start(struct amvdec_session *sess);
bool amvdec_m2m_begin_yield(struct amvdec_session *sess);
void amvdec_m2m_finish_yield(struct amvdec_session *sess);
void amvdec_m2m_job_finish(struct amvdec_session *sess);
void amvdec_m2m_job_yield(struct amvdec_session *sess);
void amvdec_m2m_retry_job(struct amvdec_session *sess);

#endif
