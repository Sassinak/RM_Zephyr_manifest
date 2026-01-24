/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rp_can_rx_manager

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <string.h>

#include <drivers/can_rx_manager.h>

#define LOG_LEVEL 3
LOG_MODULE_REGISTER(can_rx_manager);

struct rp_can_rx_listener {
	bool used;
	struct can_filter filter;
	rp_can_rx_handler_t handler;
	void *user_data;
};

struct rp_can_rx_manager_cfg {
	const struct device *can_dev;
	struct k_msgq *rx_msgq;
	k_thread_stack_t *rx_stack;
	size_t rx_stack_size;
};

struct rp_can_rx_manager_data {
	struct rp_can_rx_listener listeners[CONFIG_CAN_RX_MANAGER_MAX_LISTENERS];
	struct k_thread rx_thread;
	int hw_filter_id;
#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
	atomic_t rx_dropped;
	atomic_t rx_queued;
#endif
};

#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
static void rp_can_rx_isr_cb(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(can_dev);

	const struct device *mgr = (const struct device *)user_data;
	if ((mgr == NULL) || (frame == NULL)) {
		return;
	}

	const struct rp_can_rx_manager_cfg *cfg = mgr->config;
	struct rp_can_rx_manager_data *data = mgr->data;
	if ((cfg == NULL) || (data == NULL) || (cfg->rx_msgq == NULL)) {
		return;
	}

	/* Skip RTR frames by default */
	if ((frame->flags & CAN_FRAME_RTR) != 0U) {
		return;
	}

	int ret = k_msgq_put(cfg->rx_msgq, frame, K_NO_WAIT);
#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
	if (ret == 0) {
		(void)atomic_inc(&data->rx_queued);
	} else {
		(void)atomic_inc(&data->rx_dropped);
	}
#endif
	ARG_UNUSED(ret);
}
#endif

/**
 * @brief 匹配 CAN 帧与过滤器，支持标准/扩展 ID
 * 
 * @param filter 
 * @param frame 
 * @return true 
 * @return false 
 */
static bool rp_can_rx_match(const struct can_filter *filter, const struct can_frame *frame)
{
	uint32_t frame_id = frame->id;
	uint32_t filter_id = filter->id;

	if (((frame->flags & CAN_FRAME_IDE) != 0U) != ((filter->flags & CAN_FILTER_IDE) != 0U)) {
		return false;
	}

	if ((frame->flags & CAN_FRAME_IDE) != 0U) {
		frame_id &= CAN_EXT_ID_MASK;
		filter_id &= CAN_EXT_ID_MASK;
	} else {
		frame_id &= CAN_STD_ID_MASK;
		filter_id &= CAN_STD_ID_MASK;
	}

	return (frame_id & filter->mask) == (filter_id & filter->mask);
}

static void rp_can_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)p1;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct rp_can_rx_manager_cfg *cfg = dev->config;
	struct rp_can_rx_manager_data *data = dev->data;

	if ((cfg == NULL) || (data == NULL) || (cfg->rx_msgq == NULL)) {
		return;
	}

	struct can_frame frame;

#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
	uint32_t last_reported_drops = 0U;
#endif

	while (true) {
		int ret = k_msgq_get(cfg->rx_msgq, &frame, K_FOREVER);
		if (ret != 0) {
			continue;
		}

#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
		uint32_t drops = (uint32_t)atomic_get(&data->rx_dropped);
		if (drops != last_reported_drops) {
			uint32_t step = (uint32_t)CONFIG_CAN_RX_MANAGER_MSGQ_WARN_EVERY_N_DROPS;
			if ((drops / step) != (last_reported_drops / step)) {
				uint32_t queued = (uint32_t)atomic_get(&data->rx_queued);
				LOG_WRN("RX mgr msgq drops=%u queued=%u (msgq_len=%u)",
					drops, queued, (uint32_t)CONFIG_CAN_RX_MANAGER_RX_MSGQ_LEN);
			}
			last_reported_drops = drops;
		}
#endif

		for (int i = 0; i < CONFIG_CAN_RX_MANAGER_MAX_LISTENERS; i++) {
			struct rp_can_rx_listener *lst = &data->listeners[i];
			if (!lst->used) {
				continue;
			}
			if (!rp_can_rx_match(&lst->filter, &frame)) {
				continue;
			}

			lst->handler(&frame, lst->user_data);
		}
	}
}

int rp_can_rx_manager_register(const struct device *mgr, const struct can_filter *filter,
			      rp_can_rx_handler_t handler, void *user_data)
{
	if ((mgr == NULL) || (filter == NULL) || (handler == NULL)) {
		return -EINVAL;
	}
	if (!device_is_ready(mgr)) {
		return -ENODEV;
	}

	struct rp_can_rx_manager_data *data = mgr->data;
	if (data == NULL) {
		return -EINVAL;
	}

	for (int i = 0; i < CONFIG_CAN_RX_MANAGER_MAX_LISTENERS; i++) {
		if (data->listeners[i].used) {
			continue;
		}
		data->listeners[i].used = true;
		data->listeners[i].filter = *filter;
		data->listeners[i].handler = handler;
		data->listeners[i].user_data = user_data;
		return i;
	}

	return -ENOSPC;
}

int rp_can_rx_manager_unregister(const struct device *mgr, int listener_id)
{
	if ((mgr == NULL) || (listener_id < 0) || (listener_id >= CONFIG_CAN_RX_MANAGER_MAX_LISTENERS)) {
		return -EINVAL;
	}
	if (!device_is_ready(mgr)) {
		return -ENODEV;
	}

	struct rp_can_rx_manager_data *data = mgr->data;
	if (data == NULL) {
		return -EINVAL;
	}

	if (!data->listeners[listener_id].used) {
		return -ENOENT;
	}

	data->listeners[listener_id].used = false;
	data->listeners[listener_id].handler = NULL;
	data->listeners[listener_id].user_data = NULL;
	memset(&data->listeners[listener_id].filter, 0, sizeof(data->listeners[listener_id].filter));
	return 0;
}

static int rp_can_rx_manager_init(const struct device *dev)
{
	const struct rp_can_rx_manager_cfg *cfg = dev->config;
	struct rp_can_rx_manager_data *data = dev->data;

	if ((cfg == NULL) || (data == NULL)) {
		return -EINVAL;
	}
	if (!device_is_ready(cfg->can_dev)) {
		return -ENODEV;
	}

	int ret = can_start(cfg->can_dev);
	if ((ret < 0) && (ret != -EALREADY)) {
		return ret;
	}

	/* One broad hardware filter: accept all standard IDs into manager msgq */
	const struct can_filter hw = {
		.id = 0,
		.mask = 0,
		.flags = 0,
	};

	/*
	 * 默认路径：使用 Zephyr 提供的 can_add_rx_filter_msgq()。
	 * 监控开启时：改用自定义回调，在 k_msgq_put(K_NO_WAIT) 失败时统计丢帧（msgq 满）。
	 */
#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
	ret = can_add_rx_filter(cfg->can_dev, rp_can_rx_isr_cb, (void *)dev, &hw);
#else
	ret = can_add_rx_filter_msgq(cfg->can_dev, cfg->rx_msgq, &hw);
#endif
	if (ret < 0) {
		return ret;
	}
	data->hw_filter_id = ret;

	k_thread_create(&data->rx_thread, cfg->rx_stack,
			cfg->rx_stack_size, rp_can_rx_thread,
			(void *)dev, NULL, NULL, CONFIG_CAN_RX_MANAGER_RX_THREAD_PRIO, 0, K_NO_WAIT);

	return 0;
}

#define RP_CAN_RX_MGR_DEFINE(inst) \
	CAN_MSGQ_DEFINE(rp_can_rx_manager_msgq_##inst, CONFIG_CAN_RX_MANAGER_RX_MSGQ_LEN); \
	K_THREAD_STACK_DEFINE(rp_can_rx_manager_stack_##inst, CONFIG_CAN_RX_MANAGER_RX_STACK_SIZE); \
	static const struct rp_can_rx_manager_cfg rp_can_rx_mgr_cfg_##inst = { \
		.can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)), \
		.rx_msgq = &rp_can_rx_manager_msgq_##inst, \
		.rx_stack = rp_can_rx_manager_stack_##inst, \
		.rx_stack_size = K_THREAD_STACK_SIZEOF(rp_can_rx_manager_stack_##inst), \
	}; \
	static struct rp_can_rx_manager_data rp_can_rx_mgr_data_##inst; \
	DEVICE_DT_INST_DEFINE(inst, rp_can_rx_manager_init, NULL, &rp_can_rx_mgr_data_##inst, \
			  &rp_can_rx_mgr_cfg_##inst, POST_KERNEL, CONFIG_CAN_RX_MANAGER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RP_CAN_RX_MGR_DEFINE)
