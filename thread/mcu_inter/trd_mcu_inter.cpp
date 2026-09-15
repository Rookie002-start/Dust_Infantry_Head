/**
 * @file trd_mcu_inter.cpp
 * @author qingyu
 * @brief 上下板 FDCAN 发送线程
 *        - 入队的状态分片先存进【本线程私有】的分片缓存，再按 Dir::Up 布局聚合进同一帧周期发出
 * @version 0.4
 * @date 2026-09-11
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/trd_mcu_inter"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "to_mcu_tx.hpp"
#include "inter_cmd.hpp"
#include "can.hpp"
#include "Irq_handlers.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mcu_inter, LOG_LEVEL_INF);

namespace thread::mcu_inter
{
    static Thread<2048> thread_{};
    static Can mcu_inter_can{};

    static constexpr uint8_t  kFrameSize = 64;      // 聚合帧载荷上限

    static constexpr inter_cmd::Dir kDir      = inter_cmd::Dir::Up;            // 本线程发送方向
    static constexpr uint16_t       kTxId     = inter_cmd::StateTxId(kDir);    // 0x100
    static constexpr uint8_t        kStateLen = inter_cmd::FrameLen(kDir);     // 48

    // ---- 私有分片缓存：按契约布局表下标索引（本线程独有，不共享 → 无需锁）----
    // 布局（类型/偏移/长度）全部来自 inter_cmd.hpp 的 kStateFrags 表，这里不再重复定义
    struct Slot
    {
        uint8_t data[kFrameSize];
        uint8_t len;
        bool    valid;
    };
    static Slot slots[inter_cmd::kStateFragCount] {};

    /**
     * @brief 把状态分片写进对应槽位
     */
    static void UpdateSlot(inter_cmd::FrameType type, const uint8_t *data, uint8_t len)
    {
        for (uint8_t i = 0; i < inter_cmd::kStateFragCount; ++i)
        {
            const inter_cmd::StateFrag &f = inter_cmd::kStateFrags[i];
            if (f.dir != kDir || f.type != type) {
                continue;
            }
            if (len > f.len) {
                len = f.len;                       // 按契约长度截断
            }
            memcpy(slots[i].data, data, len);
            slots[i].len   = len;
            slots[i].valid = true;
            return;
        }
        LOG_WRN("not my direction/type: 0x%02x", static_cast<uint8_t>(type));
    }

    /**
     * @brief 聚合打包：各分片按布局偏移拼进同一帧
     * @return 有效字节数（0 = 还没有任何分片）
     */
    static uint8_t PackState(uint8_t *buf)
    {
        bool any = false;
        memset(buf, 0, kFrameSize);

        for (uint8_t i = 0; i < inter_cmd::kStateFragCount; ++i)
        {
            if (!slots[i].valid) {
                continue;
            }
            const inter_cmd::StateFrag &f = inter_cmd::kStateFrags[i];
            if (f.dir != kDir) {
                continue;
            }
            if (static_cast<uint16_t>(f.offset) + slots[i].len > kFrameSize) {
                LOG_WRN("state slot %u overflow", i);
                continue;
            }
            memcpy(buf + f.offset, slots[i].data, slots[i].len);
            any = true;
        }
        // 定长发送：接收端按固定偏移解析，短帧会让尾部区域读到未定义字节
        return any ? kStateLen : 0;
    }

    static void SendFrame(uint16_t id, const uint8_t *data, uint8_t len)
    {
        if (len > CAN_MAX_DLEN) {
            return;
        }
        can_frame tx{};
        tx.id    = id;
        tx.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;   // FD 帧 + 数据段 2Mbps（BRS）
        tx.dlc   = can_bytes_to_dlc(len);
        memcpy(tx.data, data, len);
        mcu_inter_can.Send(&tx);
    }

    static void Task(void*, void*, void*)
    {
        static constexpr uint32_t kPeriodMs = 1;

        for (;;)
        {
            const int64_t tick_start = k_uptime_get();

            // 1) 清空队列：各分片更新缓存
            topic::to_mcu_tx::Message ev{};
            while (k_msgq_get(&user_can2_msgq, &ev, K_NO_WAIT) == 0)
            {
                UpdateSlot(static_cast<inter_cmd::FrameType>(ev.tag), ev.data, ev.len);
            }

            // 2) 聚合状态帧：一帧装齐所有分片最新值
            uint8_t buf[kFrameSize];
            const uint8_t len = PackState(buf);
            if (len > 0) {
                SendFrame(kTxId, buf, len);
            }

            const int64_t elapsed = k_uptime_get() - tick_start;
            const int64_t remain  = static_cast<int64_t>(kPeriodMs) - elapsed;
            if (remain > 0) {
                k_msleep(remain);
            }
        }
    }

    bool thread_init()
    {
        const device *dev = DEVICE_DT_GET(DT_ALIAS(user_can2));
        if (!device_is_ready(dev)) {
            LOG_ERR("user_can2 not ready");
            return false;
        }

        const can_filter filter{.id = 0, .mask = 0, .flags = 0};
        // 必须让控制器进入 FD 模式（FDOE/BRSE），否则 can_send 会返回 -ENOTSUP
        if (!mcu_inter_can.Init(dev, filter, CAN_MODE_FD)) {
            LOG_ERR("mcu_inter_can init fail");
            return false;
        }
        mcu_inter_can.SetRxCallback(user_can2_rx_callback);   // 分发入口（.can_rx2）
        LOG_INF("mcu_inter_can ready");
        return true;
    }

    bool thread_start()
    {
        thread_.Start(Task, ThreadPrio::High);
        return true;
    }

    REGISTER_INIT  (thread_init,  MidInit,    Mid, "mcu_inter_init");
    REGISTER_THREAD(thread_start, MidThread,  Mid, "mcu_inter_start");
}
