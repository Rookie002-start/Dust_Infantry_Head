/**
 * @file trd_mcu_inter.cpp
 * @author qingyu
 * @brief 上板 → 下板发送线程（经典 CAN 拆帧）
 *        - 入队的状态分片存进【本线程私有】缓存，按 Dir::Up 布局聚合成 61B 聚合帧
 *        - 聚合帧按字节拆成 8 帧（0x100 ~ 0x107）周期发出
 *
 * ⚠️ 本线程是 user-can2 总线的所有者：初始化与收帧分发入口都在这里。
 *    接收侧（thread/inter_rx）只注册 CAN_RX_HANDLER，**不要**再次 Can::Init() /
 *    SetRxCallback()，否则会重复注册过滤器并覆盖分发入口。
 *
 * ⚠️ 总线带宽（经典 CAN 1Mbps，8B 帧 ≈ 121µs）：
 *    本方向 8 帧 ≈ 0.94ms + 对端 2 帧 ≈ 0.21ms = 一轮 ≈ 1.15ms，
 *    所以周期不能取 1ms（会 115% 占满总线）。当前 kPeriodMs = 2（约 57%）；
 *    实测出现丢帧/错误帧就改成 5ms（约 23%），或压缩字段减少帧数。
 *
 * @version 0.6
 * @date 2026-09-15
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

    static constexpr inter_cmd::Dir kDir       = inter_cmd::Dir::Up;     // 本线程发送方向
    static constexpr uint8_t        kFrameSize = 64;                     // 聚合帧缓冲上限
    static constexpr uint8_t        kStateLen  = inter_cmd::FrameLen(kDir);    // 61
    static constexpr uint8_t        kSliceNum  = inter_cmd::FrameCount(kDir);  // 8
    static constexpr uint32_t       kPeriodMs  = 2;                      // 见文件头带宽说明

    // ---- 私有分片缓存：按契约布局表下标索引（本线程独有，不共享 → 无需锁）----
    struct Slot
    {
        uint8_t data[kFrameSize];
        uint8_t len;
        bool    valid;
    };
    static Slot slots[inter_cmd::kStateFragCount] {};

    /// 更新本方向的分片缓存
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

    /// 按契约表把本方向分片拼成聚合帧（返回 0 = 还没有任何分片）
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
        // 定长：接收端按固定偏移解析，短帧会让尾部区域读到未定义字节
        return any ? kStateLen : 0;
    }

    /// 发一帧经典 CAN（≤8 字节，无 FD 标志）
    static void SendSlice(uint16_t id, const uint8_t *data, uint8_t len)
    {
        if (len == 0 || len > inter_cmd::kClassicPayload) {
            return;
        }
        can_frame tx{};
        tx.id    = id;
        tx.flags = 0;                       // 经典 CAN：无 FDF / BRS
        tx.dlc   = len;                     // 经典 CAN 的 DLC 就是字节数 0~8
        memcpy(tx.data, data, len);
        mcu_inter_can.Send(&tx);
    }

    /// 按固定映射把聚合帧拆成多帧发出（序号 ↔ 字节段一一对应）
    static void SendState(const uint8_t *buf)
    {
        for (uint8_t i = 0; i < kSliceNum; ++i)
        {
            const uint8_t n = inter_cmd::SliceLen(kDir, i);
            if (n == 0) {
                continue;
            }
            SendSlice(inter_cmd::SliceId(kDir, i),
                      buf + static_cast<uint8_t>(i * inter_cmd::kClassicPayload), n);
        }
    }

    static void Task(void*, void*, void*)
    {
        for (;;)
        {
            const int64_t tick_start = k_uptime_get();

            // 1) 清空队列：本方向各分片更新缓存
            topic::to_mcu_tx::Message ev{};
            while (k_msgq_get(&user_can2_msgq, &ev, K_NO_WAIT) == 0)
            {
                UpdateSlot(static_cast<inter_cmd::FrameType>(ev.tag), ev.data, ev.len);
            }

            // 2) 聚合本方向状态帧 → 拆帧发出
            uint8_t buf[kFrameSize];
            const uint8_t len = PackState(buf);
            if (len > 0) {
                SendState(buf);
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

        // 经典 CAN：不加 CAN_MODE_FD
        const can_filter filter{.id = 0, .mask = 0, .flags = 0};
        if (!mcu_inter_can.Init(dev, filter)) {
            LOG_ERR("mcu_inter_can init fail");
            return false;
        }
        // 收帧分发入口：接收侧（thread/inter_rx）的 CAN_RX_HANDLER 依赖它
        mcu_inter_can.SetRxCallback(user_can2_rx_callback);
        LOG_INF("inter tx ready (dir=%s, %u frames, period=%ums)",
                (kDir == inter_cmd::Dir::Up) ? "up" : "down",
                static_cast<unsigned>(kSliceNum), static_cast<unsigned>(kPeriodMs));
        return true;
    }

    bool thread_start()
    {
        thread_.Start(Task, ThreadPrio::High);
        return true;
    }

    REGISTER_INIT  (thread_init,  MidInit,    High, "mcu_inter_init");
    REGISTER_THREAD(thread_start, MidThread,  High, "mcu_inter_start");
}
