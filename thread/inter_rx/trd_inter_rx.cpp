/**
 * @file trd_inter_rx.cpp
 * @author qingyu
 * @brief 下板→上板接收线程
 *        - 状态帧（Dir::Down / 0x200）：ISR 只搬运 → 线程解析 + 超时判定 → zbus 发布
 *
 * 本组件只负责"收"：user-can2 总线由发送侧（thread/mcu_inter）持有并完成
 * Init + 设置收帧分发入口，这里只注册 CAN_RX_HANDLER。
 *
 * 消费者用法（1kHz 循环内）：
 * @code
 *   static topic::from_body::Message body{};
 *   if (zbus_chan_read(&pub_from_body, &body, K_NO_WAIT) != 0) {
 *       // 读失败 → 沿用上一轮值
 *   }
 *   if (!body.online) { // 安全降级 } else { // 使用 body.gimbal.yaw / body.shooter.bullet_speed }
 * @endcode
 *
 * @version 0.1
 * @date 2026-09-14
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Inter_Rx"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "from_body.hpp"
#include "inter_cmd.hpp"
#include "Irq_handlers.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(inter_rx, LOG_LEVEL_INF);

namespace thread::inter_rx
{
    static Thread<2048> thread_{};

    static constexpr inter_cmd::Dir kDir        = inter_cmd::Dir::Down;   // 本线程接收方向
    static constexpr uint32_t       kRxTimeoutMs = 30;                    // 超时判离线
    static constexpr uint32_t       kPollMs      = 1;

    // ---- ISR → 线程：最新原始状态帧（seqlock，避免读到撕裂的帧）----
    static struct
    {
        uint8_t  data[inter_cmd::kDownFrameLen];
        atomic_t seq;
    } raw_state;

    K_SEM_DEFINE(raw_sem, 0, 1);

    /// 状态帧入口：拷进最新帧槽 + 唤醒线程
    static void StateRxHandler(uint8_t *data)
    {
        atomic_inc(&raw_state.seq);                    // 奇数：写者进行中
        memcpy(raw_state.data, data, sizeof(raw_state.data));
        atomic_inc(&raw_state.seq);                    // 偶数：写完
        k_sem_give(&raw_sem);
    }

    // 注册到框架 CAN RX 分发段（.can_rx2，由 user_can2_rx_callback 按 ID 分发）
    CAN_RX_HANDLER(USER_RX_CAN2, inter_cmd::StateTxId(kDir), StateRxHandler, state_200);

    /// 取一份不撕裂的原始帧
    static void ReadRaw(uint8_t *out)
    {
        atomic_t seq;
        do
        {
            seq = atomic_get(&raw_state.seq);
            if (seq & 1U) {
                continue;                              // 写者进行中，重试
            }
            memcpy(out, raw_state.data, sizeof(raw_state.data));
        } while (atomic_get(&raw_state.seq) != seq);
    }

    /// 按契约布局表解析并发布（方向/偏移/长度都来自 inter_cmd.hpp 的同一张表）
    static void Publish(const uint8_t *raw, bool online, uint16_t age_ms)
    {
        topic::from_body::Message msg{};
        (void)inter_cmd::GetFrag(raw, kDir, inter_cmd::FrameType::StateGimbal,  msg.gimbal);
        (void)inter_cmd::GetFrag(raw, kDir, inter_cmd::FrameType::StateShooter, msg.shooter);
        msg.online = online;
        msg.age_ms = age_ms;
        (void)zbus_chan_pub(&pub_from_body, &msg, K_NO_WAIT);
    }

    static void Task(void*, void*, void*)
    {
        uint8_t  raw[inter_cmd::kDownFrameLen] {};
        uint32_t last_rx_ms  = 0;
        bool     ever_rx     = false;
        bool     online_prev = false;

        for (;;)
        {
            const bool     fresh = (k_sem_take(&raw_sem, K_MSEC(kPollMs)) == 0);
            const uint32_t now   = k_uptime_get_32();

            if (fresh)
            {
                ReadRaw(raw);
                last_rx_ms = now;
                ever_rx    = true;
            }

            const uint32_t age    = ever_rx ? (now - last_rx_ms) : UINT32_MAX;
            const bool     online = ever_rx && (age <= kRxTimeoutMs);

            // 只在「有新帧」或「在线状态翻转」时发布；离线时保留最后一帧数据
            if (fresh || online != online_prev)
            {
                const uint16_t age16 = (age > UINT16_MAX) ? UINT16_MAX
                                                          : static_cast<uint16_t>(age);
                Publish(raw, online, age16);

                if (online != online_prev) {
                    LOG_INF("lower board %s (age=%ums)", online ? "online" : "TIMEOUT",
                            static_cast<unsigned>(age16));
                }
                online_prev = online;
            }
        }
    }

    bool thread_init()
    {
        // 总线 Init / FD 模式 / 收帧分发入口由发送侧（thread/mcu_inter）负责
        LOG_INF("inter_rx ready (timeout=%ums)", kRxTimeoutMs);
        return true;
    }

    bool thread_start()
    {
        thread_.Start(Task, ThreadPrio::High);
        return true;
    }

    REGISTER_INIT  (thread_init,  MidInit,    High, "inter_rx_init");
    REGISTER_THREAD(thread_start, MidThread,  High, "inter_rx_start");
}
