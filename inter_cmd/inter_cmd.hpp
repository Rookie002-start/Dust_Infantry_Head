/**
 * @file inter_cmd.hpp
 * @author qingyu
 * @brief 上下板通信接口契约（共享：任意业务线程发数据都走这里）
 *
 * 两个方向（各自一个聚合状态帧）：
 *   · Dir::Up   上板 → 下板：CAN ID 0x100
 *   · Dir::Down 下板 → 上板：CAN ID 0x200
 *
 * 只有状态分片，没有命令帧：业务线程 PostFrame() 分片 → 发送线程收集 →
 * 按 kStateFrags[] 表聚合成一帧 → 定长周期发出。
 * 入队携带的 tag 只是"分片类型"标签，不是上线用的 CAN ID（上线 ID 由 StateTxId(dir) 给出）。
 *
 * 新增分片 = 加一枚举 + 一个结构体 + 表里加一行；帧长/校验/打包/解析全自动跟随。
 *
 * @version 0.5
 * @date 2026-09-14
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "to_mcu_tx.hpp"

namespace inter_cmd
{
    /// 通信方向
    enum class Dir : uint8_t
    {
        Up   = 0,   // 上板 → 下板
        Down = 1,   // 下板 → 上板
    };

    /// 状态分片类型注册表：协议文档同步维护
    enum class FrameType : uint8_t
    {
        // 上板 → 下板
        StateComm      = 0x20,   // CommState（底盘/云台指令汇总）
        StateAutoAim   = 0x21,   // AutoAimState
        StateImu       = 0x22,   // ImuState

        // 下板 → 上板
        StateGimbal    = 0x30,   // GimbalState（实测云台角，反馈）
        StateShooter   = 0x31,   // ShooterState（子弹速度）
    };

    /// 聚合状态帧的 CAN ID（唯一事实来源）
    constexpr uint16_t StateTxId(Dir d)
    {
        return (d == Dir::Up) ? 0x100u : 0x200u;
    }

    // ============ 载荷结构体（packed：固定线缆布局，两端定义需完全一致）============

    /// 状态分片（Up）：底盘/云台指令汇总（21B，偏移 0）
    struct __attribute__((packed)) CommState
    {
        float   yaw_angle;      // 云台偏航角【指令】(rad)
        float   pitch_angle;    // 云台俯仰角【指令】(rad)
        float   chassis_vx;     // 底盘前后
        float   chassis_vy;     // 底盘左右
        float   chassis_rot;    // 底盘自转
        uint8_t chassis_spin;   // 小陀螺（三态）
    };

    /// 状态分片（Up）：自瞄数据（24B，偏移 21）
    struct __attribute__((packed)) AutoAimState
    {
        uint8_t yaw_angle[4];
        uint8_t yaw_omega[4];
        uint8_t yaw_torque[4];
        uint8_t pitch_angle[4];
        uint8_t pitch_omega[4];
        uint8_t pitch_torque[4];
    };

    /// 状态分片（Up）：IMU 数据（16B，偏移 45）
    struct __attribute__((packed)) ImuState
    {
        uint8_t total_yaw_angle[4];
        uint8_t pitch_angle[4];
        uint8_t yaw_omega[4];
        uint8_t pitch_omega[4];
    };

    /// 状态分片（Down）：实测云台角【反馈】（8B，偏移 0）
    /// ⚠️ 与 Up 方向 CommState 的 yaw/pitch 语义相反：那边是指令，这边是实测
    struct __attribute__((packed)) GimbalState
    {
        float yaw;      // rad
        float pitch;    // rad
    };

    /// 状态分片（Down）：子弹速度（4B，偏移 8）
    struct __attribute__((packed)) ShooterState
    {
        float bullet_speed;   // 单位按协议约定
    };

    // ============ 聚合帧布局（唯一事实来源）============

    // 各分片在各自方向聚合帧中的偏移（协议事实，命名便于对照协议文档）
    constexpr uint8_t kOffComm    = 0;    // CommState    21B → 0..20
    constexpr uint8_t kOffAutoAim = 21;   // AutoAimState 24B → 21..44
    constexpr uint8_t kOffImu     = 45;   // ImuState     16B → 45..60

    constexpr uint8_t kOffGimbal  = 0;    // GimbalState   8B → 0..7
    constexpr uint8_t kOffShooter = 8;    // ShooterState  4B → 8..11

    /// 一个状态分片的布局描述
    struct StateFrag
    {
        Dir       dir;      // 属于哪个方向
        FrameType type;     // 分片类型
        uint8_t   offset;   // 在该方向聚合帧中的字节偏移
        uint8_t   len;      // 分片长度
    };

    /// 聚合帧布局表
    constexpr StateFrag kStateFrags[] = {
        // 上板 → 下板
        { Dir::Up,   FrameType::StateComm,    kOffComm,    sizeof(CommState)    },
        { Dir::Up,   FrameType::StateAutoAim, kOffAutoAim, sizeof(AutoAimState) },
        { Dir::Up,   FrameType::StateImu,     kOffImu,     sizeof(ImuState)     },
        // 下板 → 上板
        { Dir::Down, FrameType::StateGimbal,  kOffGimbal,  sizeof(GimbalState)  },
        { Dir::Down, FrameType::StateShooter, kOffShooter, sizeof(ShooterState) },
    };

    constexpr uint8_t kStateFragCount = sizeof(kStateFrags) / sizeof(kStateFrags[0]);

    /// 某方向聚合帧总长 = 该方向所有分片尾端的最大值（自动推导，无需手改）
    constexpr uint8_t FrameLen(Dir d)
    {
        uint8_t end = 0;
        for (uint8_t i = 0; i < kStateFragCount; ++i) {
            if (kStateFrags[i].dir != d) {
                continue;
            }
            const uint8_t e =
                static_cast<uint8_t>(kStateFrags[i].offset + kStateFrags[i].len);
            if (e > end) {
                end = e;
            }
        }
        return end;
    }

    constexpr uint8_t kUpFrameLen   = FrameLen(Dir::Up);     // 48
    constexpr uint8_t kDownFrameLen = FrameLen(Dir::Down);   // 12

    /// 编译期校验：分片不得越界、同方向不得重叠、类型不得重复
    constexpr bool StateFragsValid()
    {
        for (uint8_t i = 0; i < kStateFragCount; ++i) {
            const uint16_t end_i = kStateFrags[i].offset + kStateFrags[i].len;
            if (end_i > 64) {
                return false;                                   // 超出 CAN FD 上限
            }
            for (uint8_t j = i + 1; j < kStateFragCount; ++j) {
                if (kStateFrags[i].type == kStateFrags[j].type) {
                    return false;                               // 类型重复，方向/偏移会歧义
                }
                if (kStateFrags[j].dir != kStateFrags[i].dir) {
                    continue;                                   // 不同方向不参与重叠判断
                }
                const uint16_t beg_j = kStateFrags[j].offset;
                const uint16_t end_j = beg_j + kStateFrags[j].len;
                if (beg_j < end_i && kStateFrags[i].offset < end_j) {
                    return false;                               // 同方向分片重叠
                }
            }
        }
        return true;
    }

    static_assert(kUpFrameLen   <= 64, "Up state frame exceeds CAN FD max 64B");
    static_assert(kDownFrameLen <= 64, "Down state frame exceeds CAN FD max 64B");
    static_assert(StateFragsValid(),   "state fragment layout invalid: overlap / out of range / dup");

    /// 从某方向的聚合帧里取出指定分片（接收侧用）
    template <typename T>
    inline bool GetFrag(const uint8_t *frame, Dir dir, FrameType type, T &out)
    {
        for (uint8_t i = 0; i < kStateFragCount; ++i) {
            if (kStateFrags[i].dir  != dir ||
                kStateFrags[i].type != type ||
                kStateFrags[i].len  != sizeof(T)) {
                continue;
            }
            memcpy(&out, frame + kStateFrags[i].offset, sizeof(T));
            return true;
        }
        return false;
    }

    // ============ 发送接口（多线程可并发调用，底层 k_msgq 多生产者）============

    /**
     * @brief 发布一个状态分片（发送线程收集后聚合成帧）
     * @note  载荷结构体须为裸数据（无指针/虚表），两端定义一致；
     *        浮点字段两端字节序/格式一致（都是小端 IEEE754 即安全）。
     */
    template <typename T>
    inline void PostFrame(FrameType type, const T &payload)
    {
        static_assert(sizeof(T) <= 64, "payload exceeds CAN FD max 64B");
        topic::to_mcu_tx::Message msg{};
        msg.tag = static_cast<uint8_t>(type);      // 分片类型标签（非上线 ID）
        msg.len = sizeof(T);
        memcpy(msg.data, &payload, sizeof(T));
        k_msgq_put(&user_can2_msgq, &msg, K_NO_WAIT);
    }
}
