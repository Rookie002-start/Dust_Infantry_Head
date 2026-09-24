/**
 * @file pc_protocol.hpp
 * @author qingyu
 * @brief 上板 ↔ PC（自瞄）链路线缆协议：结构体 / 帧长 / CRC16
 * @version 1.0
 * @date 2026-09-23
 *
 * ## 传输与组帧
 *
 * USB CDC ACM 批量端点：字节流、没有消息边界 → **定长帧 + 双字节帧头 + CRC16** 自同步。
 * 两个方向都是定长、小端、1 字节对齐布局：
 *
 *     'S' 'P' | 载荷 | crc16(小端)
 *
 * crc16 = CRC16-CCITT-FALSE（poly 0x1021，init 0xFFFF，输入不反转，输出不异或）
 *         覆盖范围：帧头起、到 crc16 字段之前的所有字节（见 pc_proto::Crc16）
 *
 * ## 与 PC 侧对齐（三处必须一致，任一处不一致 → 整帧被判坏帧丢弃）
 *
 *   ① 结构体 1 字节对齐：本文件用 `#pragma pack(push,1)/(pop)` 包住，
 *      PC 侧同样要 `#pragma pack(1)`，不要用默认对齐 —— 默认对齐会在 mode 后面、
 *      crc16 前面插填充字节。注意：只给外层结构体加 `__attribute__((packed))`
 *      **不够**（内层匿名结构体仍按自己的对齐规则排布），必须用 pack(1)。
 *   ② CRC 参数与覆盖范围同上；
 *   ③ 小端字节序（MCU 与 PC 都是小端，用同样的结构体即可）。
 *
 * 帧长由 static_assert 锁死：上板→PC = 43B，PC→上板 = 29B。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <stdint.h>

/// 线缆布局：整段 1 字节对齐（PC 侧对应 `#pragma pack(1)`，见文件头说明）
#pragma pack(push, 1)

/**
 * @brief 上板 → PC：自瞄解算需要的本体状态（**全部来自 IMU**）
 *
 *   q / yaw / pitch ← pub_imu_to（框架 IMU 模块）
 *   bullet_speed    ← 下板实测弹速（pub_from_body，未接入时恒 0）
 *   bullet_count    ← 上板累计发弹数（发射机构线程计数，无该线程时恒 0）
 *   mode            ← 上板回显自瞄状态：0-空闲（没在用 PC 数据） 1-自瞄
 */
struct PCSendAutoAimData
{
    uint8_t head[2] = {'S','P'};

    uint8_t mode = 0;               // 0-空闲 1-自瞄

    float q[4];                     // 四元数姿态[w,x,y,z]

    struct
    {
        float yaw_ang;              // yaw轴角度
        float yaw_vel;              // yaw轴角速度
    } yaw;

    struct
    {
        float pitch_ang;            // pitch轴角度
        float pitch_vel;            // pitch轴角速度
    } pitch;

    struct
    {
        float bullet_speed;         // 子弹速度
        uint16_t bullet_count;      // 子弹累计发送次数
    } bullet;

    uint16_t crc16;                 // 校验位
};

/**
 * @brief PC → 上板：自瞄解算结果（只给下板用，经两板通信下发）
 *
 *   mode = 0 空闲（未锁定目标：上板不下发自瞄数据，下板回手动）
 *   mode = 1 自瞄不开火
 *   mode = 2 自瞄开火
 */
struct PCRecvAutoAimData
{
    uint8_t head[2] = {'S','P'};
    uint8_t mode = 0;           // 0-空闲 1-自瞄不开火 2-自瞄开火

    struct
    {
        float yaw_ang;          // yaw轴角度
        float yaw_vel;          // yaw轴角速度
        float yaw_acc;          // yaw轴角加速度
    } yaw;

    struct
    {
        float pitch_ang;        // pitch轴角度
        float pitch_vel;        // pitch轴角速度
        float pitch_acc;        // pitch轴角加速度
    } pitch;

    uint16_t crc16;             // 校验位
};

#pragma pack(pop)

static_assert(sizeof(PCSendAutoAimData) == 43, "PCSendAutoAimData layout changed (PC 侧需同步)");
static_assert(sizeof(PCRecvAutoAimData) == 29, "PCRecvAutoAimData layout changed (PC 侧需同步)");

namespace pc_proto
{
    // ---- 帧头 ----
    constexpr uint8_t kHead0 = 'S';
    constexpr uint8_t kHead1 = 'P';

    constexpr uint16_t kSendFrameLen = sizeof(PCSendAutoAimData);   // 43
    constexpr uint16_t kRecvFrameLen = sizeof(PCRecvAutoAimData);   // 29

    // ---- 接收侧 mode 取值 ----
    constexpr uint8_t kRecvModeIdle    = 0;   // 空闲：不下发自瞄数据
    constexpr uint8_t kRecvModeAim     = 1;   // 自瞄不开火
    constexpr uint8_t kRecvModeAimFire = 2;   // 自瞄开火

    // ---- 发送侧 mode 取值 ----
    constexpr uint8_t kSendModeIdle = 0;
    constexpr uint8_t kSendModeAim  = 1;

    // ---- 收包合理性上限（超过就判为坏帧，避免脏数据把云台抽飞）----
    constexpr float kMaxYawAng   = 10.0f;     // rad（允许跨圈的目标角）
    constexpr float kMaxPitchAng = 1.6f;      // rad
    constexpr float kMaxAngVel   = 60.0f;     // rad/s
    constexpr float kMaxAngAcc   = 400.0f;    // rad/s^2

    inline bool Finite(float v) { return __builtin_isfinite(v) != 0; }

    /**
     * @brief CRC16-CCITT-FALSE
     * @param data 数据首地址
     * @param len  字节数
     * @return CRC（发送时按小端写进 crc16 字段）
     */
    inline uint16_t Crc16(const uint8_t *data, uint16_t len)
    {
        uint16_t crc = 0xFFFFu;
        for (uint16_t i = 0; i < len; ++i)
        {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (uint8_t bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                      : static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }

    /// 整帧 CRC 覆盖范围：帧头起、到 crc16 之前
    template <typename T>
    inline uint16_t FrameCrc(const T &frame)
    {
        return Crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(T) - sizeof(uint16_t));
    }

    /// 接收帧合理性检查：mode 合法、所有浮点有限、在物理量程内
    inline bool RecvSane(const PCRecvAutoAimData &d)
    {
        if (d.mode > kRecvModeAimFire) {
            return false;
        }
        const float v[6] = { d.yaw.yaw_ang,   d.yaw.yaw_vel,   d.yaw.yaw_acc,
                             d.pitch.pitch_ang, d.pitch.pitch_vel, d.pitch.pitch_acc };
        for (float x : v) {
            if (!Finite(x)) {
                return false;
            }
        }
        return (__builtin_fabsf(d.yaw.yaw_ang) <= kMaxYawAng) &&
               (__builtin_fabsf(d.pitch.pitch_ang) <= kMaxPitchAng) &&
               (__builtin_fabsf(d.yaw.yaw_vel) <= kMaxAngVel) &&
               (__builtin_fabsf(d.pitch.pitch_vel) <= kMaxAngVel) &&
               (__builtin_fabsf(d.yaw.yaw_acc) <= kMaxAngAcc) &&
               (__builtin_fabsf(d.pitch.pitch_acc) <= kMaxAngAcc);
    }
}
