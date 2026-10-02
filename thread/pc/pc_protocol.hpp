/**
 * @file pc_protocol.hpp
 * @author Rookie002-start
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
 * crc16 = 表驱动 CRC16（DJI/RoboMaster 风格：poly 0x1021 的反射表 0x8408，
 *         init 0xFFFF，输入/输出按位反射，末尾不异或；标准名 CRC-16/MCRF4XX）
 *         覆盖范围：帧头起、到 crc16 字段之前的所有字节（见 pc_proto::Crc16）
 *         ⚠️ 它**不等于** CRC16-CCITT-FALSE：同一串 "123456789" 本实现得 0x6F91、
 *            CCITT-FALSE 得 0x29B1。PC 侧两个方向都必须用同一张 wCRC_Table，
 *            否则整帧会被判坏帧丢弃（发送方算的值和接收方校验的值必须一致）。
 *
 * ## 与 PC 侧对齐（三处必须一致，任一处不一致 → 整帧被判坏帧丢弃）
 *
 *   ① 结构体 1 字节对齐：本文件用 `#pragma pack(push,1)/(pop)` 包住，
 *      PC 侧同样要 `#pragma pack(1)`，不要用默认对齐 —— 默认对齐会在 mode 后面、
 *      crc16 前面插填充字节。注意：只给外层结构体加 `__attribute__((packed))`
 *      **不够**（内层匿名结构体仍按自己的对齐规则排布），必须用 pack(1)。
 *   ② CRC 算法、参数（同一张 wCRC_Table）与覆盖范围同上；
 *   ③ 小端字节序（MCU 与 PC 都是小端，用同样的结构体即可）。
 *
 * 帧长由 static_assert 锁死：上板→PC = 43B，PC→上板 = 29B。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <stddef.h>
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

    // ---- CRC16：表驱动（DJI/RoboMaster 风格），PC 侧必须用同一张表 ----
    //
    // 表由 poly 0x1021 的反射形式（0x8408）生成，配合
    //     wCRC = (wCRC >> 8) ^ wCRC_Table[(wCRC ^ chData) & 0xFF]
    // 逐字节推进，即 init=0xFFFF、输入/输出反射、末尾不异或 → CRC-16/MCRF4XX。
    // 校验值（可用于和 PC 侧对齐）：ASCII "123456789" → 0x6F91。
    // 注意：CRC16-CCITT-FALSE（同样 poly/init，但不反射）在同一串数据上是 0x29B1，
    // 两者不通用 —— 换实现就必须两端一起换，否则全部判坏帧。
    inline constexpr uint16_t wCRC_Table[256] = {
        0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
        0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
        0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
        0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
        0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
        0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
        0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
        0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
        0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
        0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
        0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
        0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
        0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
        0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
        0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
        0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
        0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
        0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
        0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
        0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
        0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
        0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
        0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
        0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
        0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
        0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
        0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
        0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
        0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
        0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
        0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
        0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
    };

    /**
     * @brief 表驱动 CRC16（DJI/RoboMaster 风格）
     * @param pchMessage 数据首地址（NULL 返回 0xFFFF）
     * @param dwLength   参与计算的字节数
     * @param wCRC       起始值（本协议用 0xFFFF）
     * @return CRC（发送时按小端写进 crc16 字段）
     */
    inline uint16_t get_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
    {
        if (pchMessage == nullptr) {
            return 0xFFFFu;
        }
        while (dwLength != 0u) {
            const uint8_t chData = *pchMessage++;
            wCRC = static_cast<uint16_t>(wCRC >> 8) ^
                   wCRC_Table[(wCRC ^ static_cast<uint16_t>(chData)) & 0x00FFu];
            --dwLength;
        }
        return wCRC;
    }

    /// 只读数据（接收校验）用的重载，算法与上面完全一致
    inline uint16_t get_crc16_check_sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
    {
        return get_crc16_check_sum(const_cast<uint8_t *>(pchMessage), dwLength, wCRC);
    }

    /**
     * @brief 从 0xFFFF 起算的 CRC16（协议默认入口）
     * @param data 数据首地址（帧头起）
     * @param len  字节数（到 crc16 之前为止）
     */
    inline uint16_t Crc16(const uint8_t *data, uint32_t len)
    {
        return get_crc16_check_sum(data, len, 0xFFFFu);
    }

    /// 整帧 CRC 覆盖范围：帧头起、到 crc16 字段之前（用 offsetof，结构体变了自动跟随）
    template <typename T>
    inline uint16_t FrameCrc(const T &frame)
    {
        return Crc16(reinterpret_cast<const uint8_t *>(&frame),
                     static_cast<uint32_t>(offsetof(T, crc16)));
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
