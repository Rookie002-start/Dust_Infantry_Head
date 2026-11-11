/**
 * @file latency_trace.cpp
 * @brief Software-only upper/lower latency aggregation.
 */

#include "latency_trace.hpp"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

LOG_MODULE_REGISTER(latency_trace, LOG_LEVEL_INF);

namespace diagnostics::latency
{
namespace
{
    constexpr uint8_t  kSeqSlots       = 16;
    constexpr uint32_t kPendingMaxMs   = 24;    // < 16 frames at 500 Hz = 32 ms
    constexpr uint32_t kReportPeriodMs = 2000;
    constexpr uint32_t kBinWidthUs     = 10;
    constexpr uint32_t kBinCount       = 2001;  // 0..20 ms, final bin is overflow
    constexpr uint32_t kMaxBinValueUs  = (kBinCount - 2) * kBinWidthUs;

    struct TxPending
    {
        uint32_t source_cycles;
        uint32_t version;
        bool     valid;
    };

    struct UpperPending
    {
        uint32_t stage_us;
        uint32_t completed_ms;
        bool     valid;
    };

    struct Report
    {
        uint32_t yaw_count;
        uint32_t pitch_count;
        uint32_t yaw_p50;
        uint32_t yaw_p95;
        uint32_t yaw_p99;
        uint32_t yaw_max;
        uint32_t pitch_p50;
        uint32_t pitch_p95;
        uint32_t pitch_p99;
        uint32_t pitch_max;
        uint32_t upper_avg_us;
        uint32_t lower_yaw_avg_us;
        uint32_t lower_pitch_avg_us;
        uint32_t yaw_overflow;
        uint32_t pitch_overflow;
    };

    static struct k_spinlock s_lock;
    static TxPending       s_tx{};
    static UpperPending    s_pending[kSeqSlots]{};
    static bool            s_arm;
    static uint32_t        s_source_cycles;
    static uint32_t        s_version;
    static uint8_t         s_marker_bits;

    static uint16_t s_yaw_hist[kBinCount]{};
    static uint16_t s_pitch_hist[kBinCount]{};
    static uint32_t s_yaw_count;
    static uint32_t s_pitch_count;
    static uint32_t s_yaw_max;
    static uint32_t s_pitch_max;
    static uint32_t s_yaw_overflow;
    static uint32_t s_pitch_overflow;
    static uint64_t s_upper_sum_us;
    static uint64_t s_lower_yaw_sum_us;
    static uint64_t s_lower_pitch_sum_us;
    static uint32_t s_last_report_ms;

    uint32_t BinOf(uint32_t us)
    {
        if (us > kMaxBinValueUs) {
            return kBinCount - 1;
        }
        return us / kBinWidthUs;
    }

    uint32_t BinValueUs(uint32_t bin)
    {
        return bin == (kBinCount - 1) ? kMaxBinValueUs + kBinWidthUs
                                      : bin * kBinWidthUs;
    }

    void AddSample(uint16_t *hist, uint32_t &count, uint32_t &max_us,
                   uint32_t &overflow, uint32_t us)
    {
        const uint32_t bin = BinOf(us);
        ++hist[bin];
        ++count;
        if (us > max_us) {
            max_us = us;
        }
        if (bin == (kBinCount - 1)) {
            ++overflow;
        }
    }

    uint32_t Percentile(const uint16_t *hist, uint32_t count, uint32_t pct)
    {
        if (count == 0u) {
            return 0u;
        }
        const uint32_t target = (count * pct + 99u) / 100u;
        uint32_t seen = 0u;
        for (uint32_t i = 0; i < kBinCount; ++i) {
            seen += hist[i];
            if (seen >= target) {
                return BinValueUs(i);
            }
        }
        return BinValueUs(kBinCount - 1);
    }

    Report TakeReport(uint32_t now_ms)
    {
        Report r{};
        r.yaw_count       = s_yaw_count;
        r.pitch_count     = s_pitch_count;
        r.yaw_p50         = Percentile(s_yaw_hist, s_yaw_count, 50);
        r.yaw_p95         = Percentile(s_yaw_hist, s_yaw_count, 95);
        r.yaw_p99         = Percentile(s_yaw_hist, s_yaw_count, 99);
        r.yaw_max         = s_yaw_max;
        r.pitch_p50       = Percentile(s_pitch_hist, s_pitch_count, 50);
        r.pitch_p95       = Percentile(s_pitch_hist, s_pitch_count, 95);
        r.pitch_p99       = Percentile(s_pitch_hist, s_pitch_count, 99);
        r.pitch_max       = s_pitch_max;
        r.yaw_overflow    = s_yaw_overflow;
        r.pitch_overflow  = s_pitch_overflow;
        r.upper_avg_us       = s_yaw_count
            ? static_cast<uint32_t>(s_upper_sum_us / s_yaw_count) : 0u;
        r.lower_yaw_avg_us   = s_yaw_count
            ? static_cast<uint32_t>(s_lower_yaw_sum_us / s_yaw_count) : 0u;
        r.lower_pitch_avg_us = s_pitch_count
            ? static_cast<uint32_t>(s_lower_pitch_sum_us / s_pitch_count) : 0u;

        memset(s_yaw_hist, 0, sizeof(s_yaw_hist));
        memset(s_pitch_hist, 0, sizeof(s_pitch_hist));
        s_yaw_count = 0u;
        s_pitch_count = 0u;
        s_yaw_max = 0u;
        s_pitch_max = 0u;
        s_yaw_overflow = 0u;
        s_pitch_overflow = 0u;
        s_upper_sum_us = 0u;
        s_lower_yaw_sum_us = 0u;
        s_lower_pitch_sum_us = 0u;
        s_last_report_ms = now_ms;
        return r;
    }
} // namespace

void OnPcFrame(uint32_t source_cycles, uint32_t version)
{
    k_spinlock_key_t key = k_spin_lock(&s_lock);
    s_source_cycles = source_cycles;
    s_version       = version;
    s_marker_bits   = static_cast<uint8_t>((version & 0x0Fu) << 4);
    s_arm           = true;
    k_spin_unlock(&s_lock, key);
}

uint8_t MarkerBits()
{
    static uint8_t cached;
    k_spinlock_key_t key = k_spin_lock(&s_lock);
    cached = s_marker_bits;
    k_spin_unlock(&s_lock, key);
    return cached;
}

void BeforeCanSend()
{
    k_spinlock_key_t key = k_spin_lock(&s_lock);
    if (s_arm) {
        s_tx.source_cycles = s_source_cycles;
        s_tx.version       = s_version;
        s_tx.valid         = true;
        s_arm = false;
    }
    k_spin_unlock(&s_lock, key);
}

void OnCanSubmitted()
{
    k_spinlock_key_t key = k_spin_lock(&s_lock);
    if (!s_tx.valid) {
        k_spin_unlock(&s_lock, key);
        return;
    }
    const uint32_t elapsed =
        k_cyc_to_us_floor32(k_cycle_get_32() - s_tx.source_cycles);
    const uint8_t slot = static_cast<uint8_t>(s_tx.version & 0x0Fu);
    s_pending[slot].stage_us     = elapsed;
    s_pending[slot].completed_ms = k_uptime_get_32();
    s_pending[slot].valid        = true;
    s_tx.valid = false;
    k_spin_unlock(&s_lock, key);
}

void OnCanSendFailed()
{
    k_spinlock_key_t key = k_spin_lock(&s_lock);
    s_tx.valid = false;
    k_spin_unlock(&s_lock, key);
}

void OnLowerTrace(const inter_cmd::TraceState &trace)
{
    if ((trace.valid_mask &
         (inter_cmd::kTraceValidYaw | inter_cmd::kTraceValidPitch)) == 0u) {
        return;
    }

    const uint32_t now_ms = k_uptime_get_32();
    Report report{};
    bool have_report = false;

    k_spinlock_key_t key = k_spin_lock(&s_lock);
    const uint8_t slot = static_cast<uint8_t>(trace.seq & 0x0Fu);
    UpperPending &p = s_pending[slot];
    if (p.valid && (now_ms - p.completed_ms) <= kPendingMaxMs) {
        p.valid = false;
        s_upper_sum_us += p.stage_us;

        if ((trace.valid_mask & inter_cmd::kTraceValidYaw) != 0u) {
            const uint32_t total = p.stage_us + trace.yaw_us;
            s_lower_yaw_sum_us += trace.yaw_us;
            AddSample(s_yaw_hist, s_yaw_count, s_yaw_max, s_yaw_overflow, total);
        }
        if ((trace.valid_mask & inter_cmd::kTraceValidPitch) != 0u) {
            const uint32_t total = p.stage_us + trace.pitch_us;
            s_lower_pitch_sum_us += trace.pitch_us;
            AddSample(s_pitch_hist, s_pitch_count, s_pitch_max,
                      s_pitch_overflow, total);
        }
    }

    if ((now_ms - s_last_report_ms) >= kReportPeriodMs) {
        report = TakeReport(now_ms);
        have_report = true;
    }
    k_spin_unlock(&s_lock, key);

    if (have_report) {
        LOG_INF("LT total seq=%u yaw n=%u avg(u/l/t)=%u/%u/%u "
                "p50=%u p95=%u p99=%u max=%u over=%u us",
                static_cast<unsigned>(trace.seq),
                static_cast<unsigned>(report.yaw_count),
                static_cast<unsigned>(report.upper_avg_us),
                static_cast<unsigned>(report.lower_yaw_avg_us),
                static_cast<unsigned>(report.upper_avg_us + report.lower_yaw_avg_us),
                static_cast<unsigned>(report.yaw_p50),
                static_cast<unsigned>(report.yaw_p95),
                static_cast<unsigned>(report.yaw_p99),
                static_cast<unsigned>(report.yaw_max),
                static_cast<unsigned>(report.yaw_overflow));
        LOG_INF("LT total seq=%u pitch n=%u avg(u/l/t)=%u/%u/%u "
                "p50=%u p95=%u p99=%u max=%u over=%u us",
                static_cast<unsigned>(trace.seq),
                static_cast<unsigned>(report.pitch_count),
                static_cast<unsigned>(report.upper_avg_us),
                static_cast<unsigned>(report.lower_pitch_avg_us),
                static_cast<unsigned>(report.upper_avg_us + report.lower_pitch_avg_us),
                static_cast<unsigned>(report.pitch_p50),
                static_cast<unsigned>(report.pitch_p95),
                static_cast<unsigned>(report.pitch_p99),
                static_cast<unsigned>(report.pitch_max),
                static_cast<unsigned>(report.pitch_overflow));
    }
}

} // namespace diagnostics::latency
