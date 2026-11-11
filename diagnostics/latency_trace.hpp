/**
 * @file latency_trace.hpp
 * @brief Upper-board latency trace collector.
 *
 * Enabled only by CONFIG_LATENCY_TRACE. The lower board reports its local
 * CAN-RX-to-motor-TX interval through the existing CAN downlink; this module
 * pairs it with the upper board's PC-RX-to-CAN-TX-complete interval.
 */

#pragma once

#include <stdint.h>
#include "inter_cmd.hpp"

namespace diagnostics::latency
{
    /// A new, valid PC auto-aim frame was consumed from pub_pc_to.
    void OnPcFrame(uint32_t source_cycles, uint32_t version);

    /// Four-bit sequence carried in CommState.flags[7:4].
    uint8_t MarkerBits();

    /// Arm timing for the frame about to be submitted to the CAN driver.
    void BeforeCanSend();

    /// The CAN driver accepted the frame into a TX mailbox/FIFO.
    void OnCanSubmitted();

    /// The CAN driver rejected the frame immediately.
    void OnCanSendFailed();

    /// Called when the lower board's TraceState fragment reaches the upper board.
    void OnLowerTrace(const inter_cmd::TraceState &trace);
}
