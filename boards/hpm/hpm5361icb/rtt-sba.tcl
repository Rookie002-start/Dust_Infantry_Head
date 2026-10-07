# HPM5361 上的「不停 CPU 读内存」helper —— 走调试模块的 SBA（system bus access）。
#
# 背景：本机 OpenOCD 的 RISC-V 目标在 hart 运行时读不了内存
#   （mdw 报 `Failed to read priv register` / `Failed to read memory via system bus`），
#   所以 tools/rtt_hpm.py 原来是 halt → 读缓冲 → resume，每轮都会把 CPU 停几十~几百毫秒。
#
# 但 DM 本身的 SBA 是好的：直接操作 DMI 的 sbcs/sbaddress0/sbdata0
#   （0x38 / 0x39 / 0x3c，和 hpm_common.cfg 里的 sba_read_mem 一致）
#   就能在 hart 全程 running 的情况下读写内存。实测：
#     * 读取结果与 halt 后的 mdw 逐字一致；
#     * 每个 DMI 事务约 3 ms（瓶颈是探针的 USB 往返，不是 JTAG 时钟：8 MHz 只快 12%）；
#     * 3 次 DMI/字 ≈ 0.5 KB/s，2 次 DMI/字（流水线）≈ 0.8~0.9 KB/s；
#     * 读取期间 `targets` 始终是 running。
#
# 注意（实测踩过的坑，sbcs 的位是逐位试出来的，和标准 RISC-V DM 不完全一样）：
#   * 想拿到 32 位访问，必须同时置 bit15 和 bit18：
#       0x20008407 -> 8 位，0x20028407 -> 16 位，0x20048407 -> 32 位；
#   * bit20（SDK 的 sba_read_mem 里会 OR 上的那一位）必须清掉，置上之后读出来是脏数据；
#   * 连读 sbdata0 不会前进（一直返回同一个字），autoinc / readonaddr / readondata 都不管用，
#     所以每个字都要重新写一次 sbaddress0；
#   * 写地址后第一次读 sbdata0 拿到的是**上一次**的结果（滞后一拍），要读两次、取第二次；
#   * 写路径（写 sbaddress0 + 写 sbdata0）落在正确地址，实测回读一致。
#   * ⚠️ 只能读 RAM 里的地址。访问 XIP flash（0x80000000 段）这类 SBA 到不了的地址，
#     会把 DM 的 SBA 引擎卡死（之后所有读都返回陈旧数据、清 sberror 也无效），
#     要恢复得做一次**整机复位**：`riscv dmi_write 0x10 0x3` 再 `riscv dmi_write 0x10 0x1`
#     （dmcontrol.ndmreset）或断电；实测只拉 SRST（`tools/rtt.sh -r` / OpenOCD `reset halt`）
#     清不掉。RTT 控制块/缓冲都在 RAM 里，正常使用踩不到这条。

proc rtt_sba_config {} {
    set b [riscv dmi_read 0x38]
    # 32 位访问（bit15 + bit18）；13~22 位全清。
    # ⚠️ 千万别想着"顺手清 sberror"去写 0x700000：这颗 DM 上 bit20 一置上去访问就变成 8 位，
    #    读出来全是脏数据（实测）。
    set want [expr {($b & ~0x7fc000) | 0x8000 | 0x40000}]
    riscv dmi_write 0x38 $want
    set rb [riscv dmi_read 0x38]
    if {($rb & 0x7fc000) != 0x48000} {
        error "SBA 访问宽度设置失败 (sbcs=0x[format %08x $rb])：SBA 引擎可能卡住了，需要整机复位/断电"
    }
    return $rb
}

# 读 count 个 32 位字，返回十六进制列表。期间不发 halt。
# 每字 3 次 DMI 事务（写地址 + 丢弃滞后的一拍 + 读数据），最稳，默认用它。
proc rtt_sba_read {addr count} {
    rtt_sba_config
    set out {}
    for {set i 0} {$i < $count} {incr i} {
        riscv dmi_write 0x39 [expr {($addr + $i * 4) & 0xffffffff}]
        riscv dmi_read 0x3c          ;# 第一次读回来的是上一次的结果
        lappend out [riscv dmi_read 0x3c]
    }
    return $out
}

# 流水线版：每字 2 次 DMI 事务（先写下一个地址，再读，读回的是上一个地址的字）。
# 比 rtt_sba_read 快 ~1.6 倍（0.8~0.9 KB/s vs 0.5 KB/s），也不越界访问、
# 不会碰到缓冲区之后的内存；但流水线的时序假设更强，所以 tools/rtt_hpm.py
# 会先用 rtt_sba_read 做基准比对，一致才切到这一版。
proc rtt_sba_read_fast {addr count} {
    rtt_sba_config
    riscv dmi_write 0x39 [expr {$addr & 0xffffffff}]
    riscv dmi_read 0x3c
    set out {}
    for {set i 0} {$i < $count - 1} {incr i} {
        riscv dmi_write 0x39 [expr {($addr + ($i + 1) * 4) & 0xffffffff}]
        lappend out [riscv dmi_read 0x3c]
    }
    lappend out [riscv dmi_read 0x3c]
    return $out
}

# 写一个 32 位字（RTT 的 RdOff 回执用）。期间不发 halt。
proc rtt_sba_write {addr value} {
    rtt_sba_config
    riscv dmi_write 0x39 [expr {$addr & 0xffffffff}]
    riscv dmi_write 0x3c [expr {$value & 0xffffffff}]
    return 0
}
