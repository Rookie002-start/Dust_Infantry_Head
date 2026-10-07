#!/usr/bin/env python3
"""HPM5361(RISC-V) 上的 SEGGER RTT 主机侧读取器 —— 走 OpenOCD telnet。

为什么需要它：
  OpenOCD 的「目标侧 RTT」（`rtt setup` / `rtt start` / `rtt server`）只对 ARM(Cortex-M) 目标注册，
  RISC-V 目标上没有这些命令；而且本机的 OpenOCD 在 HPM5361 上**运行中读不了内存**
  （sysbus / abstract / progbuf 都报 `Failed to read priv register`）。
  所以这里直接按 SEGGER RTT 协议自己读控制块和环形缓冲：

  默认走调试模块的 SBA（system bus access，见 boards/hpm/hpm5361icb/rtt-sba.tcl）：
  直接操作 DMI 的 sbcs/sbaddress0/sbdata0 读写内存，**hart 全程 running**，
  CPU 不会停（实测 0.5~0.9 KiB/s，读到的内容与 halt 后的 mdw 逐字一致）。

  刷新节奏：默认 1000ms 一轮，一轮里把积压的数据连续读空（最多 --drain-ms），
  读掉的时间算在周期里 —— 所以"新日志最多晚 --poll-ms 出现"，想更实时给 200~500。
  吞吐瓶颈是探针的 USB 往返（每个 DMI 事务约 3ms），启动时会自动比对
  流水线协议（2 次 DMI/字，~0.9KiB/s）和稳妥协议（3 次 DMI/字，~0.5KiB/s），
  一致就用快的那个。

  只有 SBA 不可用时才回退到老办法（每轮都会让 CPU 停几十~几百毫秒）：
      halt → 读控制块/上行缓冲 → 把读到的位置写回 RdOff → resume

  目标是 HPM 板时用它；STM32 板（rm_c / dm_mc02）用 OpenOCD 原生的 rtt 命令更顺，
  见 tools/rtt.sh / docs/segger_rtt.md。

用法：
    tools/rtt_hpm.py                       # 自动找 build/zephyr/zephyr.elf，连 127.0.0.1:4444
    tools/rtt_hpm.py -e /tmp/build-hpm/zephyr/zephyr.elf
    tools/rtt_hpm.py --once                # 只读一次控制块信息（排查用）
    tools/rtt_hpm.py --poll-ms 1000 --log rtt.log
    tools/rtt_hpm.py --halt                # 强制 halt→读→resume（排查 SBA 时对比用）

前提：另开一个终端先起 OpenOCD（tools/rtt.sh 会自动做），telnet 口默认 4444。
"""

import argparse
import glob
import os
import re
import select
import signal
import socket
import struct
import sys
import time

# ---- SEGGER RTT 控制块布局（32 位目标：指针/unsigned 都是 4 字节）----
CB_OFF_MAXUP = 16          # char acID[16]; int MaxNumUpBuffers; int MaxNumDownBuffers;
CB_OFF_AUP0 = 24           # SEGGER_RTT_BUFFER_UP aUp[]
UP_OFF_SNAME = 0
UP_OFF_PBUF = 4
UP_OFF_SIZE = 8
UP_OFF_WR = 12
UP_OFF_RD = 16
UP_OFF_FLAGS = 20
UP_STRUCT_SIZE = 24
RTT_ID = b"SEGGER RTT"

TIMINGS = []          # RTT_DEBUG=1 时记录每条 openocd 命令的耗时


def find_symbol(elf: str, sym: str):
    """用 SDK 里的任意 readelf 找符号地址（兼容 arm / riscv 两种工具链）。"""
    sdk = os.environ.get("ZEPHYR_SDK_INSTALL_DIR",
                         os.path.expanduser("~/zephyrproject/zephyr-sdk-1.0.1"))
    for tool in sorted(glob.glob(f"{sdk}/gnu/*/bin/*-readelf")):
        out = os.popen(f"'{tool}' -sW '{elf}' 2>/dev/null").read()
        for line in out.splitlines():
            fields = line.split()
            # 形如：  123: 00085780     4 OBJECT  GLOBAL DEFAULT   18 _SEGGER_RTT
            if len(fields) >= 8 and fields[-1] == sym:
                try:
                    return int(fields[1], 16)
                except ValueError:
                    continue
    return None


class OcdTelnet:
    """极简 OpenOCD telnet 客户端：发一行，读到提示符 '>' 为止。"""

    def __init__(self, host: str, port: int, timeout: float = 2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setblocking(False)       # 用 select 控制等待，避免 recv 白等超时
        time.sleep(0.1)
        self._drain()

    def _read_some(self, idle: float = 0.02, total: float = 0.05) -> bytes:
        """把当前已经到达的数据读走；最多等 total 秒（有数据就继续读到 idle 静默）。"""
        buf = b""
        deadline = time.time() + total
        last = time.time()
        while time.time() < deadline and (time.time() - last) < idle:
            r, _, _ = select.select([self.sock], [], [], min(idle, max(0.0, deadline - time.time())))
            if not r:
                break
            try:
                chunk = self.sock.recv(65536)
            except (BlockingIOError, OSError):
                break
            if not chunk:
                break
            buf += chunk
            last = time.time()
        return buf

    def _drain(self):
        # 上一次 cmd() 已经读到提示符了；这里非阻塞清残留，几乎不花时间
        while True:
            r, _, _ = select.select([self.sock], [], [], 0)
            if not r:
                return
            try:
                if not self.sock.recv(65536):
                    return
            except (BlockingIOError, OSError):
                return

    @staticmethod
    def _strip_telnet(data: bytes) -> bytes:
        out = bytearray()
        i = 0
        while i < len(data):
            if data[i] == 0xFF and i + 2 < len(data):     # IAC ...
                i += 3
                continue
            out.append(data[i])
            i += 1
        return bytes(out)

    def cmd(self, line: str, timeout: float = 3.0) -> str:
        t0 = time.time()
        self._drain()
        self.sock.sendall(line.encode() + b"\n")
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            r, _, _ = select.select([self.sock], [], [], 0.05)
            if not r:
                continue
            try:
                chunk = self.sock.recv(65536)
            except (BlockingIOError, OSError):
                break
            if not chunk:
                break
            buf += chunk
            if buf.rstrip().endswith(b">"):               # openocd 提示符
                break
        text = self._strip_telnet(buf).decode(errors="replace")
        if os.environ.get("RTT_DEBUG"):
            TIMINGS.append((line.split()[0], time.time() - t0))
        lines = [ln for ln in text.splitlines() if ln.strip() and not ln.strip().startswith(">")]
        if lines and lines[0].strip() == line.strip():     # 去掉命令回显
            lines = lines[1:]
        return "\n".join(lines)

    def words(self, addr: int, count: int):
        """mdw：读 32 位字。"""
        out = self.cmd(f"mdw 0x{addr:08x} {count}")
        words = []
        for m in re.finditer(r"0x[0-9a-fA-F]{8}:\s*((?:[0-9a-fA-F]{8}\s*)+)", out):
            words += [int(w, 16) for w in m.group(1).split()]
        if len(words) < count:
            raise RuntimeError(f"mdw 失败: {out.strip()[:120]}")
        return words[:count]

    def bytes_(self, addr: int, count: int) -> bytes:
        """mdb：按字节读。"""
        out = self.cmd(f"mdb 0x{addr:08x} {count}")
        data = bytearray()
        for m in re.finditer(r"0x[0-9a-fA-F]{8}:\s*((?:[0-9a-fA-F]{2}\s*)+)", out):
            data += bytes(int(b, 16) for b in m.group(1).split())
        if len(data) < count:
            raise RuntimeError(f"mdb 失败: {out.strip()[:120]}")
        return bytes(data[:count])

    def dump(self, addr: int, count: int, tmp: str = "/tmp/.rtt_hpm_dump.bin") -> bytes:
        """dump_image：批量导出到文件再本地读回。
        halt 期间读取是 RTT 读取器唯一的开销来源，dump_image 比 mdb 快 3~4 倍
        （实测本机约 3.4 KiB/s vs mdb 约 1 KiB/s）。"""
        self.cmd(f"dump_image {tmp} 0x{addr:08x} {count}")
        with open(tmp, "rb") as f:
            return f.read(count)

    def write_word(self, addr: int, value: int):
        self.cmd(f"mww 0x{addr:08x} 0x{value & 0xffffffff:08x}")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def find_sba_tcl() -> str | None:
    """找工程里的 rtt-sba.tcl（非侵入式读取的 Tcl helper）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    found = sorted(glob.glob(os.path.join(here, os.pardir, "boards", "hpm", "*", "rtt-sba.tcl")))
    return os.path.abspath(found[0]) if found else None


class SbaChannel:
    """把 DM 的 SBA 内存访问包成和 halt 路径一样的接口（words / dump / write_word）。

    与 OcdTelnet 的区别：**不发 halt**，hart 全程 running。
    实现见 boards/hpm/hpm5361icb/rtt-sba.tcl 里的 rtt_sba_read / rtt_sba_write。
    """

    def __init__(self, ocd: "OcdTelnet", tcl: str | None = None, fast: bool = False):
        self.ocd = ocd
        self.fast = fast
        if tcl:
            ocd.cmd(f"source {tcl}")
        # 试跑一次配置命令：helper 不在 / SBA 用不了都会在这里报错，调用方据此回退到 halt 路径
        out = ocd.cmd("rtt_sba_config")
        if not re.search(r"0x[0-9a-fA-F]{8}", out):
            raise RuntimeError(f"SBA 不可用: {out.strip()[:160]}")
        self.cmd("resume")       # 目标可能被上一次调试会话留在 halt 状态，先放它跑

    def use_fast_if_verified(self, base: int) -> bool:
        """流水线协议要快 ~1.6 倍，但它的时序假设更强：
        先用稳的 rtt_sba_read 做基准，比对控制块头部（不含会变的 WrOff/RdOff），一致才启用。"""
        try:
            slow = self.words(base, 8)
            self.fast = True
            fast = self.words(base, 8)
        except RuntimeError:
            self.fast = False
            return False
        if fast != slow:
            self.fast = False
        return self.fast

    def plausible(self, base: int) -> bool:
        """控制块还在不在：SBA 卡死时会一直返回陈旧数据，靠这个能发现。"""
        try:
            return struct.pack("<4I", *self.words(base, 4)).startswith(RTT_ID)
        except RuntimeError:
            return False

    # halt 是空操作（这条路径本来就不停核）；resume 照发，保持"不让板子停在 halt"的语义
    def cmd(self, line: str) -> str:
        if line.strip() == "resume":
            return self.ocd.cmd("resume", timeout=5.0)
        return ""

    def words(self, addr: int, count: int):
        # 每个字 2~3 次 DMI 事务，每次事务约 3ms -> 0.5~0.9 KiB/s
        proc = "rtt_sba_read_fast" if self.fast else "rtt_sba_read"
        out = self.ocd.cmd(f"{proc} 0x{addr:08x} {count}", timeout=10.0 + count * 0.02)
        words = [int(m, 16) for m in re.findall(r"0x[0-9a-fA-F]+", out)]
        if len(words) < count:
            raise RuntimeError(f"SBA 读取失败: {out.strip()[:160]}")
        return words[:count]

    def bytes_(self, addr: int, count: int) -> bytes:
        return self.dump(addr, count)

    def dump(self, addr: int, count: int) -> bytes:
        """按字节读：对齐到 4 字节后逐字读回，再按需要的偏移切片。"""
        head = addr & 3
        nwords = (head + count + 3) // 4
        words = self.words(addr - head, nwords)
        raw = b"".join(struct.pack("<I", w) for w in words)
        return raw[head:head + count]

    def write_word(self, addr: int, value: int):
        out = self.ocd.cmd(f"rtt_sba_write 0x{addr:08x} 0x{value & 0xffffffff:08x}")
        if re.search(r"Error|error", out):
            raise RuntimeError(f"SBA 写入失败: {out.strip()[:160]}")

    def close(self):
        pass


class RttReader:
    def __init__(self, ocd: OcdTelnet, base: int):
        self.ocd = ocd
        self.base = base
        head = ocd.words(base, 6)
        self.id_bytes = struct.pack("<4I", *head[:4])
        self.max_up = head[4]
        up0 = ocd.words(base + CB_OFF_AUP0, 6)
        self.p_buffer = up0[UP_OFF_PBUF // 4]
        self.size = up0[UP_OFF_SIZE // 4]
        self.wr = up0[UP_OFF_WR // 4]
        # 注意：设备侧的 RdOff 会被固件自己推进（日志后端用 OVERWRITE 模式时，
        # 写满就调 RdOff 丢最旧数据）。所以主机必须维护【自己的】读取位置，
        # 只把设备 RdOff 当作"接入时的起点"，不能拿它当游标（否则会读不到数据）。
        self.dev_rd = up0[UP_OFF_RD // 4]
        self.host_rd = self.dev_rd

    @property
    def valid(self) -> bool:
        return self.id_bytes.startswith(RTT_ID)

    def refresh_cursor(self):
        self.wr = self.ocd.words(self.base + CB_OFF_AUP0 + UP_OFF_WR, 1)[0]

    def take_chunks(self):
        """返回 [(offset, len), ...]：从上一次读取位置到最新 WrOff 之间还没取走的字节。
        如果中间被设备覆盖掉了（超出缓冲长度），丢掉最旧的部分并告知调用者。"""
        if self.wr == self.host_rd:
            return [], 0

        if self.wr > self.host_rd:
            pending = self.wr - self.host_rd
        else:
            pending = self.size - self.host_rd + self.wr

        lost = 0
        if pending > self.size:                 # 中间被覆盖，只能给最后一整圈
            lost = pending - self.size
            self.host_rd = (self.host_rd + lost) % self.size
            pending = self.size

        chunks = []
        pos, left = self.host_rd, pending
        while left > 0:
            n = min(left, self.size - pos)
            chunks.append((pos, n))
            pos = (pos + n) % self.size
            left -= n
        return chunks, lost

    def ack(self, consumed: int):
        """告诉目标"这 consumed 个字节已经取走"。只 ack 真正读到的部分，
        没读完的留在缓冲里下一轮继续（调试链路慢，一轮读不完整个缓冲）。"""
        self.host_rd = (self.host_rd + consumed) % self.size
        self.ocd.write_word(self.base + CB_OFF_AUP0 + UP_OFF_RD, self.host_rd)


def main() -> int:
    # 被 kill / 终端关闭时也要走 finally 里的 resume，别把板子留在 halt 状态
    # （SIGINT 本来就会变成 KeyboardInterrupt；SIGTERM/SIGHUP 默认直接杀进程，不走 finally）
    def _graceful_exit(signum, frame):
        raise KeyboardInterrupt

    for _sig in ("SIGTERM", "SIGHUP"):
        if hasattr(signal, _sig):
            try:
                signal.signal(getattr(signal, _sig), _graceful_exit)
            except (ValueError, OSError):
                pass

    ap = argparse.ArgumentParser(description="HPM5361 RTT reader (via OpenOCD telnet)")
    ap.add_argument("-e", "--elf", default="build/zephyr/zephyr.elf", help="固件 ELF（找 _SEGGER_RTT 用）")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=4444, help="OpenOCD telnet 端口")
    ap.add_argument("--poll-ms", type=int, default=None,
                    help="轮询周期（从上一轮开始算到这一轮开始，含读取时间）。默认：SBA 模式 1000ms "
                         "（新日志最多晚 1s 出现，想更实时可以给 200~500）；halt 模式 5000ms"
                         "（每次读都要把 CPU 停一下，别调太小）")
    ap.add_argument("--max-read", type=int, default=1024,
                    help="单次读取的字节数上限（分片读，避免一次卡太久），默认 1024")
    ap.add_argument("--drain-ms", type=int, default=2000,
                    help="一轮里最多连续读多久。有新数据就接着读（把积压排空），读满这个时间就歇一下")
    ap.add_argument("--halt", action="store_true",
                    help="强制用 halt→读→resume（会让 CPU 停）而不是默认的非侵入式 SBA 读取")
    ap.add_argument("--safe-read", action="store_true",
                    help="SBA 只用最稳的 3 次 DMI/字协议（0.5KiB/s），不做流水线加速")
    ap.add_argument("--once", action="store_true", help="只读一次控制块信息后退出")
    ap.add_argument("--log", default=None, help="同时把原始数据写入文件")
    args = ap.parse_args()

    base = find_symbol(args.elf, "_SEGGER_RTT")
    if base is None:
        print(f"在 {args.elf} 里找不到 _SEGGER_RTT —— 固件没开 CONFIG_DEBUG_RTT 或 ELF 不对", file=sys.stderr)
        return 1

    try:
        ocd = OcdTelnet(args.host, args.port)
    except OSError as exc:
        print(f"连不上 OpenOCD telnet {args.host}:{args.port} —— {exc}", file=sys.stderr)
        print("先运行 tools/rtt.sh（会拉起 OpenOCD 并带上 HPM 的探针配置）。", file=sys.stderr)
        return 1

    print(f"控制块地址 _SEGGER_RTT = 0x{base:08x}（来自 {args.elf}）")

    try:
        chan = ocd
        mode = "halt（读的时候 CPU 会停）"
        rtt = None
        sba_tcl = find_sba_tcl()
        if not args.halt:
            try:
                chan = SbaChannel(ocd, sba_tcl, fast=not args.safe_read)
                if chan.fast and not chan.use_fast_if_verified(base):
                    print("[warn] 流水线读取协议自检没过，改用稳的 3 次 DMI/字协议。",
                          file=sys.stderr)
                mode = "SBA（CPU 不停" + ("，流水线 2 次 DMI/字" if chan.fast else "") + "）"
                rtt = RttReader(chan, base)
            except (RuntimeError, OSError) as exc:
                print(f"[warn] 非侵入式 SBA 读取不可用：{exc}", file=sys.stderr)
                print("[warn] 回退到 halt→读→resume：每轮会把 CPU 停几十~几百毫秒"
                      "（--poll-ms 调大可减少；恢复 SBA 要复位板子）。",
                      file=sys.stderr)
                chan = ocd
                mode = "halt（读的时候 CPU 会停）"
                rtt = None
        if rtt is None:
            # halt 路径：读内存前必须停核
            ocd.cmd("halt")
            rtt = RttReader(ocd, base)
        # 读取方式决定轮询周期的默认值：halt 模式每次读都要停 CPU，默认拉长一点
        poll_ms = args.poll_ms if args.poll_ms else (5000 if chan is ocd else 1000)
        print(f"读取方式：{mode}；周期 {poll_ms}ms，Ctrl-C 退出。", file=sys.stderr)
        if args.once:
            print(f"acID      : {rtt.id_bytes!r}")
            print(f"MaxNumUp  : {rtt.max_up}")
            print(f"pBuffer   : 0x{rtt.p_buffer:08x}")
            print(f"SizeOfBuffer: {rtt.size}")
            print(f"WrOff/RdOff: {rtt.wr} / {rtt.dev_rd}")
            chan.cmd("resume")
            return 0 if rtt.valid else 2

        if not rtt.valid:
            chan.cmd("resume")
            print("控制块里没有 'SEGGER RTT' 标识 —— 板子里跑的固件不是带 RTT 的这一版（先 reflash）",
                  file=sys.stderr)
            return 2

        log = open(args.log, "ab", buffering=0) if args.log else None
        sys.stderr.write(f"已连接（{mode}，周期 {poll_ms}ms），Ctrl-C 退出\n")
        sys.stderr.flush()

        try:
            halt_mode = chan is ocd
            last_check = time.time()
            last_sba_retry = time.time()
            sba_fail = 0
            halt_fail = 0

            def fall_back_to_halt(reason: str):
                """SBA 不可用时改用 halt→读→resume，别让 RTT 直接断掉。"""
                nonlocal chan, halt_mode, poll_ms
                sys.stderr.write(f"[warn] {reason}\n")
                sys.stderr.write("[warn] 改用 halt 方式继续：数据仍能读，但每一轮都会把 CPU 停"
                                 "几十~几百毫秒（想少停一点就把 --poll-ms 调大）；"
                                 "要让核不被停，得先复位板子恢复 SBA（tools/rtt.sh -r）或断电重来。\n")
                sys.stderr.flush()
                chan = ocd
                rtt.ocd = ocd
                halt_mode = True
                if args.poll_ms is None and poll_ms < 3000:
                    poll_ms = 5000        # 别让回退后的 1s 周期把 CPU 一直按停
                    sys.stderr.write(f"[warn] 轮询周期自动放宽到 {poll_ms}ms（可用 --poll-ms 覆盖）。\n")
                    sys.stderr.flush()
                ocd.cmd("halt")

            while True:
                t_poll = time.time()
                # 一轮里把积压的数据排空（有新数据就接着读），最多读 --drain-ms
                while True:
                    try:
                        # halt 路径下这对 halt/resume 必须成对：中间出任何岔子（Ctrl-C、kill、
                        # 探针报错）都要把核放回去，否则板子会一直停在 halt
                        chan.cmd("halt")
                        try:
                            rtt.refresh_cursor()
                            chunks, lost = rtt.take_chunks()
                            if os.environ.get("RTT_DEBUG"):
                                sys.stderr.write(f"[dbg] wr={rtt.wr} host_rd={rtt.host_rd} "
                                                 f"dev_rd={rtt.dev_rd} chunks={chunks} lost={lost}\n")
                                sys.stderr.flush()
                            if chunks:
                                data = b""
                                budget = args.max_read
                                for off, n in chunks:
                                    want = min(n, budget)
                                    if want:
                                        data += chan.dump(rtt.p_buffer + off, want)
                                        budget -= want
                                    if budget <= 0:
                                        break
                                rtt.ack(len(data))    # 只 ack 已取走的字节
                                if lost:
                                    sys.stderr.write(f"[rtt] 缓冲区被覆盖，丢了 {lost} 字节\n")
                                    sys.stderr.flush()
                                if data:
                                    sys.stdout.buffer.write(data)
                                    sys.stdout.buffer.flush()
                                    if log is not None:
                                        log.write(data)
                        finally:
                            chan.cmd("resume")
                    except RuntimeError as exc:
                        # 探针偶发抽风（CMSIS-DAP command ... failed）不该直接判 SBA 死刑：
                        # 先原地重试，连续几次不行才退回 halt 方式
                        if halt_mode:
                            halt_fail += 1
                            if halt_fail >= 5:
                                sys.stderr.write(f"[err] 连续 {halt_fail} 次读取失败，退出：{exc}\n")
                                sys.stderr.write("[err] 探针可能又假死了：sudo /usr/local/sbin/hpm-probe-usb reset\n")
                                sys.stderr.flush()
                                return 1
                            if halt_fail == 1:
                                sys.stderr.write(f"[warn] 读取失败（{halt_fail}），重试：{exc}\n")
                            time.sleep(1.0)
                            continue
                        sba_fail += 1
                        if sba_fail < 3:
                            sys.stderr.write(f"[warn] SBA 读取出错（第 {sba_fail}/3 次，重试）：{exc}\n")
                            time.sleep(0.2)
                            continue
                        sba_fail = 0
                        fall_back_to_halt(f"SBA 连续读失败：{exc}")
                        continue
                    else:
                        sba_fail = 0
                        halt_fail = 0
                    if time.time() - t_poll > args.drain_ms / 1000.0:
                        break
                    if not chunks:
                        break
                if os.environ.get("RTT_DEBUG"):
                    sys.stderr.write(f"[dbg] 本轮读取 {time.time()-t_poll:.2f}s, "
                                     f"命令耗时 {TIMINGS}\n")
                    TIMINGS.clear()
                    sys.stderr.flush()

                # 每隔一会儿自检一次：SBA 卡死时读回来的是陈旧数据，看上去像"没有新日志"
                if not halt_mode and time.time() - last_check > 30:
                    last_check = time.time()
                    if not chan.plausible(rtt.base):
                        fall_back_to_halt("控制块读回来不对 —— SBA 可能已经卡死。")

                # 回退到 halt 之后也别放弃 SBA：每 30s 试一次，能用了就切回去（不用重启脚本）
                elif halt_mode and not args.halt and time.time() - last_sba_retry > 30:
                    last_sba_retry = time.time()
                    try:
                        retry = SbaChannel(ocd, sba_tcl, fast=not args.safe_read)
                        if retry.plausible(rtt.base):
                            if retry.fast:
                                retry.use_fast_if_verified(rtt.base)   # 自检不过会自己退回稳妥协议
                            chan = retry
                            rtt.ocd = retry
                            halt_mode = False
                            if args.poll_ms is None:
                                poll_ms = 1000
                            sys.stderr.write("[info] SBA 又能用了，切回不停 CPU 的读取方式。\n")
                            sys.stderr.flush()
                    except (RuntimeError, OSError):
                        pass              # 还不行就继续用 halt 方式，下分钟再试

                # 周期从本轮开始算：读取花掉的时间算在 --poll-ms 里
                time.sleep(max(0.0, poll_ms / 1000.0 - (time.time() - t_poll)))
        except KeyboardInterrupt:
            pass
        finally:
            chan.cmd("resume")
            if log is not None:
                log.close()
    finally:
        ocd.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
