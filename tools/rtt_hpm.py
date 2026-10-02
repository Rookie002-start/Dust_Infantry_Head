#!/usr/bin/env python3
"""HPM5361(RISC-V) 上的 SEGGER RTT 主机侧读取器 —— 走 OpenOCD telnet。

为什么需要它：
  OpenOCD 的「目标侧 RTT」（`rtt setup` / `rtt start` / `rtt server`）只对 ARM(Cortex-M) 目标注册，
  RISC-V 目标上没有这些命令；而且本机的 OpenOCD 在 HPM5361 上**运行中读不了内存**
  （sysbus / abstract / progbuf 都报 `Failed to read priv register`，必须 halt 才能读）。
  所以这里直接按 SEGGER RTT 协议自己读控制块和环形缓冲：

      halt → 读控制块/上行缓冲 → 把读到的位置写回 RdOff → resume

  目标是 HPM 板时用它；STM32 板（rm_c / dm_mc02）用 OpenOCD 原生的 rtt 命令更顺，
  见 tools/rtt.sh / docs/segger_rtt.md。

用法：
    tools/rtt_hpm.py                       # 自动找 build/zephyr/zephyr.elf，连 127.0.0.1:4444
    tools/rtt_hpm.py -e /tmp/build-hpm/zephyr/zephyr.elf
    tools/rtt_hpm.py --once                # 只读一次控制块信息（排查用）
    tools/rtt_hpm.py --poll-ms 300 --log rtt.log

前提：另开一个终端先起 OpenOCD（tools/rtt.sh 会自动做），telnet 口默认 4444。
"""

import argparse
import glob
import os
import re
import select
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

    def cmd(self, line: str) -> str:
        t0 = time.time()
        self._drain()
        self.sock.sendall(line.encode() + b"\n")
        buf = b""
        deadline = time.time() + 3.0
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
    ap = argparse.ArgumentParser(description="HPM5361 RTT reader (via OpenOCD telnet)")
    ap.add_argument("-e", "--elf", default="build/zephyr/zephyr.elf", help="固件 ELF（找 _SEGGER_RTT 用）")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=4444, help="OpenOCD telnet 端口")
    ap.add_argument("--poll-ms", type=int, default=5000,
                    help="轮询周期。每轮都要 halt 目标把缓冲读走（实测本机读取速度约 2~3.4KiB/s），"
                         "所以持续读取会占用固件一成以上的运行时间；周期越大影响越小，"
                         "日志积压超过缓冲 4KB（本工程约 13s 的日志量）才会丢")
    ap.add_argument("--max-read", type=int, default=2048,
                    help="每轮最多读取的字节数（halt 时长与读取量成正比）")
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
    print("提示：HPM 上 OpenOCD 只能 halt 读内存，读取期间 CPU 会停；"
          f"当前周期 {args.poll_ms}ms，越大越不影响固件（--poll-ms 10000）。", file=sys.stderr)

    try:
        ocd.cmd("halt")                              # 只有 halt 才能读内存
        rtt = RttReader(ocd, base)
        if args.once:
            print(f"acID      : {rtt.id_bytes!r}")
            print(f"MaxNumUp  : {rtt.max_up}")
            print(f"pBuffer   : 0x{rtt.p_buffer:08x}")
            print(f"SizeOfBuffer: {rtt.size}")
            print(f"WrOff/RdOff: {rtt.wr} / {rtt.dev_rd}")
            ocd.cmd("resume")
            return 0 if rtt.valid else 2

        if not rtt.valid:
            ocd.cmd("resume")
            print("控制块里没有 'SEGGER RTT' 标识 —— 板子里跑的固件不是带 RTT 的这一版（先 reflash）",
                  file=sys.stderr)
            return 2

        log = open(args.log, "ab", buffering=0) if args.log else None
        sys.stderr.write(f"已连接（halt 周期 {args.poll_ms}ms），Ctrl-C 退出\n")
        sys.stderr.flush()

        try:
            while True:
                t_poll = time.time()
                ocd.cmd("halt")
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
                            data += ocd.dump(rtt.p_buffer + off, want)
                            budget -= want
                        if budget <= 0:
                            break
                    rtt.ack(len(data))                # 只 ack 已取走的字节
                    if lost:
                        sys.stderr.write(f"[rtt] 缓冲区被覆盖，丢了 {lost} 字节\n")
                        sys.stderr.flush()
                    if data:
                        sys.stdout.buffer.write(data)
                        sys.stdout.buffer.flush()
                        if log is not None:
                            log.write(data)
                ocd.cmd("resume")
                if os.environ.get("RTT_DEBUG"):
                    sys.stderr.write(f"[dbg] 本轮 halt→resume {time.time()-t_poll:.2f}s, "
                                     f"命令耗时 {TIMINGS}\n")
                    TIMINGS.clear()
                    sys.stderr.flush()
                time.sleep(args.poll_ms / 1000.0)
        except KeyboardInterrupt:
            pass
        finally:
            ocd.cmd("resume")
            if log is not None:
                log.close()
    finally:
        ocd.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
