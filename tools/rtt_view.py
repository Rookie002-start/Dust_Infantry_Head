#!/usr/bin/env python3
"""极简 SEGGER RTT 终端（主机侧查看器）。

连接 OpenOCD 的 `rtt server`（默认 127.0.0.1:9090，由 tools/rtt.sh 拉起），
把目标板上行通道的数据实时打到终端。Ctrl-C 退出。

用法:
    tools/rtt_view.py                # 连接 127.0.0.1:9090
    tools/rtt_view.py -p 9098
    tools/rtt_view.py --log rtt.log  # 同时存文件
"""

import argparse
import socket
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="SEGGER RTT terminal (via OpenOCD)")
    parser.add_argument("--host", default="127.0.0.1", help="OpenOCD rtt server 地址")
    parser.add_argument("-p", "--port", type=int, default=9090, help="OpenOCD rtt server 端口")
    parser.add_argument("--log", default=None, help="同时把原始数据写入该文件")
    args = parser.parse_args()

    try:
        sock = socket.create_connection((args.host, args.port), timeout=5)
    except OSError as exc:
        print(f"连不上 OpenOCD RTT {args.host}:{args.port} —— {exc}", file=sys.stderr)
        print("先运行 tools/rtt.sh 起服务（需要调试器在线、固件已烧录）。", file=sys.stderr)
        return 1

    sock.settimeout(None)
    sys.stderr.write(f"已连接 RTT {args.host}:{args.port}，Ctrl-C 退出\n")
    sys.stderr.flush()

    log = open(args.log, "ab", buffering=0) if args.log else None
    try:
        while True:
            data = sock.recv(4096)
            if not data:
                # openocd 侧没数据时不会断开，这里只是兜底
                time.sleep(0.1)
                continue
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
            if log is not None:
                log.write(data)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if log is not None:
            log.close()
    sys.stderr.write("\nRTT 连接已关闭\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
