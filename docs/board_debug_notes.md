# 板级调试笔记：OpenOCD + telnet + RTT + GDB

本文记录"用调试器连上板子、把问题查清楚"的完整手法，覆盖**上板（HPM5361，RISC-V）**
和**下板（STM32H723，Cortex-M7）**。

适用场景：CAN 收发不通、外设没反应、怀疑时钟/引脚配置错、需要看应用内部变量、
需要量化某个循环的真实速率或丢帧率。

> 本文件在 `Dust_Infantry_Head/docs/` 下，但下板相关章节同样适用于 `Dust_Infantry_Body`。
> 相关工具：`tools/rtt.sh`、`tools/rtt_hpm.py`（上板 RTT）、
> `boards/*/*/openocd*.cfg`（OpenOCD 配置）。

---

## 0. 硬件拓扑（先搞清楚"连的是哪块板"）

VM 里挂着**两个 Horco CMSIS-DAP**，VID 相同（都是 `faed`），只能靠 PID / serial 区分：

| 板 | 芯片 | 探针 | 传输 | 目标名 |
|---|---|---|---|---|
| 上板 | HPM5361（Andes V5 RISC-V） | `faed:4870`（serial 630123326860，FW v0.2） | **JTAG** | `hpm5361.cpu0` |
| 下板 | STM32H723VGT6（Cortex-M7） | `faed:4873`（serial 343420334454，FW v0.1） | **SWD** | `STM32H723VG.cpu0` |

```bash
# 看当前插了哪个探针
lsusb | grep -i faed
```

⚠️ **沙箱里看不到 USB**：`/dev/bus/usb` 在沙箱下不可访问，且沙箱与宿主机不在同一个
网络命名空间（`nc 127.0.0.1:4444` 会失败）。所以下面所有 OpenOCD / telnet / RTT 命令
都需要**提权**执行。

---

## 1. 快速上手（TL;DR）

```bash
# ① 上板：起 OpenOCD（JTAG / Horco CMSIS-DAP）
HPM=$HOME/Zephyr_HPMicro/sdk_env/hpm_sdk/boards/openocd
$HOME/openocd-hpm/bin/openocd \
  -s "$HPM" -s "$HOME/Zephyr_HPMicro/sdk_glue_user/boards/openocd" \
  -f "$HPM/probes/cmsis_dap.cfg" -f "$HPM/soc/hpm5300.cfg" \
  -f "$HPM/boards/hpm5300evk.cfg" \
  -c "adapter speed 1000" \
  -c "gdb port 3334" -c "tcl port 6667" -c "telnet port 4445" \
  -c "hpm5361.cpu0 configure -event gdb-attach {halt}" \
  -c "init" -c 'if {[catch {resume} msg]} { echo "resume: $msg" }'

# ② 读寄存器 / 内存（另开一个终端 telnet 进去发命令）
telnet localhost 4445
> halt
> mdw 0xF028C000 24        # MCAN3 寄存器块
> reg pc
> resume

# ③ 看上板日志（RTT）
cd ~/zephyrproject/projects/Dust_Infantry_Head
python3 tools/rtt_hpm.py -e build/zephyr/zephyr.elf --port 4445
```

```bash
# 下板：起 OpenOCD（SWD），并同时起 RTT 服务
BODY=~/zephyrproject/projects/Dust_Infantry_Body
/usr/bin/openocd -s /usr/share/openocd/scripts \
  -f "$BODY/boards/st/board_dm_mc02/openocd.cfg" \
  -c "STM32H723VG.cpu0 configure -event gdb-attach {halt}" \
  -c init \
  -c "rtt setup 0x24001020 4096 \"SEGGER RTT\"" \
  -c "rtt start" -c "rtt server start 9090 0" \
  -c 'if {[catch {resume} msg]} { echo "resume: $msg" }'

# 看下板日志
nc localhost 9090
```

---

## 2. 探针与 USB 层

### 2.1 不要用工程里的 `bin/openocd` 垫片

`boards/hpm/hpm5361icb/bin/openocd` 是 wrapper 脚本，作用是"杀残留 openocd → 复位 USB
设备 → 解绑 cdc_acm"，给 `west flash` 用的。**手动调试时直接调真 openocd**，原因：

- wrapper 会**复位 USB 设备**，打断正在用的调试会话；
- 两个探针 VID 相同，它的自动探测会挑到**第一个匹配**（`1-2.1`），实测会去动**另一个**
  探针；
- 它解绑 cdc_acm 后**不保证绑回去**，会导致探针后续不应答（详见第 12 节）。

真 openocd 路径：`$HOME/openocd-hpm/bin/openocd`（先楫自编译版，带 `hpm_xpi` flash 驱动）。

### 2.2 探针假死与恢复

这个型号会偶发"假死"：设备/配置/接口描述符都能读，但所有 CMSIS-DAP 命令超时：

```
Warn : could not read product string for device 0xfaed:0x4870: Operation timed out
Error: CMSIS-DAP command CMD_INFO failed.
Error: [hpm5361.cpu0] Unsupported DTM version: -1
```

恢复顺序（免密 helper，只有 `reset`/`unbind-cdc`/`bind-cdc` 有免密权限）：

```bash
sudo -n /usr/local/sbin/hpm-probe-usb reset        # authorized 0->1，强制重新枚举
sudo -n /usr/local/sbin/hpm-probe-usb unbind-cdc   # 解绑探针虚拟串口
sudo -n /usr/local/sbin/hpm-probe-usb bind-cdc     # 绑回去（实测这一步常是解药）
```

都无效时：**拔插探针**（VM 里 `VM → Removable Devices → 设备 → Disconnect/Connect` 等效）。

### 2.3 减少把探针搞死的概率

- **退出时让 openocd 自己收尾**：在 telnet 里发 `shutdown`，或用 `-c shutdown`；
  不要 `kill -9` / SIGTERM 打断正在进行的 USB 传输。
- 结束前先 `resume`，别让目标停在 halt 状态后被强杀。
- 只用一支探针时，避免两个 openocd 抢同一个设备。

---

## 3. 上板（HPM5361 / RISC-V）调试环境

### 3.1 启动命令

```bash
HPM=$HOME/Zephyr_HPMicro/sdk_env/hpm_sdk/boards/openocd
$HOME/openocd-hpm/bin/openocd \
  -s "$HPM" \
  -s "$HOME/Zephyr_HPMicro/sdk_glue_user/boards/openocd" \
  -f "$HPM/probes/cmsis_dap.cfg" \
  -f "$HPM/soc/hpm5300.cfg" \
  -f "$HPM/boards/hpm5300evk.cfg" \
  -c "adapter speed 1000" \
  -c "gdb port 3334" -c "tcl port 6667" -c "telnet port 4445" \
  -c "hpm5361.cpu0 configure -event gdb-attach {halt}" \
  -c "init" \
  -c 'if {[catch {resume} msg]} { echo "resume: $msg" }'
```

### 3.2 关键点（都是踩过的坑）

| 项 | 说明 |
|---|---|
| 端口冲突 | 默认 gdb 3333 / telnet 4444 / tcl 6666。若另一个 OpenOCD（比如下板）在跑会冲突，这里改成 3334/4445/6667 |
| `gdb-attach` | `boards/hpm5300evk.cfg` 里写的是 `gdb-attach { reset halt }`，**一挂 GDB 就复位芯片**。手动"只看不动"时要覆盖成 `{halt}` |
| `resume` 报错 | 目标没被 halt 时 `resume` 会报 `Not halted` 并让 openocd 退出，所以必须用 `catch` 包住 |
| 目标名 | `hpm5361.cpu0`（CHIPNAME 未设置时 soc/hpm5300.cfg 的默认值） |
| 运行中读不了内存 | OpenOCD 自己的内存访问（`mdw` / sysbus / abstract / progbuf）在目标运行时全失败（`Failed to read priv register`）。**绕开它、直接用 DM 的 SBA（见 9.1 / `boards/hpm/hpm5361icb/rtt-sba.tcl`）就能不停 CPU 读**；只有 SBA 不可用时才需要 halt |

### 3.3 暂停/恢复的代价（很重要）

这支探针每次 `halt` + 读 + `resume` 大约要 **50~200 ms**（读 90 个字甚至要 1~3 秒）。
也就是说**每次"看一眼"都会让 CPU 停住几十到几百毫秒**。

后果：

- 会让"1 ms 周期"的通路（CAN、UART、控制环）产生**真实的数据丢失/相位跳变**；
- FIFO 类外设（CAN RX FIFO 只有几格）会因此溢出，**满/丢帧标志出现是必然的**，
  不能据此判断正常运行时的丢帧 —— 见第 10、12 节。

所以：**能用应用自己的计数器就别用调试器去数事件**。

---

## 4. 下板（STM32H723 / Cortex-M7）调试环境

### 4.1 启动命令（同时开 RTT 服务）

```bash
BODY=~/zephyrproject/projects/Dust_Infantry_Body
/usr/bin/openocd -s /usr/share/openocd/scripts \
  -f "$BODY/boards/st/board_dm_mc02/openocd.cfg" \
  -c "STM32H723VG.cpu0 configure -event gdb-attach {halt}" \
  -c init \
  -c "rtt setup 0x24001020 4096 \"SEGGER RTT\"" \
  -c "rtt start" \
  -c "rtt server start 9090 0" \
  -c 'if {[catch {resume} msg]} { echo "resume: $msg" }'
```

RTT 控制块地址不要写死，从 ELF 里的 `_SEGGER_RTT` 符号取：

```bash
arm-zephyr-eabi-readelf -sW "$BODY/build/zephyr/zephyr.elf" \
  | awk '$NF=="_SEGGER_RTT"{print $2}'
# 例：0x24001020
```

### 4.2 关键点

| 项 | 说明 |
|---|---|
| 端口参数名 | 系统版 OpenOCD 0.12 用 **`gdb_port` / `telnet_port` / `tcl_port`**；HPM 那个自编译 dev 版才吃 `gdb port` 这种写法。用错会报 `invalid command name "gdb"` 并退出 |
| `gdb-attach` | 项目 cfg 里是 `reset halt`（挂 GDB 会复位），同样按需覆盖成 `{halt}` |
| `examine-end` | cfg 里已经把 `examine-end` 事件清空（运行中 CPU 访问 DBGMCU 会失败），别把它改回去 |
| 目标名 | `STM32H723VG.cpu0`（cfg 里 `set CHIPNAME STM32H723VG`） |
| 运行中读内存 | Cortex-M 一般可以不停机读，但为了不引入扰动，建议同样用 `halt`/`resume` 成对操作 |

---

## 5. 读数据：telnet 命令速查

OpenOCD 的 telnet 口（默认 4444）是**手动调试的主力**。注意：**不要指望用 openocd 的
stdin 发命令**（实测不生效，只会回显），必须走 telnet。

```
halt                      # 停核（读内存前必须）
resume                    # 继续跑
reset                     # 复位（慎用，会打断实验）
reset halt                # 复位后停住（触发 reset-init 事件）

reg pc                    # 读单个寄存器
reg                       # 读所有寄存器

mdw 0xF028C000 24         # 读 24 个 32 位字
mdb 0x240029c4 68         # 读 68 个字节
mww 0xE0001000 0x01000001 # 写 32 位字（改寄存器 / 开 DWT 等）

targets                   # 看目标状态（running / halted）
shutdown                  # 优雅退出（推荐用它收尾）
```

批量读取建议写成 Python 脚本走 socket，输出好解析（见第 13 节模板）。

---

## 6. 寄存器地图与位定义

### 6.1 上板：HPM MCAN（上下板通信总线 = `inter-can`）

4 个实例，基地址间隔 0x4000：

| 实例 | 基地址 | 用途（本工程） |
|---|---|---|
| mcan0 | `0xF0280000` | 预留 |
| mcan1 | `0xF0284000` | `booster-can`（实测是 C620 电机反馈总线，帧 ID 0x201~0x204） |
| mcan2 | `0xF0288000` | 预留 |
| mcan3 | `0xF028C000` | **`inter-can`（上下板通信，ID 0x100，63B FD 帧）** |

常用寄存器偏移（与 Bosch M_CAN 基本一致）：

| 偏移 | 名称 | 关心的位 |
|---|---|---|
| 0x0C | DBTP | DBRP[20:16] DTSEG1[12:8] DTSEG2[7:4] DSJW[3:0] TDC[23] |
| 0x10 | TEST | **bit7 = RX 引脚实际电平**（0=显性 / 1=隐性） |
| 0x18 | CCCR | INIT[0] CCE[1] MON[5] TEST[7] **FDOE[8] BRSE[9]** |
| 0x1C | NBTP | NSJW[31:25] NBRP[24:16] NTSEG1[15:8] NTSEG2[6:0] |
| 0x40 | ECR | TEC[7:0] REC[14:8] CEL[22:16] |
| 0x44 | PSR | 见下方"位定义不同"警告 |
| 0x48 | TDCR | TDC 偏移/滤波（TDC 关闭时为 0） |
| 0x50/0x54 | IR / IE | 中断标志 / 使能 |
| 0xA0/0xA4 | RXF0C / RXF0S | RX FIFO0 配置 / 状态 |
| 0xB0/0xB4 | RXF1C / RXF1S | RX FIFO1 |
| 0xBC | RXESC | RX 元素数据长度（7 = 64 字节） |
| 0xC0/0xC4/0xC8 | TXBC / TXFQS / TXESC | TX 缓冲配置 / 空闲状态 / 元素长度 |
| 0xCC/0xD8/0xDC | TXBRP / TXBTO / TXBCF | 待发 / **发送成功过的邮箱掩码** / 取消完成 |

⚠️ **HPM 的 PSR 位定义和 Bosch M_CAN 不一样**（按 Bosch 解会误判）：

```
HPM   PSR:  LEC[2:0]  ACT[4:3](2位)  EP[5]  EW[6]  BO[7]  DLEC[10:8]
Bosch PSR:  LEC[2:0]  ACT[3](1位)    EP[4]  EW[5]  BO[6]  DLEC[10:8]
```

- ACT（HPM 是 2 位）：0=同步中 / 1=空闲 / 2=接收 / 3=发送
- LEC/DLEC：0=NoError 1=Stuff 2=Form 3=Ack 4=Bit1 5=Bit0 6=CRC 7=NoChange
  （`DLEC` 是**数据段**最后错误码）

同理 RXF0C / RXF0S 的字段宽度也不同（来自 `hpm_mcan_regs.h`）：

```
RXF0C: F0SA[15:2]  F0S[22:16](7位)  F0WM[30:24]  F0OM[31]
RXF0S: F0FL[6:0](7位)  F0GI[13:8]  F0PI[21:16]  F0F[24]  RF0L[25]
```

**用某个 MCAN 前，先到 SDK 头文件核对位定义**
（`sdk_env/hpm_sdk/soc/<soc>/ip/hpm_mcan_regs.h`），不要凭 Bosch 手册的记忆。

**Message RAM 位置（上板）**

- MCAN 的 message RAM 在 AHB SRAM：基址 `0xF0400000`，每实例一段。
  `MCAN_MSG_BUF_SIZE_IN_WORDS = 640`（= 2560 B），实例 i 的段 = `0xF0400000 + i*2560`。
- 但**寄存器里的 FIFO 起始地址（F0SA/TBSA/FLSSA…）是从 `0xF0400000` 起算的绝对偏移**，
  不是相对实例段。例如 mcan3 的 RX FIFO0 实际落在 `0xF0401EC0` 附近。
- RX 元素步长 = `8 + 负载字节数`（FD 64 字节负载 → 72 字节）。
- 元素头部两个字：

```
R0: ESI[31] XTD[30] RTR[29] ID[28:0]      (标准帧 ID 在 [28:18])
R1: TS[15:0] DLC[19:16] BRS[20] FDF[21] FIDX[28:24] ANMF[31]
```

**上板 CAN 时钟**：`clk-source = CLK_SRC_OSC24M`、`clk-divider = 1` → **24 MHz**。
1 Mbps = 24 tq（NBRP=0 / NTSEG1=16 / NTSEG2=5）。反算实际位率：

```
位率 = 时钟 / ((NBRP+1) * (1 + (NTSEG1+1) + (NTSEG2+1)))
```

再和 DT 里写的目标位率比对。

### 6.2 下板：STM32H7 FDCAN

| 实例 | 基地址 | 用途 |
|---|---|---|
| fdcan1 | `0x4000A000` | `user-can1`（底盘电机总线） |
| fdcan2 | `0x4000A400` | `user-can2` / **`inter-can`（上下板通信，ID 0x200，12B FD 帧）** |
| fdcan3 | `0x4000D400` | `user-can3`（云台电机，DM） |

message RAM：`0x4000AC00`（`reg-names = "message_ram"`），FIFO 起始地址是**相对它的偏移**
（例如 `F0SA=0x400` → 实际地址 `0x4000B000`）。

寄存器偏移与 HPM 相同（都是 M_CAN 风格），**但 PSR 位定义是 Bosch 的**：

```
Bosch PSR: LEC[2:0] ACT[3](1位) EP[4] EW[5] BO[6] DLEC[10:8]
```

**时钟源必须显式配置**（下板踩过的大坑）：

```
RCC 基址 0x58024400
RCC_D2CCIP1R (0x50) 的 FDCANSEL[29:28]：
    0 = HSE（24MHz，复位默认值！）
    1 = PLL1Q（本项目 480/4 = 120MHz）
    2 = PLL2Q（本项目 80MHz）
```

DT 里只写 `clocks = <&rcc STM32_CLOCK(APB1_2, 8)>` 时，FDCANSEL 保持复位默认 = HSE，
**实际位率会只有标称的 1/5**。正确写法要带第二项：

```dts
clocks = <&rcc STM32_CLOCK(APB1_2, 8)>,
         <&rcc STM32_SRC_PLL1_Q FDCAN_SEL(1)>;
```

### 6.3 下板常用的其它外设

| 外设 | 基址 | 说明 |
|---|---|---|
| GPIOB | `0x58020400` | MODER +0x00 / **IDR +0x10** / AFR[0] +0x20。PB5=FDCAN2_RX、PB6=FDCAN2_TX |
| GPIOD | `0x58020C00` | PD0/PD1 = FDCAN1 RX/TX |
| RCC | `0x58024400` | 时钟树 |
| DWT | `0xE0001000` | 调试用周期计数器（见 10.5） |

**读引脚电平直接看 IDR**：引脚在 AF 模式下 IDR 仍反映真实电平。
判断"TX 是不是我们在拉低"、"RX 看到的总线是显性还是隐性"非常有用。

---

## 7. 内存里的应用状态（怎么找、读什么）

### 7.1 找符号地址

```bash
cd ~/zephyrproject/projects/Dust_Infantry_Head      # 或 Dust_Infantry_Body
readelf -sW build/zephyr/zephyr.elf | grep -E "inter_rx|mcu_inter|from_body|slots"
```

典型对象（**地址每次重新编译都会变，必须现查**）：

| 符号 | 含义 |
|---|---|
| `thread::inter_rx::asm_` | 收帧缓冲 + **`atomic rx_cnt` 收帧计数器** |
| `thread::mcu_inter::slots` | 发送侧私有分片缓存（`valid/len/stamp_ms/flags`） |
| `...SendFrame(...)::last_warn_ms` | 发送失败限流时间戳（= 发送在失败的证据） |
| `_SEGGER_RTT` | RTT 控制块 |

### 7.2 不要凭结构体推偏移 —— 用"差值法"找计数器

`asm_` 的结构是 `{ uint8_t data[N]; struct k_spinlock lock; atomic_t rx_cnt; }`，
但**编译器的填充/重排会让 `rx_cnt` 的偏移与直觉不一致**（上板实测在 +16，下板在 +64）。
可靠做法：

1. 把 `asm_` 连后面几个字一起读下来（比如 20 个字）；
2. 隔 20~30 秒**完全不打断**再读一次；
3. 差值最大、且单调递增的那个字就是计数器。

### 7.3 用应用计数器做"速率/丢帧"测量（关键手法）

调试器数不了事件，但**应用自己在数**。典型测法：

```python
c1 = read(ASM + cnt_offset)      # 第一次读数（会 halt 一次）
time.sleep(25)                   # 中间完全不碰调试器
c2 = read(ASM + cnt_offset)      # 第二次读数
rate = (c2 - c1) / elapsed
```

两次 halt 只损失约 2x100ms 的帧（在 25 秒窗口里 <1.5%），
**远小于用"反复采样"去估速率引入的误差**。

---

## 8. GDB 用法与限制

```bash
# 上板（RISC-V 工具链）
~/zephyrproject/zephyr-sdk-1.0.1/gnu/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-gdb \
  build/zephyr/zephyr.elf \
  -ex 'set confirm off' -ex 'set pagination off' \
  -ex 'target extended-remote :3334' \
  -ex 'bt' -ex 'info registers' -ex 'detach'

# 下板用 arm-zephyr-eabi-gdb，端口换成下板 OpenOCD 的 gdb 口
```

**限制（实测）**：

- **硬件断点在这块 HPM 核上用不了**：
  `Cannot insert hardware breakpoint ... Remote failure reply: 0E`
  → 本项目的排查**全程不用断点**，改用"读寄存器/变量"。
- 连 GDB 前记得把 `gdb-attach` 事件改成 `{halt}`（见第 3/4 节），否则会复位芯片。
- GDB 更适合做**符号化**：把 PC 交给 `addr2line` 即可。

```bash
riscv64-zephyr-elf-addr2line -f -C -e build/zephyr/zephyr.elf 0x80013e2a
# 落在 arch_irq_unlock → CPU 正常在跑
# 落在 arch_system_halt → 固件已经进了 fatal handler（CPU exception 等）
```

---

## 9. RTT 日志

### 9.1 上板（RISC-V）：用 `tools/rtt_hpm.py`

OpenOCD 的 `rtt setup/start/server` 只对 ARM 目标注册，RISC-V 上没有；
脚本按 SEGGER RTT 协议自己读控制块和上行缓冲，默认用 **DM 的 SBA**
（`rtt-sba.tcl` 里的 `rtt_sba_read` / `rtt_sba_write`，直接操作 `sbcs` / `sbaddress0` / `sbdata0`），
**hart 全程 running，CPU 不停**（实测 0.5~0.9 KiB/s，读到的内容和 halt 后的 `mdw` 逐字一致）。
只有 SBA 不可用时才回退到 `halt → 读 → 写回 RdOff → resume`：

```bash
cd ~/zephyrproject/projects/Dust_Infantry_Head
python3 tools/rtt_hpm.py -e build/zephyr/zephyr.elf --port 4445            # 持续看
python3 tools/rtt_hpm.py -e build/zephyr/zephyr.elf --port 4445 --once     # 只看控制块
python3 tools/rtt_hpm.py ... --log /tmp/rtt.log                            # 同时存盘
python3 tools/rtt_hpm.py ... --poll-ms 200                                 # 想更实时（默认 1000ms）
python3 tools/rtt_hpm.py ... --halt                                        # 强制停下 CPU 读（对比用）
```

SBA 读的坑（都写在 `rtt-sba.tcl` 顶部）：`sbcs` 位定义和标准 DM 不一样，
要同时置 bit15+bit18 才是 32 位访问；bit20 必须清；每读一个字都要重写 `sbaddress0`，
且第一次读 `sbdata0` 拿到的是上一次的结果（要读两次取第二次）。
提速与刷新率：瓶颈是探针 USB 往返（每个 DMI 事务约 3ms，8MHz 只快 12%），
所以用「写下一个地址 + 读」的流水线把每字从 3 次 DMI 降到 2 次（实测
稳 0.45~0.5 KB/s → 快 0.6~0.85 KB/s，两种协议读同一段内存逐字一致）；
读取器还会在一轮里把积压连续读空、并把读取时间算进 `--poll-ms`，
默认 1s 周期 = 新日志最多晚 1s 出现，`--poll-ms 200` 实测也稳（读取时 `targets` 全程 running）。
⚠️ SBA 只能读 RAM：读 XIP flash 这类地址会把 DM 的 SBA 引擎卡死（之后全是陈旧数据，
写 `sbcs` 也救不回来）。恢复要**整机复位**：`riscv dmi_write 0x10 0x3` 再 `riscv dmi_write 0x10 0x1`
（`dmcontrol.ndmreset`）或断电；实测只拉 SRST（`reset halt` / `tools/rtt.sh -r`）清不掉。
读 RTT 缓冲（RAM）不会碰到这条。

### 9.2 下板（Cortex-M）：OpenOCD 原生 RTT

```bash
nc localhost 9090            # 或 python socket 读一段时间
```

### 9.3 注意

- **RTT 通道是有限的环形缓冲**，高频打印会把 LOG_* 冲掉；
  上板实测被一条每秒数条的浮点打印刷屏，排查时容易误判"没有日志"。
- 默认（SBA）读取不会停 CPU，做速率测量时影响远小于以前；但 `--halt` 模式仍然会 halt/resume，
  测量期间不要开着它，否则污染测量。

---

## 10. 方法论：怎样用这套工具"查出一个 bug"

### 10.1 取证顺序：外设寄存器 → 驱动状态 → 应用变量 → 日志

1. **外设寄存器**是硬件的唯一真相：
   - 通不通：`TXBTO`（有没有发出去过）、`RXF0S` + FIFO 内容（收到了什么帧）
   - 配置对不对：`CCCR.FDOE/INIT`、`NBTP/DBTP` 折算出的实际位率
   - 出什么错：`ECR`（TEC/REC）、`PSR`（LEC/DLEC/BO）
2. **驱动状态**：例如 `hpm_mcan_send()` 的返回值路径、Zephyr CAN 驱动的 tx_sem。
3. **应用变量**：分片缓存、`comm_state`、收帧计数器。
4. **日志**：最后再看，因为日志可能被刷掉或被限流。

### 10.2 先证明"板子里跑的就是手上的 ELF"

读 flash 里某个函数的机器码，和 ELF 对字节比对：

```bash
# ELF 侧
riscv64-zephyr-elf-objdump -d --start-address=0x800158ca \
  --stop-address=0x800158fa build/zephyr/zephyr.elf
# 目标侧（telnet）
mdb 0x800158ca 16
```

两边字节完全一致 → 符号地址可信，才能放心用 `readelf` 查出来的地址读变量。

### 10.3 分清"测到的"和"推断的"

这轮排查里犯过的两个错，都值得记下来：

- 把**下板 STM32 的 `RXF0C`（F0S=3）**当成上板的，推出"FIFO 只有 3 格"的错误前提；
- 按 **Bosch 的 PSR 位定义**解 HPM 的 PSR，把 `EW` 读成 `BO`，一度误判"仍在 bus-off"。

对策：**每个寄存器值都回头文件核一遍位定义；每个结论都标注它是"实测"还是"推断"。**

### 10.4 警惕"测量本身扰动被测对象"

- 每次 halt 会让 CPU 停几十~几百毫秒 → 1kHz 通路上必然丢帧；
- 因此 `RXF0S` 的"FIFO 满 / 消息丢失"标志**在你读数的那一刻出现是必然的**，
  不能用来证明"正常运行也在丢帧"；
- 想量化丢帧/速率，用第 7.3 节的"长窗口静置 + 应用计数器"。

### 10.5 需要量化"时间"时的替代手段

调试器测不了亚毫秒时序，可以：

- **DWT 周期计数器**（Cortex-M）：

```
mww 0xE000EDFC 0x01000000     # DEMCR.TRCENA
mww 0xE0001000 0x00000001     # DWT_CTRL.CYCCNTENA
mdw 0xE0001004 1              # 读 DWT_CYCCNT（32 位，会回绕，注意分多段累加）
```

  用"括号住 halt 的 wall 时间"误差很大（探针开销可达数百毫秒），
  最好**多段累加**抵消固定开销。
- **应用里加最小 instrumentation**（一个 `atomic_inc` 或 cycle 差值）
  比用调试器猜可靠得多。

---

## 11. 实战案例（这套方法查出过的三个真问题）

### 案例 1：上板 63B FD 帧"一帧都发不出去，且没有任何日志"

- 现象：`TXBTO=0`、`TXBRP=0`、`ECR=0`，RTT 里没有任何错误日志。
- 寄存器：`CCCR.FDOE=0`（没进 FD 模式）、`DBTP` 是复位默认值（从没被写过）。
- 编译产物：`can tx busy` 这个字符串**不在 ELF 里**，全 ELF 找不到 `Can::Send` 调用。
- 根因：板级 conf 没有 `CONFIG_CAN_FD_MODE=y` → `CAN_MAX_DLEN=8` →
  `SendFrame()` 里 `len > CAN_MAX_DLEN` 对 63B 帧永远成立 → `-O2` 把整条发送路径
  （连日志）当死代码删掉。

**教训**：寄存器能证明"从来没发过"，编译器产物能解释"为什么连日志都没有"。

### 案例 2：下板实际位率只有标称的 1/5

- 现象：USB-CAN 模块按 1 Mbps 收不到任何数据；上板也完全收不到下板。
- 寄存器：`NBTP` 折算 = 1 Mbps（**按 DT 假设的 120MHz** 算）；
  但 `RCC_D2CCIP1R.FDCANSEL[29:28] = 0` → CAN 内核时钟实际是 **HSE 24MHz**
  → 真实位率 = `1Mbps x 24/120 = 200 kbps`。
- 根因：下板 overlay 的 `&fdcanN` 只配了 `clocks = <&rcc STM32_CLOCK(APB1_2, 8)>`，
  **没有第二项内核时钟源选择**，FDCANSEL 保持复位默认值。
- 修法：加 `<&rcc STM32_SRC_PLL1_Q FDCAN_SEL(1)>`；修完 `FDCANSEL` 读回 1、
  `TEC=0`、`TXBTO` 非 0，链路立通。

**教训**：位率不能只看 `NBTP`，要连**内核时钟源**一起核；
"某个外设的内核时钟选择寄存器复位默认是 HSE"这类默认值非常容易踩。

### 案例 3："丢帧 57%" —— 其实是发送周期不是 1ms

- 现象：上板 RX FIFO 长期显示满 + `RF0L`（丢帧），像是收帧来不及。
- 第一次量：上板 `inter_rx::asm_.rx_cnt` 静置 25 秒 → **432.7 帧/秒**（标称 1000）。
- 换到下板量它收到的帧 → **638 帧/秒** = 上板的实际发送速率。
- 结论：**两侧都不是丢帧，而是两边发送任务周期被拉长了**：

```cpp
// 相对写法：周期 = 干活时间 + 睡眠
elapsed = k_uptime_get() - tick_start;
remain  = kPeriodMs - elapsed;         // k_uptime_get() 只有 1ms 分辨率
if (remain > 0) k_msleep(remain);      // k_msleep(1) 实际睡 2~3 个 tick
```

  干活 <1ms 时 `elapsed` 读出来是 0，于是每轮白送 0.25~0.5ms；
  上板实测周期 1.57ms、下板 2.31ms。

**教训**：

1. "丢帧"结论要看**两侧的速率**，不能只看一侧；
2. 调试器 halt 造成的丢帧是你自己的影子；
3. 控制环/广播环的周期要用**绝对时刻调度**，否则工作量会被加到周期上，
   而 PID 的 `dt` 若是写死的常数，等效增益会系统性偏移（云台环就属于这种情况）。

---

## 12. 坑与故障排除清单

| 症状 | 原因 | 处理 |
|---|---|---|
| `could not read product string` / `CMD_INFO failed` | 探针假死 | `hpm-probe-usb reset/unbind-cdc/bind-cdc`，无效则拔插 |
| 烧录/调试把**另一块板**的探针复位了 | wrapper 按 VID 只匹配第一个设备 | 手动调试不要用 wrapper；或按 serial 指定探针 |
| `Address already in use`（3333/4444/6666） | 另一个 OpenOCD（下板 / VS Code）在跑 | 换端口，或先停掉对方 |
| `invalid command name "gdb"` | 系统版 OpenOCD 用 `gdb_port` | 改成 `gdb_port/telnet_port/tcl_port`，或干脆用默认值 |
| 一挂 GDB 芯片就复位 | cfg 里 `gdb-attach { reset halt }` | 覆盖成 `{halt}` |
| `Failed to read priv register` | HPM 运行中读内存（OpenOCD 的访问路径不行） | 用 `rtt-sba.tcl` 的 SBA 读；实在不行再 `halt` → 读 → `resume` |
| `Cannot insert hardware breakpoint` | 这块核 + 该 OpenOCD 不支持 | 不用断点，改读寄存器/变量 |
| `target not halted` / `resume` 报错导致 openocd 退出 | 目标本来就在跑 | 用 `if {[catch {resume} msg]}` 包住 |
| FIFO 满/丢帧标志总是置位 | **你自己的 halt 造成的** | 用长窗口静置 + 应用计数器 |
| 读到"看起来不可能"的值 | 位定义/地址搞错 | 回 SDK/CMSIS 头核对；符号地址重新 `readelf` 查 |
| 探针退出后整体不可用 | `kill` 打断了 USB 传输 | 用 `shutdown` 优雅退出；最后 `resume` |

---

## 13. 附录：Python 读取模板

工程里已经有现成的登录/读字封装，直接用最省事：

```python
import sys, time, struct
sys.path.insert(0, '/home/rookie/zephyrproject/projects/Dust_Infantry_Head/tools')
from rtt_hpm import OcdTelnet          # 极简 openocd telnet 客户端

o = OcdTelnet('127.0.0.1', 4445)       # 上板 telnet 口；下板是 4444

def w(addr, n=1):
    return o.words(addr, n)            # mdw：读 32 位字

def b(addr, n):
    return o.bytes_(addr, n)           # mdb：读字节

o.cmd('halt')                          # 读内存前必须停核
F = 0xF028C000                         # mcan3（下板换成 0x4000A400）
cc    = w(F + 0x18)[0]                 # CCCR
ecr   = w(F + 0x40)[0]                 # ECR
txbto = w(F + 0xD8)[0]                 # 发送成功掩码
rx0s  = w(F + 0xA4)[0]                 # RXF0S

# 读 RX FIFO 内容（上板 mcan3：message RAM = 0xF0400000 + F0SA）
f0c = w(F + 0xA0)[0]
mem = 0xF0400000 + (f0c & 0xFFFC)
for i in range(min(rx0s & 0x3F, 3)):
    el = w(mem + i * 72, 18)           # 72B/元素（64B 负载）
    r0, r1 = el[0], el[1]
    cid = ((r0 >> 18) & 0x7FF) if not ((r0 >> 30) & 1) else (r0 & 0x1FFFFFFF)
    print('id=0x%03X dlc=%d FDF=%d BRS=%d' %
          (cid, (r1 >> 16) & 0xF, (r1 >> 21) & 1, (r1 >> 20) & 1))

print('CCCR=0x%08X ECR=0x%08X TEC=%d REC=%d TXBTO=0x%X RXF0S=0x%08X'
      % (cc, ecr, ecr & 0xFF, (ecr >> 8) & 0x7F, txbto, rx0s))

o.cmd('resume')
o.cmd('shutdown')                      # 优雅退出，避免把探针搞卡
o.close()
```

速率测量（长窗口静置）模板：

```python
def snap():
    o.cmd('halt')
    v = w(ASM_ADDR, 20)                # 连计数器附近一起读
    o.cmd('resume')
    return v

a = snap(); t1 = time.time()
time.sleep(25)                         # 中间不要有任何调试器交互
b = snap(); t2 = time.time()
for i, (x, y) in enumerate(zip(a, b)):
    print('word[%2d] %10d -> %10d  d=%d' % (i, x, y, y - x))
# 差值最大且单调增的那个字就是计数器；rate = d / (t2 - t1)
```

---

## 14. 一句话总结

**建三个通道：读寄存器/内存（OpenOCD telnet）、看日志（RTT）、符号化（GDB/addr2line）；
然后按"外设寄存器 → 驱动 → 应用变量"的顺序取证。
所有时序类结论必须用应用自己的计数器在长窗口里量，
而不是用会打断 CPU 的调试器去采样。**
