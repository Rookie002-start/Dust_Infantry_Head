# SEGGER RTT 调试日志通道（本工程已部署）

RTT（Real Time Transfer）用调试探针（SWD/JTAG）直接读写目标板 RAM 里的一小块环形缓冲，
把日志 / 变量实时送到电脑上：

* 不占 UART，不阻塞控制线程（主机没连、缓冲满时直接丢数据）；
* 速度比串口快几个数量级，适合刷高频变量；
* 本工程用的是 **OpenOCD 自带的 RTT 实现**，所以 CMSIS-DAP / ST-Link / J-Link 都能用，
  不必安装 SEGGER 的 J-Link 软件。

## 1. 一分钟上手

```bash
# 1) 编译 + 烧录（照旧）
west build -d build
./flash.sh                # 或 ./flash.sh jlink

# 2) 起 RTT 服务（另开一个终端，别关）
tools/rtt.sh

# 3) 再开一个终端看输出
nc localhost 9090
#    或者用自带查看器（Ctrl-C 退出）：
tools/rtt_view.py
```

只想一步到位，可以直接：

```bash
tools/rtt.sh -v
```

VS Code 用户更省事：`.vscode/launch.json` 里的 6 个 cortex-debug 配置
（CMSIS-DAP / ST-Link / J-Link 的 Debug 与 Attach）都已打开 RTT，
地址用 `"address": "auto"` 从 ELF 的 `_SEGGER_RTT` 符号自动取，
启动调试后 RTT 输出会出现在调试会话的 **RTT** 终端里。

## 2. 代码里怎么打日志 / 打变量

### 2.1 printk() —— 不用改代码，自动进 RTT

Zephyr 默认 `CONFIG_LOG_PRINTK=y`，`printk()` 会被日志子系统接管，
所以工程里**已有的** `printk(...)`（例如 `thread/test/trd_test.cpp` 里的 IMU 姿态）
现在会同时出现在 UART 和 RTT 上，浮点 `%.3f` 也能用。

### 2.2 LOG_INF / LOG_WRN / LOG_ERR / LOG_DBG —— 推荐

```c
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(my_mod, LOG_LEVEL_INF);

LOG_INF("roll=%.3f pitch=%.3f yaw=%.3f", (double)r, (double)p, (double)y);
LOG_DBG("raw: %u %u %u", a, b, c);
```

* 日志同时输出到 **UART + RTT**；
* 支持浮点（工程已开 `CONFIG_CBPRINTF_FP_SUPPORT=y`）；
* 想临时看 `LOG_DBG`：把模块日志级别临时改成 `LOG_LEVEL_DBG`。

### 2.3 SEGGER_RTT_printf() —— 直接写 RTT（最轻量）

```c
#include <SEGGER_RTT.h>      /* 仅在 CONFIG_DEBUG_RTT 打开时可用 */

SEGGER_RTT_printf(0, "tick=%u  imu_age=%u ms\r\n", tick, age_ms);
SEGGER_RTT_WriteString(0, "here!\r\n");
```

**注意：SEGGER 自带的 printf 不支持 `%f`**（只支持 `%d / %u / %x / %c / %s` 等）。
要打小数就自己缩放成整数（例如 `(int)(value * 1000)`），或者改用 `LOG_INF`。

`thread/test/trd_test.cpp` 里加了一段演示（`CONFIG_DEBUG_RTT` 打开时才编译），
烧录后每 2 秒一行 `[rtt] tick=... uptime=... imu_age=...`，可以用来确认链路是通的。

## 3. 本次改了什么

| 文件 | 作用 |
| --- | --- |
| `modules/segger/` | 工程自带的 SEGGER RTT 模块：`SEGGER/`（RTT 7.58c 原版源码）+ `Config/SEGGER_RTT_Conf.h`（Zephyr 版配置）+ `zephyr/module.yml`（`cmake-ext`/`kconfig-ext`，构建胶水直接复用 `$ZEPHYR_BASE/modules/segger`）。原因见该目录 `README.md` |
| `Kconfig` | 新增 `CONFIG_DEBUG_RTT`：一键打开 `USE_SEGGER_RTT` + `LOG_BACKEND_RTT`（并显式 `select HAS_SEGGER_RTT`，让非 STM32 板卡也能用） |
| `prj.conf` | `CONFIG_DEBUG_RTT=y` + 缓冲区大小 / 非阻塞模式等参数 |
| `CMakeLists.txt` | 把 `modules/segger` 加进 `ZEPHYR_EXTRA_MODULES`；把 `SEGGER_RTT_printf.c` 与头文件路径挂到 app |
| `tools/rtt.sh`、`tools/rtt_view.py` | 主机侧 RTT 服务与终端查看器（OpenOCD 实现） |
| `.vscode/launch.json` | 6 个 cortex-debug 配置打开 `rttConfig`（`address: auto`） |
| `thread/test/trd_test.cpp` | 一段 RTT 演示输出（受 `CONFIG_DEBUG_RTT` 保护） |

## 4. 配置项（`prj.conf`）

| 配置 | 当前值 | 含义 |
| --- | --- | --- |
| `CONFIG_DEBUG_RTT` | `y` | 总开关，关掉即回到没有 RTT 的状态 |
| `CONFIG_SEGGER_RTT_BUFFER_SIZE_UP` | `4096` | 目标→主机缓冲；越大越不容易丢日志，代价是 RAM |
| `CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN` | `32` | 主机→目标缓冲（目前用不到输入，留小值即可） |
| `CONFIG_SEGGER_RTT_PRINTF_BUFFER_SIZE` | `128` | `SEGGER_RTT_printf()` 的攒包缓冲 |
| `CONFIG_SEGGER_RTT_MODE_NO_BLOCK_SKIP` | `y` | 通道 0 非阻塞：主机没连 / 缓冲满就丢，**不会阻塞线程** |
| `CONFIG_LOG_BACKEND_RTT_MODE_OVERWRITE` | `y` | 日志缓冲满时覆盖最旧数据（另一选项 `BLOCK` 会阻塞线程，不建议） |
| `CONFIG_SEGGER_RTT_SECTION_CUSTOM` | `y`（默认） | 控制块 / 缓冲放在 `.rtt_buff_data` 段（RAM 起始，约 `0x20000000`），主机好找 |
| `CONFIG_LOG_BACKEND_RTT_BUFFER` | `0` | 日志走 RTT 通道 0（与 `SEGGER_RTT_printf` 同一通道） |

内存 / Flash 代价（rm_c / stm32f407igh6 实测，`west build` 结尾的 `Memory region Used Size`）：

| | FLASH | RAM |
| --- | --- | --- |
| `CONFIG_DEBUG_RTT=n` | 80068 B | 18600 B |
| `CONFIG_DEBUG_RTT=y`（当前） | 82872 B | 22912 B |
| 差值 | +2.8 KB | +4.3 KB（`.rtt_buff_data` = `0x10c8`，其中 4 KB 是上行缓冲） |

不需要时把 `CONFIG_DEBUG_RTT=n` 即可（`modules/segger/` 会空编译，不占空间）。

想看当前用到的地址：

```bash
arm-zephyr-eabi-nm build/zephyr/zephyr.elf | grep -E "_SEGGER_RTT|__rtt_buff"
```

## 5. 只让日志走 RTT（省掉串口开销）

现在是 "UART + RTT 双份"。如果串口带宽成了瓶颈（例如 100000 波特率的调试口），
可以改成只走 RTT：

1. 打开 `Kconfig`，把 `config DEBUG_LOG` 里的 `select LOG_BACKEND_UART` 注释掉；
2. （可选）想连 `printk` 都直接写 RTT 而不经日志子系统：`prj.conf` 里
   `CONFIG_UART_CONSOLE=n` + `CONFIG_RTT_CONSOLE=y`。

> 不要同时开 `UART_CONSOLE` 和 `RTT_CONSOLE`：两者都往 `printk` 上挂钩子，
> 初始化顺序决定谁生效，容易出现"到底是哪个在输出"的困惑。

## 6. 常见问题

**Q: `tools/rtt.sh` 报 "在 ELF 里找不到 _SEGGER_RTT 符号"**
固件没开 `CONFIG_DEBUG_RTT`（或没重新编译）。确认 `prj.conf` 里 `CONFIG_DEBUG_RTT=y` 后重新 `west build`。

**Q: OpenOCD 报 `No control block found`**
RTT 控制块是**固件运行时**在 RAM 里初始化的，所以：

* 板子必须已经烧录并且跑起来（不要停在复位 / 下载状态）；
* 刚 `west flash` 完可以直接连；如果板子被 halt 住了，用 `tools/rtt.sh -r` 复位一下再连；
* 换固件后控制块地址可能变化，脚本每次都从 ELF 重新读，不用手工填。

**Q: 探针被占用 / OpenOCD 起不来**
调试探针同一时刻只能被一个进程占用。VS Code 调试会话（cortex-debug）已经占用了探针时，
`tools/rtt.sh` 会连不上；反过来也一样。要么关掉其中一个，要么直接用 VS Code 里已打开 RTT 的配置。

**Q: 手搓 openocd 命令时，服务刚起来就自己退出（退出码 1）**
OpenOCD 的 `-c` 命令一旦返回错误，它就会**直接退出**。典型坑是 `resume`：
目标本来就在跑时，`resume` 会返回 "target not halted" 错误，把整个 OpenOCD 带走。
`tools/rtt.sh` 已经用 `if {[catch {resume} msg]} { ... }` 把它包住，只在目标真的停住时才恢复运行。
同理 `rtt start` 也做了 100ms×30 的重试（复位后控制块要等固件启动才出现）。

**Q: 输出里每行结尾多一个 `^M`（`\r\r\n`）**
`printk("...\r\n")` 里的 `\r` 加上日志后端的 LF→CRLF 转换，就会变成两个 CR。
终端上看不出问题，重定向到文件会多一个字符。想要干净的单换行，在 `prj.conf` 加上
`CONFIG_LOG_BACKEND_CRLF_LFONLY=y`（UART 日志也会一起变干净）。

**Q: CMSIS-DAP 提示权限问题**
只有 `board_dm_mc02` 有 `openocd-cmsis-dap-wrapper.sh`（里面用 `sudo` 解绑 `cdc_acm`），
`tools/rtt.sh` 在该板卡上会复用它；其它板卡（如 `board_rm_c`）直接调 `/usr/bin/openocd`，
实测 Horco CMSIS-DAP 探针可以直接用，不需要 sudo。若报 `Permission denied`，
说明当前用户对探针没有权限，可加 udev 规则或临时用 sudo 运行。

**Q: 日志偶尔缺行**
说明主机没跟上的那一瞬间缓冲满了。调大 `CONFIG_SEGGER_RTT_BUFFER_SIZE_UP`（占 RAM），
或者用 `CONFIG_LOG_BACKEND_RTT_MODE_OVERWRITE`（默认）保证拿到的是最新数据。

**Q: 想用 SEGGER 官方软件看**
装了 SEGGER J-Link 软件包后，也可以用 `JLinkRTTViewer` / `JLinkRTTClient`
（搜 "SEGGER RTT" 或选 `_SEGGER_RTT` 符号），效果一样。本工程默认不需要它。

**Q: HPM5361（RISC-V）能用吗？**
固件侧已经放开（`CONFIG_DEBUG_RTT` 里显式 `select HAS_SEGGER_RTT`），
`tools/rtt.sh` 也会自动切到先楫的 OpenOCD + `boards/hpm/hpm5361icb/openocd-debug.cfg`。
但 OpenOCD 0.12 的 RTT 对 RISC-V 目标的支持不如 Cortex-M 成熟，
如果 `rtt setup` / `rtt start` 失败，改用 SEGGER J-Link 官方软件的 RTT Viewer 更稳。

## 7. 关掉 RTT

* 临时关：`prj.conf` 里 `CONFIG_DEBUG_RTT=n`；
* 彻底移除：删掉 `modules/segger/`、去掉根 `CMakeLists.txt` 里 `ZEPHYR_EXTRA_MODULES`
  的那一行、`Kconfig` 里的 `DEBUG_RTT` 段、`prj.conf` 里的 RTT 段。

以后如果 `west update` 拉到了官方 `segger` 模块，注意**不要**和本目录同时存在
（同名模块会冲突）：删掉 `modules/segger/` 和那行 `ZEPHYR_EXTRA_MODULES` 即可，
`prj.conf` / `Kconfig` 配置全部不用改。
