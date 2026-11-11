# Dust_Infantry_Head

RoboMaster 步兵机器人**上板**固件，基于 Zephyr + Dust 架构。上板负责读取遥控器和
上位机数据，完成自瞄目标折算与状态聚合，并通过 CAN FD 与下板交换数据。

## 仓库与目录

上板是与下板平级的应用仓库，不依赖下板源码。当前开发环境使用以下目录布局：

```text
~/zephyrproject/
├── zephyr/                         # 社区 Zephyr
├── modules/                        # Zephyr 社区模块
├── zephyr_user/                    # Dust_Zephyr_Tree
└── projects/
    ├── Dust_Infantry_Head/         # 本仓库
    └── Dust_Infantry_Body/         # 下板仓库

~/Zephyr_HPMicro/
├── sdk_env/                        # 社区 sdk_env，内含 hpm_sdk
├── sdk_glue/                       # 社区 zephyr_sdk_glue
└── sdk_glue_user/                  # Dust_Zephyr_HPMicro_Tree
```

- `Dust_Zephyr_Tree` 提供通用框架、CMSIS 兼容层和共享板卡定义；其中的
  `framework/` 已转换为普通目录，不再需要 Git submodule。
- `Dust_Zephyr_HPMicro_Tree` 提供 HPM5361ICB、HPM5E3Y 板级与 SoC 支持。
- `zephyr`、`sdk_env`、`sdk_glue` 保持社区上游仓库；本地修改通过两个自有仓库中的
  补丁应用。

## 硬件与板卡配置

- STM32H723VGT6：板卡 `stm32h723vgt6`，上板配置 `board_dm_mc02`。
- HPM5361：板卡 `hpm5361icb`。
- STM32F407：板卡 `stm32f407igh6`，上板配置 `board_rm_c`。

应用自己的 `boards/` 只保存产品相关的 overlay、conf、调试器配置和 `board.cmake`；
芯片级板卡定义由 `Dust_Zephyr_Tree.boards` 提供。

## 主要业务线程

- `imu`：惯性数据与姿态
- `remote`：遥控输入
- `pc`：上位机通信
- `booster`：发射控制
- `mcu_inter` / `inter_bus` / `inter_rx`：上下板 CAN FD 收发与解析
- `watchdog`：任务健康检查和硬件看门狗监督

## 编译

先加载 Zephyr、工具链和 West 环境：

```bash
source ~/zephyrproject/setup_env.sh
```

已验证的 HPM5361 构建：

```bash
west build -p always -b hpm5361icb -d build -- -DBOARD_CFG=hpm5361icb
```

STM32H723 上板配置：

```bash
west build -p always -b stm32h723vgt6 -d build -- -DBOARD_CFG=board_dm_mc02
```

> 当前 `prj.conf` 仍包含 HPM 专属的 `CONFIG_WDT_HPMICRO=y`。该配置移到
> `boards/hpm/hpm5361icb/hpm5361icb.conf` 之前，STM32H723 构建会在 Kconfig
> 阶段报未定义符号。

## 烧录与调试

```bash
./flash.sh jlink        # J-Link
./flash.sh dap-cmsis    # CMSIS-DAP；HPM 上会经过探针 wrapper
```

SEGGER RTT 和 OpenOCD 调试说明见 `docs/`。RTT 控制文件位于
`/tmp/dust_infantry_head_rtt_openocd.*`。

## 自瞄控制延迟诊断

本仓库提供默认关闭的 `CONFIG_LATENCY_TRACE`，用于测量：

```text
上板确认一帧完整、CRC/量程均有效的 PC 自瞄数据
    -> 上板把 CAN 0x100 提交到 MCAN TX FIFO
    -> 下板 CAN 0x100 RX 中断
    -> 下板向 yaw / pitch 电机提交控制帧
```

详细的测量契约和日志说明也见 [`docs/latency_trace.md`](docs/latency_trace.md)。

### 实现方式

- PC 链路在 [`thread/pc/trd_pc.cpp`](thread/pc/trd_pc.cpp) 完成 CRC 和量程检查后，
  用 `k_cycle_get_32()` 记录有效帧到达时刻。
- [`thread/mcu_inter/trd_mcu_inter.cpp`](thread/mcu_inter/trd_mcu_inter.cpp)
  检测到新的 PC 帧后，用 `pc.version` 的低 4 位生成诊断序号，放在
  `CommState.flags[7:4]`。500 Hz 时序号每 32 ms 回绕一次。
- 上板段 `upper_us` 从“有效 PC 帧确认”计到 `Can::Send()` 成功提交驱动 TX FIFO。
- 下板段从 CAN RX 中断计到 `can3.Send()` 前，yaw 和 pitch 分别计时；下板通过
  原有 CAN 0x200 下行帧新增的 `TraceState` 把两个耗时回传上板。
- 上板统计模块在 [`diagnostics/latency_trace.cpp`](diagnostics/latency_trace.cpp)
  按序号配对，分别计算 yaw/pitch 的平均值、p50、p95、p99 和最大值。

上板不能使用 HPM MCAN 的 TX complete 回调作为终点。当前驱动的 TXBTO 完成事件
回收存在约 20 ms 的滞后，会把正常数毫秒的链路误报成 20~45 ms。提交到 TX FIFO
到下板 RX 中断之间还有约 0.1~0.3 ms 的邮箱、总线和中断时间，日志中的
`total_us` 不包含这一小段。

### 构建与烧录

上板的诊断开关现在放在 `prj.conf`。测量构建建议同时关闭看门狗：

```conf
CONFIG_WDT_SUPERVISOR=n
CONFIG_LATENCY_TRACE=y
```

构建和烧录：

```bash
source ~/zephyrproject/setup_env.sh
west build -p always -b hpm5361icb -d build -- -DBOARD_CFG=hpm5361icb
grep '^CONFIG_LATENCY_TRACE=y$' build/zephyr/.config
west flash -d build
```

上板和下板必须使用同一版 `inter_cmd/inter_cmd.hpp`，并且都打开
`CONFIG_LATENCY_TRACE=y`。不能只烧上板：正常下板的 0x200 帧为 16 B，诊断下板
的下行帧为 28 B，并在末尾携带 `TraceState`。

如果之前用 `-DEXTRA_CONF_FILE=latency_trace.conf` 构建过，现在已把开关移到
`prj.conf` 并删除了该文件，CMake Cache 仍会保留旧路径并报：

```text
File not found: .../latency_trace.conf
```

这时必须做一次 pristine 重建，并且不要再传 `EXTRA_CONF_FILE`。直接修改
`prj.conf` 后的普通增量构建无法删除已经写入 CMake Cache 的参数。

### RTT 使用

先启动 OpenOCD/RTT 服务：

```bash
tools/rtt.sh -d build -i dap-cmsis
```

另开终端读取 HPM 的 SBA RTT：

```bash
python3 tools/rtt_hpm.py \
  -e build/zephyr/zephyr.elf \
  --poll-ms 500
```

测量期间保持：

- PC 数据约为 500 Hz，每 2 ms 一帧。
- 自瞄模式为 `mode=1` 或 `mode=2`，不要持续发 `mode=0`。
- 遥控自瞄开关打开，遥控链路、上板 IMU 和云台电机反馈正常。
- yaw/pitch 电机已使能，云台没有停机或故障。
- 先丢弃启动和模式切换阶段，稳定采集至少 30 秒。

如果只看到普通日志而没有 `LT total`，依次检查 `CONFIG_LATENCY_TRACE`、两板是否
都烧录了诊断固件，以及自瞄是否真正进入 `VisionOk`。

### 日志解释

上板每 2 秒输出两行：

```text
LT total seq=... yaw n=... avg(u/l/t)=... p50=... p95=... p99=... max=... over=... us
LT total seq=... pitch n=... avg(u/l/t)=... p50=... p95=... p99=... max=... over=... us
```

- `n`：2 秒窗口内成功配对的样本数，理论值约 1000。
- `avg(u/l/t)`：上板段平均、下板段平均、两者之和的平均，单位 µs。
- `p50/p95/p99/max`：总耗时的分位数和最大值。直方图按 10 µs 分档，并统计
  超过 20 ms 的溢出样本。
- `over`：总耗时超过 20 ms 的样本数。

PC 发送频率有抖动时，`n` 会低于 1000。一个 1 ms 上板轮询周期内如果收到两帧，
`pub_pc_to` 的 level 语义只保留最新帧，中间帧可能不会形成独立诊断样本。
因此应结合 `n` 和实际 PC 帧率解释结果，不能只把 `n < 1000` 判成链路丢帧。

当前板卡稳定实测的软件分段约为：

```text
上板段平均：0.87~0.94 ms
下板段平均：1.28~1.50 ms
软件总计平均：2.2~2.4 ms
p95：约 3.0~3.3 ms
p99：约 3.3~3.6 ms
偶发最大：约 9~12 ms
```

计入提交 FIFO 到下板 RX 中断的约 0.1~0.3 ms 后，实际端到端平均约为
2.3~2.7 ms。该结果只代表当前板卡、当前 PC 帧率和负载，不应直接作为其他版本的
固定指标。

### 与看门狗同时运行

延迟诊断会改变实时负载，不应和用于正式保护的看门狗监督混在同一个测量结论里。
当前监督器每 50 ms 检查服务心跳：

- IMU、Booster、McuInter、InterRx、PC 连续约 500 ms 未更新即判故障。
- Remote 连续约 750 ms 未更新即判故障。
- 故障一旦锁存，监督器停止喂狗，硬件在 2 秒后复位；重启后可能再次重复。

上板 fault mask：

| bit | 服务 |
|---:|---|
| `0x01` | IMU |
| `0x02` | Remote |
| `0x04` | Booster |
| `0x08` | McuInter |
| `0x10` | InterRx |
| `0x20` | PC |

出现复位时先捕获：

```text
watchdog health fault mask=0x........
watchdog pre-reset fault=0x........
```

有非零 mask 表示监督器主动停喂；mask 为 0 仍复位则优先检查监督线程是否被饿死、
硬件喂狗是否失败。为了取得干净的延迟数据，诊断构建可以关闭
`CONFIG_WDT_SUPERVISOR`；正式控制固件应单独验证看门狗，不要用关闭看门狗的结果
代替正式保护测试。
