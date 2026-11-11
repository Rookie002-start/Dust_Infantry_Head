# 自瞄控制延迟软件追踪

本功能测量：

```text
上板确认一帧完整、CRC/量程均有效的 PC 自瞄数据
    -> 上板 CAN 0x100 成功提交到驱动 TX FIFO
    -> 下板 CAN 0x100 RX 中断
    -> 下板向 yaw / pitch 电机提交控制帧
```

默认关闭，不改变正常固件的帧长和内存布局。上下板必须同时使用
`CONFIG_LATENCY_TRACE=y` 并保持 `inter_cmd/inter_cmd.hpp` 一致。

## 数据流

- PC 以 500 Hz 发送自瞄数据，正常应该每 2 ms 一帧。
- 上板在 `trd_pc` 完成 CRC 和量程检查后取 `k_cycle_get_32()`。
- 每次出现新的有效 PC 帧，上板用 `PC 帧 version` 的低 4 位作为序号，
  放在 `CommState.flags[7:4]`。500 Hz 下序号每 32 ms 回绕一次。
- 上板在 `Can::Send()` 成功把 CAN 帧提交给驱动后计算 `upper_us`。
- 下板在 CAN RX 回调中记录本地周期；云台线程看到序号变化后，在
  `can3.Send()` 前分别记录 yaw / pitch 的 `lower_us`。
- 下板把 `TraceState` 加入原有 CAN 0x200 下行帧（16 B 扩展为 28 B），
  上板按序号配对并计算 `upper_us + lower_us`。
- 上板每 2 秒输出两行 `LT total` 日志，分别是 yaw 和 pitch。

跨板时钟不做相减。未包含的时间是“上板提交 TX FIFO”到“下板 RX 中断”
之间的邮箱调度、总线仲裁/传输和中断进入时间；无拥堵时约为 0.1~0.3 ms。

这里不能使用 HPM MCAN 的 TX complete 回调作为终点：当前驱动的 TXBTO
完成事件回收会产生约 20 ms 的回调滞后，测到的是驱动回收延迟，不是业务延迟。

## 构建与烧录

上板的开关位于 `prj.conf`：

```conf
CONFIG_WDT_SUPERVISOR=n
CONFIG_LATENCY_TRACE=y
```

下板目前仍可使用 `-DEXTRA_CONF_FILE=latency_trace.conf`。

上板：

```bash
cd ~/zephyrproject/projects/Dust_Infantry_Head
west build -p always -b hpm5361icb -d build -- -DBOARD_CFG=hpm5361icb
```

下板：

```bash
cd ~/zephyrproject/projects/Dust_Infantry_Body
west build -p always -b stm32h723vgt6 -d build-latency -- \
  -DBOARD_CFG=board_dm_mc02 \
  -DEXTRA_CONF_FILE=latency_trace.conf
```

确认配置：

```bash
grep '^CONFIG_LATENCY_TRACE=y$' build-latency/zephyr/.config
```

上板检查 `build/zephyr/.config`，下板检查 `build-latency/zephyr/.config`。
两板必须一起烧录诊断构建，不能一块诊断、一块正常固件。

烧录命令：

```bash
cd ~/zephyrproject/projects/Dust_Infantry_Head
west flash -d build

cd ~/zephyrproject/projects/Dust_Infantry_Body
west flash -d build-latency
```

如果上板之前使用过 `EXTRA_CONF_FILE=latency_trace.conf`，后来把开关移到
`prj.conf` 并删除该文件，旧 CMake Cache 仍会引用不存在的文件。必须省略
`EXTRA_CONF_FILE` 并做一次 `west build -p always`。

## 测量条件

- PC 固定为 500 Hz，每 2 ms 一帧。
- 自瞄模式保持 `mode=1` 或 `mode=2`，不要发 `mode=0`。
- 遥控自瞄开关打开，遥控链路在线。
- 云台 yaw / pitch 电机已使能，未触发停机、故障或掉线。
- 不在 500 Hz 路径里打印日志；固件只做内存累加，每 2 秒汇总一次。
- 先丢弃启动后前 5 秒数据，再至少采集 30 秒。按 `p95`、`p99`、`max`
  评价抖动，不要只看平均值。

## 日志解释

上板每 2 秒输出：

```text
LT total seq=... yaw n=... avg(u/l/t)=... p50=... p95=... p99=... max=... over=... us
LT total seq=... pitch n=... avg(u/l/t)=... p50=... p95=... p99=... max=... over=... us
```

- `n`：统计窗口内配对成功的样本数，500 Hz、2 秒理论值是 1000。
- `avg(u/l/t)`：上板平均耗时 / 下板平均耗时 / 两者之和的平均值。
- `p50/p95/p99/max`：上板加下板总耗时的分布，单位 µs。
- `over`：总耗时超过 20 ms、落入最后一个溢出 bin 的样本数。

如果 `n` 明显低于 1000，依次检查：

1. PC 实际发送频率是否稳定在 500 Hz。
2. 自瞄开关和 `VisionOk` 是否一直有效。
3. 电机反馈是否正常，云台是否处于使能控制态。
4. 两板是否都烧录了带 `CONFIG_LATENCY_TRACE=y` 的同一版协议。

## 关闭诊断

省略 `EXTRA_CONF_FILE=latency_trace.conf` 重新构建即可。默认配置下：

- 上行 CAN 0x100 仍为 63 B。
- 下行 CAN 0x200 保持 16 B。
- 不编译上板统计模块，不传序号，不记录 RX 周期。

测量完成后必须把两板都恢复为默认构建。

## 与看门狗同时运行

延迟诊断会改变实时负载，不能把看门狗复位误认为延迟算法或硬件 CAN 的随机故障。
监督器每 50 ms 检查一次服务心跳，连续约 500 ms 没有新计数就锁存故障并停止喂狗；
HPM/STM32 硬件随后按 2 秒超时复位。由于重启后同样负载仍存在，故障可能周期性
重复。

上板 fault mask：

| bit | 服务 |
|---:|---|
| `0x01` | IMU |
| `0x02` | Remote |
| `0x04` | Booster |
| `0x08` | McuInter |
| `0x10` | InterRx |
| `0x20` | PC |

下板 fault mask：

| bit | 服务 |
|---:|---|
| `0x01` | Chassis |
| `0x02` | Gimbal |
| `0x04` | McuInter |
| `0x08` | InterRx |

复位前优先查找：

```text
watchdog health fault mask=0x........
watchdog pre-reset fault=0x........
```

非零 mask 表示监督器检测到对应服务停顿并主动停喂；mask 为 0 仍复位时，应检查
监督线程是否被高优先级任务长期饿死或 `wdt_feed()` 是否失败。

为了保证延迟数据的可比性，测量构建应关闭 `CONFIG_WDT_SUPERVISOR`。正式固件必须
独自完成看门狗压力测试，不能用关闭看门狗时的延迟结果替代正式实时性验证。
