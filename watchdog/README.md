# Watchdog 分层

```text
业务线程
  | wdg_health::Mark()
健康监测层
  | wdt_supervisor::Init/Run()
Zephyr watchdog API
  | zephyr,watchdog
SoC watchdog 驱动
```

- `supervisor.*`：预热、心跳检查、故障锁存、喂狗。
- `health.*`：业务心跳计数，不依赖 SoC。
- `adapters/<soc>/`：寄存器、时钟、中断和复位原因等硬件细节。
- 业务代码只看 `CONFIG_WDT_SUPERVISOR`，不引用具体芯片驱动。

移植新 SoC：

1. 启用该 SoC 的 Zephyr watchdog 驱动。
2. 在板级 devicetree 设置 `chosen { zephyr,watchdog = &wdtX; };`。
3. 设置 `CONFIG_WDT_SUPERVISOR=y` 和通用超时参数，无需修改监督线程。
