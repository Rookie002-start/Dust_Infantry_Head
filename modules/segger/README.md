# modules/segger —— 工程自带的 SEGGER RTT

## 这个目录是什么

Zephyr 官方把 SEGGER RTT 放在独立的 west 模块 `modules/debug/segger` 里，
而这个工程所在的 west 工作区（`~/zephyrproject`）只拉取了 `hal/stm32`、`cmsis`
等少数模块，**没有** `segger` 模块，所以原来的工程里根本不存在
`CONFIG_USE_SEGGER_RTT` 这类符号。

为了不动系统里的 Zephyr 仓库、也不用联网 `west update`，这里把 SEGGER RTT
源码直接放进工程，做成一个名为 `segger` 的本地模块（在根 `CMakeLists.txt`
里通过 `ZEPHYR_EXTRA_MODULES` 引入）。因为 `zephyr/module.yml` 里写了
`cmake-ext: true` / `kconfig-ext: true`，Zephyr 会用它仓库里现成的胶水代码
来编译本目录，所以：

* `CONFIG_USE_SEGGER_RTT`、`CONFIG_LOG_BACKEND_RTT`、`CONFIG_RTT_CONSOLE`
  等**标准 Kconfig 全部可用**，行为与官方模块一致；
* 业务代码可以直接用 `<SEGGER_RTT.h>` 的 `SEGGER_RTT_printf()` 等 API；
* `LOG_INF/WRN/ERR` 同时输出到 UART 和 RTT（见 `prj.conf`）。

## 文件来源

* `SEGGER/SEGGER_RTT.c`、`SEGGER/SEGGER_RTT.h`、`SEGGER/SEGGER_RTT_printf.c`
  —— SEGGER RTT 7.58c 原版源码，**未做任何修改**，保留 SEGGER 原始版权声明；
* `Config/SEGGER_RTT_Conf.h` —— 由 SEGGER 默认配置改写成 Zephyr 版：
  缓冲区大小、工作模式、链接段、加锁方式全部来自 Kconfig。

SEGGER RTT 的授权是 BSD 风格（保留版权声明即可自由使用、修改、再分发），
原始声明保留在各源文件头部。

## 以后想换回官方模块

如果哪天执行了 `west update`（或手动把
<https://github.com/zephyrproject-rtos/segger> clone 到 `modules/debug/segger`），
就会出现**两个同名模块 `segger`**，CMake 会直接报错。此时只要：

1. 删掉本目录 `modules/segger/`；
2. 去掉根 `CMakeLists.txt` 里对应的 `ZEPHYR_EXTRA_MODULES` 一行。

`prj.conf` / `Kconfig` 里的配置**不用改**（Kconfig 符号名完全一致）。

## 用法

见 `docs/segger_rtt.md`。
