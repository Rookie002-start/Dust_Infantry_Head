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
- `custom_boards` 已移除。STM32H723VGT6 板卡定义位于
  `zephyr_user/boards/st/stm32h723vgt6/`。

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

## 上下板协议

上下板使用 CAN FD，仲裁段 1 Mbps、数据段 2 Mbps。协议当前以
`inter_cmd/inter_cmd.hpp` 的两份源码分别维护，修改时必须同步到
`Dust_Infantry_Body/inter_cmd/inter_cmd.hpp`。
