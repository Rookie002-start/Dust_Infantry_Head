# Dust_Infantry_Head

RoboMaster 步兵机器人**上板**固件，基于 Zephyr + Dust 架构。

上板是整机的指令与决策来源：读取遥控器与上位机输入，周期性地通过 CAN FD 把状态帧
发给下板，同时接收下板回传的云台、发射机构实测状态。

## 硬件

- 主用板卡：STM32H723VGT6（`board_dm_mc02`）
- 另支持：HPM5361（RISC-V）、STM32F407（`board_rm_c`）

## 主要业务线程

`imu` 惯性解算 · `remote` 遥控输入 · `pc` 上位机通信 · `booster` 发射控制
`mcu_inter` / `inter_bus` 上下板 CAN FD 收发 · `inter_rx` 接收解析

## 编译与烧录

```bash
source ~/zephyrproject/setup_env.sh
west build -b stm32h723vgt6 -- -DBOARD_CFG=board_dm_mc02
./flash.sh jlink        # 或 dap-cmsis
```

## 说明

与下板通过 CAN FD 通信（仲裁段 1 Mbps / 数据段 2 Mbps），协议定义在
`inter_cmd/inter_cmd.hpp`，必须与 Dust_Infantry_Body 中的同名文件保持一致。
SEGGER RTT 日志与板卡调试说明见 `docs/`。
