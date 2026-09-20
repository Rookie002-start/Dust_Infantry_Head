/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*  SEGGER RTT 的版权与授权声明见 SEGGER_RTT.c / SEGGER_RTT.h
*  （BSD 风格：保留版权声明即可自由使用、修改、再分发）
*
*  本文件是 **Zephyr 版** 的 RTT 配置文件：所有可调参数都来自 Kconfig，
*  不再使用 SEGGER 默认值，这样 prj.conf 里改 CONFIG_SEGGER_RTT_* 即可生效。
*
*  模块位置与构建方式见 modules/segger/README.md
**********************************************************************
File    : SEGGER_RTT_Conf.h
Purpose : Zephyr 版 SEGGER RTT 配置（缓冲区 / 模式 / 段放置 / 加锁）
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#include <zephyr/kernel.h>

/*********************************************************************
*
*      缓冲区数量与大小（Kconfig: CONFIG_SEGGER_RTT_*）
*
**********************************************************************
*/
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS    (CONFIG_SEGGER_RTT_MAX_NUM_UP_BUFFERS)    /* 目标→主机 */
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS  (CONFIG_SEGGER_RTT_MAX_NUM_DOWN_BUFFERS)  /* 主机→目标 */

#define BUFFER_SIZE_UP                   (CONFIG_SEGGER_RTT_BUFFER_SIZE_UP)
#define BUFFER_SIZE_DOWN                 (CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN)

/* SEGGER_RTT_printf() 的临时缓冲：攒够这么多字节再一次性写入上行缓冲 */
#define SEGGER_RTT_PRINTF_BUFFER_SIZE    (CONFIG_SEGGER_RTT_PRINTF_BUFFER_SIZE)

/*
 * 通道 0 的工作模式：
 *   SEGGER_RTT_MODE_NO_BLOCK_SKIP       —— 缓冲满 / 主机未连接时丢弃，绝不阻塞
 *   SEGGER_RTT_MODE_NO_BLOCK_TRIM       —— 能写多少写多少
 *   SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL  —— 阻塞等待主机取走数据
 * 实时控制任务里请用前两者。
 */
#define SEGGER_RTT_MODE_DEFAULT          CONFIG_SEGGER_RTT_MODE

/*
 * 1 = 用字节循环代替 memcpy()：数据量小、调用频繁时开销更低
 *     （默认 0，即用 memcpy）
 */
#ifdef CONFIG_SEGGER_RTT_MEMCPY_USE_BYTELOOP
#define SEGGER_RTT_MEMCPY_USE_BYTELOOP   CONFIG_SEGGER_RTT_MEMCPY_USE_BYTELOOP
#endif

/*********************************************************************
*
*      RTT 控制块 / 缓冲区放到哪个段
*
*      Cortex-M 默认走 CONFIG_SEGGER_RTT_SECTION_CUSTOM：
*      段名取自 CONFIG_SEGGER_RTT_SECTION_CUSTOM_NAME（默认 .rtt_buff_data），
*      由 Zephyr 自带链接片段 modules/segger/segger_rtt.ld 放在 RAM 起始处，
*      主机（J-Link / OpenOCD）可以很快找到 "SEGGER RTT" 控制块。
*
**********************************************************************
*/
#if defined(CONFIG_SEGGER_RTT_SECTION_DTCM)
#define SEGGER_RTT_SECTION               ".n"
#elif defined(CONFIG_SEGGER_RTT_SECTION_CCM)
#define SEGGER_RTT_SECTION               ".n"
#elif defined(CONFIG_SEGGER_RTT_SECTION_CUSTOM)
#define SEGGER_RTT_SECTION               CONFIG_SEGGER_RTT_SECTION_CUSTOM_NAME
#elif defined(CONFIG_SEGGER_RTT_SECTION_CUSTOM_DTS_REGION)
#include <zephyr/devicetree.h>
/* DTS 里别名为 rtt_custom_section 的内存区域，zephyr,memory-region 即段名 */
#define SEGGER_RTT_SECTION               DT_PROP(DT_ALIAS(rtt_custom_section), zephyr_memory_region)
#endif
/* CONFIG_SEGGER_RTT_SECTION_NONE：不指定段，跟随普通 .bss */

/*********************************************************************
*
*      加锁：多线程 / 中断里同时写 RTT 时会互相踩数据
*
*      Zephyr 模块胶水 modules/segger/SEGGER_RTT_zephyr.c 提供了
*      zephyr_rtt_irq_lock()/zephyr_rtt_irq_unlock()，直接关中断实现互斥。
*      （等价于 CONFIG_SEGGER_RTT_CUSTOM_LOCKING，但这里无条件打开，
*        避免只开 RTT_CONSOLE、没开 log backend 时丢失保护）
*
**********************************************************************
*/
unsigned int zephyr_rtt_irq_lock(void);
void zephyr_rtt_irq_unlock(unsigned int key);

#define SEGGER_RTT_LOCK()   {                                                                \
                                unsigned int _zephyr_rtt_lock_key = zephyr_rtt_irq_lock();
#define SEGGER_RTT_UNLOCK()     zephyr_rtt_irq_unlock(_zephyr_rtt_lock_key);                 \
                            }

#endif /* SEGGER_RTT_CONF_H */
