#include "pc_to.hpp"

#pragma message "Compiling Topic/PC_To"

// 状态通道：无订阅者（消费者用 zbus_chan_read 轮询，避免多线程共用 subscriber 抢通知）
ZBUS_CHAN_DEFINE(pub_pc_to,
                 topic::pc_to::Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT({}));