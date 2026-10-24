#define DT_DRV_COMPAT hpmicro_hpm_ewdg

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <hpm_clock_drv.h>
#include <hpm_ewdg_drv.h>
#include <hpm_plic_drv.h>
#include <hpm_ppor_drv.h>
#include <hpm_soc.h>

LOG_MODULE_REGISTER(wdt_hpmicro, CONFIG_WDT_LOG_LEVEL);

struct wdt_hpmicro_config {
    EWDG_Type *base;
    clock_name_t clock_name;
    uint32_t irq_num;
};

struct wdt_hpmicro_data {
    uint32_t timeout_ms;
    wdt_callback_t callback;
    bool installed;
    bool configured;
};

static int wdt_hpmicro_install_timeout(const struct device *dev,
                                       const struct wdt_timeout_cfg *cfg)
{
    struct wdt_hpmicro_data *data = dev->data;

    if (data->configured) {
        return -EBUSY;
    }
    if (cfg == NULL || cfg->window.max == 0U || cfg->window.min != 0U ||
        cfg->flags != WDT_FLAG_RESET_SOC) {
        return -ENOTSUP;
    }

    data->timeout_ms = cfg->window.max;
    data->callback = cfg->callback;
    data->installed = true;
    return 0;
}

static int wdt_hpmicro_setup(const struct device *dev, uint8_t options)
{
    const struct wdt_hpmicro_config *cfg = dev->config;
    struct wdt_hpmicro_data *data = dev->data;
    const uint32_t unsupported = ~(WDT_OPT_PAUSE_IN_SLEEP |
                                   WDT_OPT_PAUSE_HALTED_BY_DBG);

    if ((options & unsupported) != 0U) {
        return -ENOTSUP;
    }
    if (!data->installed) {
        return -EINVAL;
    }
    if (data->configured) {
        return -EBUSY;
    }
    if (data->callback != NULL &&
        data->timeout_ms <= CONFIG_WDT_HPMICRO_WARNING_MS) {
        return -EINVAL;
    }

    const uint32_t reset_flags = ppor_reset_get_flags(HPM_PPOR);
    ppor_reset_clear_flags(HPM_PPOR, BIT(16));

    ewdg_config_t ewdg_cfg;
    ewdg_get_default_config(cfg->base, &ewdg_cfg);
    ewdg_cfg.enable_watchdog = true;
    ewdg_cfg.int_rst_config.enable_timeout_reset = true;
    ewdg_cfg.int_rst_config.enable_timeout_interrupt =
        data->callback != NULL;
    ewdg_cfg.ctrl_config.use_lowlevel_timeout = false;
    ewdg_cfg.ctrl_config.cnt_clk_sel = ewdg_cnt_clk_src_ext_osc_clk;
    ewdg_cfg.ctrl_config.timeout_reset_us = data->timeout_ms * 1000U;
    if (data->callback != NULL) {
        const uint32_t warning_ms =
            MIN(CONFIG_WDT_HPMICRO_WARNING_MS, data->timeout_ms - 1U);
        ewdg_cfg.ctrl_config.timeout_interrupt_us =
            (data->timeout_ms - warning_ms) * 1000U;
    }
    ewdg_cfg.ctrl_config.keep_running_in_debug_mode =
        (options & WDT_OPT_PAUSE_HALTED_BY_DBG) == 0U;
    ewdg_cfg.ctrl_config.low_power_mode =
        (options & WDT_OPT_PAUSE_IN_SLEEP) != 0U
            ? ewdg_low_power_mode_halt
            : ewdg_low_power_mode_work_clock_normal;
    ewdg_cfg.cnt_src_freq = 32768U;

    if (ewdg_init(cfg->base, &ewdg_cfg) != status_success ||
        ewdg_refresh(cfg->base) != status_success) {
        return -EIO;
    }

    if (data->callback != NULL) {
        irq_enable(cfg->irq_num);
    }
    data->configured = true;
    if ((reset_flags & BIT(16)) != 0U) {
        LOG_WRN("last reset: EWDG0");
    }
    LOG_INF("HPM EWDG ready: timeout=%ums", data->timeout_ms);
    return 0;
}

static int wdt_hpmicro_disable(const struct device *dev)
{
    const struct wdt_hpmicro_config *cfg = dev->config;
    struct wdt_hpmicro_data *data = dev->data;

    if (!data->configured) {
        return -EFAULT;
    }

    irq_disable(cfg->irq_num);
    ewdg_disable(cfg->base);
    data->installed = false;
    data->configured = false;
    return 0;
}

static int wdt_hpmicro_feed(const struct device *dev, int channel_id)
{
    const struct wdt_hpmicro_config *cfg = dev->config;
    const struct wdt_hpmicro_data *data = dev->data;

    if (!data->configured || channel_id != 0) {
        return -EINVAL;
    }
    return ewdg_refresh(cfg->base) == status_success ? 0 : -EIO;
}

static void wdt_hpmicro_isr(const void *arg)
{
    const struct device *dev = arg;
    const struct wdt_hpmicro_config *cfg = dev->config;
    struct wdt_hpmicro_data *data = dev->data;
    const uint32_t status = ewdg_get_status_flags(cfg->base);

    irq_disable(cfg->irq_num);
    if ((status & EWDG_EVENT_TIMEOUT_INTERRUPT) != 0U &&
        data->callback != NULL) {
        data->callback(dev, 0);
    }
    ewdg_clear_status_flags(
        cfg->base, status & ~EWDG_EVENT_TIMEOUT_INTERRUPT);
}

static const struct wdt_driver_api wdt_hpmicro_api = {
    .install_timeout = wdt_hpmicro_install_timeout,
    .setup = wdt_hpmicro_setup,
    .disable = wdt_hpmicro_disable,
    .feed = wdt_hpmicro_feed,
};

#define WDT_HPMICRO_INIT(n)                                                   \
    static const struct wdt_hpmicro_config wdt_hpmicro_config_##n = {         \
        .base = (EWDG_Type *)DT_INST_REG_ADDR(n),                             \
        .clock_name = (clock_name_t)DT_INST_PROP(n, clk_name),                \
        .irq_num = DT_INST_IRQN(n),                                            \
    };                                                                        \
    static struct wdt_hpmicro_data wdt_hpmicro_data_##n;                      \
    static int wdt_hpmicro_init_##n(const struct device *dev)                 \
    {                                                                         \
        const struct wdt_hpmicro_config *cfg = dev->config;                   \
        if (IS_ENABLED(CONFIG_SOC_SERIES_HPM5300)) {                          \
            __plic_enable_irq_edge_trigger(HPM_PLIC_BASE, cfg->irq_num);      \
        }                                                                     \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),               \
                    wdt_hpmicro_isr, DEVICE_DT_INST_GET(n), 0);               \
        clock_add_to_group(cfg->clock_name, 0);                               \
        return 0;                                                             \
    }                                                                         \
    DEVICE_DT_INST_DEFINE(n, wdt_hpmicro_init_##n, NULL,                      \
                          &wdt_hpmicro_data_##n, &wdt_hpmicro_config_##n,     \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,    \
                          &wdt_hpmicro_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_HPMICRO_INIT)
