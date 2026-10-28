#include <zephyr/sys/printk.h>

#include "compatibility_hooks.h"

#if defined(__GNUC__) && (__GNUC__ >= 11)
#define APP_COMPATIBILITY_KEEP __attribute__((used, retain))
#else
#define APP_COMPATIBILITY_KEEP __attribute__((used))
#endif

APP_COMPATIBILITY_KEEP int config_mandatory_printf(void)
{
    return 2026;
}

APP_COMPATIBILITY_KEEP int print_alarm_messages(void)
{
    printk("this is an alarm\r\n");
    return 155;
}

APP_COMPATIBILITY_KEEP void print_academic_integrity_marker(void)
{
    printk("Academic-integrity marker: EP-MUIOT-2026-ORION.\r\n");
}
