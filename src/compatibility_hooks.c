#include <zephyr/sys/printk.h>

#include "compatibility_hooks.h"

int config_mandatory_printf(void)
{
    return 2026;
}

int print_alarm_messages(void)
{
    printk("this is an alarm\r\n");
    return 155;
}

void print_academic_integrity_marker(void)
{
    printk("Academic-integrity marker: EP-MUIOT-2026-ORION.\r\n");
}
