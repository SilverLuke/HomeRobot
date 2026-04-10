#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "robot.h"
#include "constants.h"

extern "C" {
#include <esp_rom_sys.h>
}

static Robot robot;

extern "C" int main(void)
{
    esp_rom_printf("\n\n--- HomeRobot Zephyr Entry Point ---\n");
    k_sleep(K_MSEC(500));

    robot.setup();

    while (1) {
        robot.loop();
        k_msleep(constants::MAIN_LOOP_DELAY_MS);
    }

	return 0;
}
