#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "robot.h"
#include "constants.h"

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include "bridge/gazebo_bridge.h"
#endif

#if !defined(CONFIG_BOARD_NATIVE_SIM)
extern "C" {
#include <esp_rom_sys.h>
}
#endif

extern "C" int main(void)
{
#if defined(CONFIG_BOARD_NATIVE_SIM)
    printk("\n\n--- HomeRobot Zephyr native_sim Entry Point ---\n");
#else
    esp_rom_printf("\n\n--- HomeRobot Zephyr Entry Point ---\n");
#endif

    k_sleep(K_MSEC(500));

    static Robot robot;
    robot.setup();

    while (1) {
        robot.loop();
        k_msleep(constants::MAIN_LOOP_DELAY_MS);
        k_yield();
    }

	return 0;
}
