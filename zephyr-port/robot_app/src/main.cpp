#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>

extern "C" {
#include <esp_rom_uart.h>
#include <esp_rom_sys.h>
}

#include "actuator/status_led.h"
#include "actuator/motor.h"
#include "sensors/lidar.h"
#include "sensors/battery.h"
#include "sensors/encoders.h"
#include "constants.h"

using namespace constants;

LOG_MODULE_REGISTER(robot_app, LOG_LEVEL_DBG);

// DT Specs
static const struct device *const lidar_uart_dev = DEVICE_DT_GET(DT_ALIAS(lidar_uart));
static const struct gpio_dt_spec lidar_en_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(lidar_en), gpios);
static const struct device *const adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));

// PWM DT Specs for Motors
static const struct pwm_dt_spec motor_sx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_fwd_pwm));
static const struct pwm_dt_spec motor_sx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_bwd_pwm));
static const struct pwm_dt_spec motor_dx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_fwd_pwm));
static const struct pwm_dt_spec motor_dx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_bwd_pwm));

// PCNT Devices for Encoders
static const struct device *const encoder_dev = DEVICE_DT_GET(DT_ALIAS(encoder_sx));

extern "C" int main(void)
{
    k_msleep(1000); // Give serial bridge time
    esp_rom_printf("\n\n!!! KERNEL REACHED MAIN !!!\n");
    esp_rom_printf("Standalone Independent Motor Test (Fixed Encoders)\n");

    StatusLed statusLed;
    Battery battery(adc_dev, 2); 
    Lidar lidar(lidar_uart_dev, &lidar_en_gpio);
    
    // SX is Unit 0, DX is Unit 1
    Encoders encSx(encoder_dev, 0);
    Encoders encDx(encoder_dev, 1);
    
    Motor motorSx("SX", &motor_sx_fwd, &motor_sx_bwd, &encSx);
    Motor motorDx("DX", &motor_dx_fwd, &motor_dx_bwd, &encDx);

    statusLed.init();
    battery.init();
    lidar.init();
    encSx.init();
    encDx.init();
    motorSx.init(1.0, 0.01, 0.1);
    motorDx.init(1.0, 0.01, 0.1);

    statusLed.set_status(RobotStatus::WIFI_ONLY);
    
    // Independent Motor Test Sequence
    esp_rom_printf("Starting Independent Motor Test...\n");
    uint64_t start_ms = k_uptime_get();
    while (k_uptime_get() - start_ms < 6000) {
        uint64_t elapsed = k_uptime_get() - start_ms;

        if (elapsed < 2000) {
            if (elapsed % 500 < 100) esp_rom_printf("Moving SX ONLY...\n");
            motorSx.set_motor(FORWARD, 100);
            motorDx.set_motor(BRAKE, 0);
        } else if (elapsed < 4000) {
            if (elapsed % 500 < 100) esp_rom_printf("Moving DX ONLY...\n");
            motorSx.set_motor(BRAKE, 0);
            motorDx.set_motor(FORWARD, 100);
        } else {
            motorSx.set_motor(BRAKE, 0);
            motorDx.set_motor(BRAKE, 0);
        }

        int32_t sx = encSx.get_total_ticks();
        int32_t dx = encDx.get_total_ticks();
        if (elapsed % 500 < 100) esp_rom_printf("SX: %d | DX: %d\n", sx, dx);
        k_msleep(100);
    }

    while (1) {
        uint32_t v_mv = battery.get_voltage_mv();
        int32_t sx = encSx.get_total_ticks();
        int32_t dx = encDx.get_total_ticks();
        
        static uint32_t last_print = 0;
        if (k_uptime_get_32() - last_print >= 1000) {
            esp_rom_printf("BAT: %u mV | SX: %d | DX: %d | Standalone\n", v_mv, sx, dx);
            last_print = k_uptime_get_32();
        }

        lidar.loop();
        statusLed.update();
        k_msleep(100);
    }

	return 0;
}
