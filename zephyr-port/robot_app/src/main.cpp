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
    k_msleep(2000);
    esp_rom_printf("\n\n!!! KERNEL READY - INDEPENDENT MOTOR TEST !!!\n");

    StatusLed statusLed;
    Battery battery(adc_dev, 2); 
    Lidar lidar(lidar_uart_dev, &lidar_en_gpio);
    
    // Original correct mapping: SX=0, DX=1
    Encoders encSx(encoder_dev, 0);
    Encoders encDx(encoder_dev, 1);
    
    Motor motorSx("SX", &motor_sx_fwd, &motor_sx_bwd, &encSx);
    Motor motorDx("DX", &motor_dx_fwd, &motor_dx_bwd, &encDx);

    statusLed.init();
    battery.init();
    encSx.init();
    encDx.init();
    motorSx.init(1.0, 0.0, 0.0);
    motorDx.init(1.0, 0.0, 0.0);

    esp_rom_printf("Moving SX ONLY for 1s...\n");
    motorSx.set_motor(FORWARD, 100);
    k_msleep(1000);
    motorSx.set_motor(BRAKE, 0);
    
    esp_rom_printf("Moving DX ONLY for 1s...\n");
    motorDx.set_motor(FORWARD, 100);
    k_msleep(1000);
    motorDx.set_motor(BRAKE, 0);

    while (1) {
        uint32_t v_mv = battery.get_voltage_mv();
        esp_rom_printf("BAT: %u mV | SX: %d | DX: %d\n", v_mv, encSx.get_total_ticks(), encDx.get_total_ticks());
        statusLed.update();
        k_msleep(1000);
    }

	return 0;
}
