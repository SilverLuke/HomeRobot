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
#include "sensors/imu.h"
#include "communication/wifi_manager.h"
#include "communication/zephyr_net_client.h"
#include "communication/protobuf_handler.h"
#include "diagnostic.h"
#include "secrets.h"
#include "constants.h"

using namespace constants;

LOG_MODULE_REGISTER(robot_app, LOG_LEVEL_DBG);

// Legacy PID Constants
#define KP 1.0
#define KI 0.01
#define KD 0.1

// DT Specs
static const struct device *const lidar_uart_dev = DEVICE_DT_GET(DT_ALIAS(lidar_uart));
static const struct gpio_dt_spec lidar_en_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(lidar_en), gpios);
static const struct device *const adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));

// PWM DT Specs for Motors
static const struct pwm_dt_spec motor_sx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_fwd_pwm));
static const struct pwm_dt_spec motor_sx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_bwd_pwm));
static const struct pwm_dt_spec motor_dx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_fwd_pwm));
static const struct pwm_dt_spec motor_dx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_bwd_pwm));

// PCNT Devices for Encoders
static const struct device *const encoder_dev = DEVICE_DT_GET(DT_ALIAS(encoder_sx));

const char* get_payload_name(pb_size_t which) {
    switch(which) {
        case homerobot_ServerToRobotMessage_motor_move_tag: return "MOTOR_MOVE";
        case homerobot_ServerToRobotMessage_motor_config_tag: return "MOTOR_CONFIG";
        case homerobot_ServerToRobotMessage_lidar_control_tag: return "LIDAR_CTRL";
        case homerobot_ServerToRobotMessage_stop_all_tag: return "STOP_ALL";
        case homerobot_ServerToRobotMessage_stop_moving_tag: return "STOP_MOVING";
        case homerobot_ServerToRobotMessage_request_data_tag: return "REQ_DATA";
        case homerobot_ServerToRobotMessage_rpc_request_tag: return "RPC_REQ";
        default: return "UNKNOWN";
    }
}

extern "C" int main(void)
{
    // Fast bootstrap: initialize battery first
    esp_rom_printf("\n\n--- HomeRobot Booting (Fast) ---\n");
    k_sleep(K_MSEC(100)); // Minimal stabilization

    StatusLed statusLed;
    Battery battery(adc_dev, 2); 

    statusLed.init(); // note: statusLed.init() now has 50ms delay
    battery.init();

    // Power Guard: Check if battery is connected (>10V)
    // This prevents BOD resets when running only on USB (5V)
    uint32_t boot_voltage = battery.get_voltage_mv();
    esp_rom_printf("Boot Voltage: %u mV\n", boot_voltage);
    
    if (boot_voltage < 10000) {
        statusLed.set_color(50, 25, 0); // Orange warning
        while (boot_voltage < 10000) {
            esp_rom_printf("!!! LOW POWER: %u mV !!! (Please turn on battery switch)\n", boot_voltage);
            k_sleep(K_MSEC(1000));
            boot_voltage = battery.get_voltage_mv();
        }
        statusLed.set_status(RobotStatus::NO_WIFI);
    }
    esp_rom_printf("Power OK: %u mV. Proceeding with full init...\n", boot_voltage);

    // Initialize the rest of the hardware
    Lidar lidar(lidar_uart_dev, &lidar_en_gpio);
    Imu imu(imu_dev);
    Encoders encSx(encoder_dev, 0);
    Encoders encDx(encoder_dev, 1);
    Motor motorSx("SX", &motor_sx_fwd, &motor_sx_bwd, &encSx);
    Motor motorDx("DX", &motor_dx_fwd, &motor_dx_bwd, &encDx);

    encSx.init();
    encDx.init();
    imu.init();
    lidar.init();
    motorSx.init(KP, KI, KD);
    motorDx.init(KP, KI, KD);

    esp_rom_printf("Peripherals Ready. Starting Wi-Fi...\n");

    WifiManager& wifi = WifiManager::instance();
    ZephyrNetClient netClient;
    ProtobufHandler protoHandler(netClient);
    Diagnostic diagnostic(motorSx, motorDx, imu, battery, statusLed);

    wifi.connect(wifi_ssid, wifi_password);
    statusLed.set_status(RobotStatus::NO_WIFI);

    while (1) {
        if (wifi.is_connected()) {
            if (!netClient.connected()) {
                esp_rom_printf("Wi-Fi OK. Connecting to server: %s:%d\n", wifi_server_host, wifi_server_port);
                if (netClient.connect(wifi_server_host, wifi_server_port)) {
                    esp_rom_printf("[%u] CONNECTED to server at %s:%d\n", k_uptime_get_32(), wifi_server_host, wifi_server_port);
                    statusLed.set_status(RobotStatus::CONNECTED);
                } else {
                    esp_rom_printf("Server connection failed\n");
                    k_sleep(K_MSEC(2000));
                }
            } else {
                homerobot_ServerToRobotMessage rx_msg;
                // Important: receive multiple messages if available
                while (protoHandler.receive_and_decode(rx_msg)) {
                    esp_rom_printf("RX: [%s] (Seq: %u)\n", 
                        get_payload_name(rx_msg.which_payload), rx_msg.sequence_millis);

                    if (rx_msg.which_payload == homerobot_ServerToRobotMessage_motor_move_tag) {
                        int lp = (int)rx_msg.payload.motor_move.left_power;
                        int rp = (int)rx_msg.payload.motor_move.right_power;
                        float la = rx_msg.payload.motor_move.left_angle;
                        float ra = rx_msg.payload.motor_move.right_angle;
                        
                        esp_rom_printf(" -> MOVE: L=%d R=%d\n", lp, rp);
                        
                        if (lp == 0) motorSx.set_motor(BRAKE, 0);
                        else motorSx.set_motor(la >= 0 ? FORWARD : BACKWARD, (uint8_t)lp);
                        
                        if (rp == 0) motorDx.set_motor(BRAKE, 0);
                        else motorDx.set_motor(ra >= 0 ? FORWARD : BACKWARD, (uint8_t)rp);
                    }
                    else if (rx_msg.which_payload == homerobot_ServerToRobotMessage_rpc_request_tag) {
                        esp_rom_printf(" -> RPC: %s\n", rx_msg.payload.rpc_request.method);
                        diagnostic.run_rpc();
                        protoHandler.send_rpc_response(k_uptime_get_32(), rx_msg.payload.rpc_request.call_id, nullptr, 0);
                    }
                    else if (rx_msg.which_payload == homerobot_ServerToRobotMessage_lidar_control_tag) {
                        bool active = rx_msg.payload.lidar_control.active;
                        esp_rom_printf(" -> LIDAR CTRL: %s\n", active ? "START" : "STOP");
                        if (active) lidar.start();
                        else lidar.stop();
                    }
                    else if (rx_msg.which_payload == homerobot_ServerToRobotMessage_stop_all_tag) {
                        esp_rom_printf(" -> STOP ALL\n");
                        motorSx.set_motor(BRAKE, 0);
                        motorDx.set_motor(BRAKE, 0);
                        lidar.stop();
                    }
                    else if (rx_msg.which_payload == homerobot_ServerToRobotMessage_stop_moving_tag) {
                        esp_rom_printf(" -> STOP MOVING\n");
                        motorSx.set_motor(BRAKE, 0);
                        motorDx.set_motor(BRAKE, 0);
                    }
                }
            }
        }

        lidar.loop(&protoHandler);
        statusLed.update();
        
        static uint32_t last_telemetry = 0;
        if (k_uptime_get_32() - last_telemetry >= 5000) {
            uint32_t v_mv = battery.get_voltage_mv();
            esp_rom_printf("STAT: BAT=%u WiFi=%d Net=%d\n", v_mv, wifi.is_connected(), netClient.connected());
            protoHandler.send_battery_status(k_uptime_get_32(), battery.get_percentage(), v_mv, battery.read_raw());
            last_telemetry = k_uptime_get_32();
        }

        k_msleep(MAIN_LOOP_DELAY_MS);
    }
	return 0;
}
