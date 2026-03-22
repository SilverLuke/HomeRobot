#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
// #include <zephyr/mgmt/mcumgr/transport/smp_udp.h>

#include "communication/wifi_manager.h"
#include "communication/zephyr_net_client.h"
#include "communication/protobuf_handler.h"
#include "actuator/motor.h"
#include "actuator/status_led.h"
#include "sensors/lidar.h"
#include "sensors/imu.h"
#include "sensors/battery.h"
#include "sensors/encoders.h"
#include "secrets.h"
#include "constants.h"

using namespace constants;

LOG_MODULE_REGISTER(robot_app, LOG_LEVEL_DBG);

/* PWM DT Specs */
static const struct pwm_dt_spec motor_sx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_fwd_pwm));
static const struct pwm_dt_spec motor_sx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_bwd_pwm));
static const struct pwm_dt_spec motor_dx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_fwd_pwm));
static const struct pwm_dt_spec motor_dx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_bwd_pwm));
static const struct pwm_dt_spec lidar_motor_pwm = PWM_DT_SPEC_GET(DT_ALIAS(lidar_pwm));

int main(void)
{
    printk("\n\n*** MAIN START ***\n\n");
    StatusLed status_led;
    status_led.init();
    status_led.set_status(RobotStatus::NO_WIFI);

    printk("EARLY BOOT: Entering main\n");
    k_msleep(2000);
	printk("Robot application started on Zephyr!\n");

	WifiManager& wifi = WifiManager::instance();
	if (!wifi.connect(wifi_ssid, wifi_password)) {
		printk("Failed to start Wi-Fi connection\n");
	}

	printk("Waiting for Wi-Fi connection...\n");
	if (!wifi.wait_for_connection(K_MSEC(WIFI_CONNECT_TIMEOUT_MS))) {
		printk("Wi-Fi connection timeout\n");
	} else {
		printk("Wi-Fi connected!\n");
        status_led.set_status(RobotStatus::WIFI_ONLY);
		/* Start MCUmgr SMP UDP service for OTA */
		/*
		int err = smp_udp_open();
		if (err) {
			LOG_ERR("Failed to start MCUmgr SMP UDP service (err %d)", err);
		} else {
			LOG_INF("MCUmgr SMP UDP service started!");
		}
		*/
	}

	ZephyrNetClient client;
	ProtobufHandler proto_handler(client);
	bool server_connected = false;

	const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));
	Imu imu(imu_dev);
	if (!imu.init()) {
		LOG_ERR("IMU initialization failed!");
	}

	const struct device *const adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
	Battery battery(adc_dev, 2); // Channel 2 from overlay
	if (!battery.init()) {
		LOG_ERR("Battery initialization failed!");
	}

	/* PCNT devices for each encoder unit */
	const struct device *const pcnt_sx = DEVICE_DT_GET(DT_ALIAS(encoder_sx));
	const struct device *const pcnt_dx = DEVICE_DT_GET(DT_ALIAS(encoder_dx));

	Encoders encoder_sx(pcnt_sx, 0);
	Encoders encoder_dx(pcnt_dx, 1);

	Motor motor_sx("SX", &motor_sx_fwd, &motor_sx_bwd, &encoder_sx);
	Motor motor_dx("DX", &motor_dx_fwd, &motor_dx_bwd, &encoder_dx);

	motor_sx.init(1.0, 0.0, 0.0);
	motor_dx.init(1.0, 0.0, 0.0);

	const struct device *const lidar_uart = DEVICE_DT_GET(DT_ALIAS(lidar_uart));
	Lidar lidar(lidar_uart, &lidar_motor_pwm);
	lidar.init();
	lidar.start();

	uint32_t last_slow_telemetry = 0;
    uint32_t last_battery_telemetry = 0;
    uint32_t last_heartbeat = 0;

	while (1) {
        status_led.update();

		if (wifi.is_connected()) {
			if (!client.connected()) {
                status_led.set_status(RobotStatus::WIFI_ONLY);
				LOG_INF("Attempting to connect to server %s:%d...", wifi_server_host, wifi_server_port);
				if (client.connect(wifi_server_host, wifi_server_port)) {
					server_connected = true;
                    status_led.set_status(RobotStatus::CONNECTED);
				} else {
					k_msleep(SERVER_RECONNECT_INTERVAL_MS);
				}
			} else {
				uint32_t now = k_uptime_get_32();

				// IMU Telemetry (High frequency)
				if (imu.update()) {
					float ax, ay, az, gx, gy, gz;
					imu.get_accel(ax, ay, az);
					imu.get_gyro(gx, gy, gz);

					proto_handler.send_imu_data(now, ax, ay, az, gx, gy, gz);
				}

				// Encoder telemetry
				if (now - last_slow_telemetry >= ENCODER_TELEMETRY_INTERVAL_MS) {
					proto_handler.send_encoders_data(now, encoder_sx.get_ticks(), encoder_dx.get_ticks());
					last_slow_telemetry = now;
				}

				// Battery telemetry
				if (now - last_battery_telemetry >= BATTERY_TELEMETRY_INTERVAL_MS) {
					proto_handler.send_battery_status(now, battery.get_percentage(), 
													  battery.get_voltage_mv(), battery.read_raw());
					last_battery_telemetry = now;
				}

                // Heartbeat
                if (now - last_heartbeat >= HEARTBEAT_INTERVAL_MS) {
                    proto_handler.send_heartbeat(now);
                    last_heartbeat = now;
                }

				lidar.loop(proto_handler);
				motor_sx.loop();
				motor_dx.loop();

				/* Process incoming commands */
				homerobot_ServerToRobotMessage rx_msg;
				if (proto_handler.receive_and_decode(rx_msg)) {
					LOG_INF("Received command from server, type: %d", rx_msg.which_payload);
					switch (rx_msg.which_payload) {
						case homerobot_ServerToRobotMessage_motor_move_tag:
							motor_sx.set_motor(rx_msg.payload.motor_move.left_power > 0 ? FORWARD : BRAKE, 
											   (uint8_t)rx_msg.payload.motor_move.left_power);
							motor_dx.set_motor(rx_msg.payload.motor_move.right_power > 0 ? FORWARD : BRAKE, 
											   (uint8_t)rx_msg.payload.motor_move.right_power);
							// Note: Simplified logic, should handle direction from angle or power sign
							break;
						case homerobot_ServerToRobotMessage_motor_config_tag:
							LOG_INF("Applying new PID configuration");
							if (rx_msg.payload.motor_config.has_left_motor) {
								motor_sx.config_set_pid(rx_msg.payload.motor_config.left_motor.kp,
														rx_msg.payload.motor_config.left_motor.ki,
														rx_msg.payload.motor_config.left_motor.kd);
								if (rx_msg.payload.motor_config.left_motor.max_speed > 0) {
									motor_sx.config_set_limit(50, (uint8_t)rx_msg.payload.motor_config.left_motor.max_speed);
								}
							}
							if (rx_msg.payload.motor_config.has_right_motor) {
								motor_dx.config_set_pid(rx_msg.payload.motor_config.right_motor.kp,
														rx_msg.payload.motor_config.right_motor.ki,
														rx_msg.payload.motor_config.right_motor.kd);
								if (rx_msg.payload.motor_config.right_motor.max_speed > 0) {
									motor_dx.config_set_limit(50, (uint8_t)rx_msg.payload.motor_config.right_motor.max_speed);
								}
							}
							break;
						case homerobot_ServerToRobotMessage_stop_all_tag:
							motor_sx.turn_off();
							motor_dx.turn_off();
							lidar.stop();
							break;
						default:
							break;
					}
				}
			}
		} else {
			LOG_WRN("Wi-Fi disconnected, waiting...");
			server_connected = false;
            status_led.set_status(RobotStatus::NO_WIFI);
		}
		
		k_msleep(MAIN_LOOP_DELAY_MS);
	}

	return 0;
}
