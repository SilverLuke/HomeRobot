#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include "communication/wifi_manager.h"
#include "communication/zephyr_net_client.h"
#include "communication/protobuf_handler.h"
#include "actuator/motor.h"
#include "sensors/lidar.h"
#include "secrets.h"

LOG_MODULE_REGISTER(robot_app, LOG_LEVEL_DBG);

/* PWM DT Specs */
static const struct pwm_dt_spec motor_sx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_fwd_pwm));
static const struct pwm_dt_spec motor_sx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_bwd_pwm));
static const struct pwm_dt_spec motor_dx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_fwd_pwm));
static const struct pwm_dt_spec motor_dx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_bwd_pwm));
static const struct pwm_dt_spec lidar_motor_pwm = PWM_DT_SPEC_GET(DT_ALIAS(lidar_pwm));

int main(void)
{
	LOG_INF("Robot application started on Zephyr!");

	WifiManager& wifi = WifiManager::instance();
	if (!wifi.connect(wifi_ssid, wifi_password)) {
		LOG_ERR("Failed to start Wi-Fi connection");
	}

	LOG_INF("Waiting for Wi-Fi connection...");
	if (!wifi.wait_for_connection(K_SECONDS(30))) {
		LOG_ERR("Wi-Fi connection timeout");
	} else {
		LOG_INF("Wi-Fi connected!");
	}

	ZephyrNetClient client;
	ProtobufHandler proto_handler(client);
	bool server_connected = false;

	const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu));
	if (!device_is_ready(imu)) {
		LOG_ERR("IMU device not ready!");
	}

	/* PCNT device (the peripheral) */
	const struct device *const pcnt = DEVICE_DT_GET(DT_INST(0, espressif_esp32_pcnt));

	Motor motor_sx("SX", &motor_sx_fwd, &motor_sx_bwd, pcnt, 0); // Unit 0
	Motor motor_dx("DX", &motor_dx_fwd, &motor_dx_bwd, pcnt, 1); // Unit 1

	motor_sx.init(1.0, 0.0, 0.0);
	motor_dx.init(1.0, 0.0, 0.0);

	const struct device *const lidar_uart = DEVICE_DT_GET(DT_ALIAS(lidar_uart));
	Lidar lidar(lidar_uart, &lidar_motor_pwm);
	lidar.init();
	lidar.start();

	while (1) {
		if (wifi.is_connected()) {
			if (!client.connected()) {
				LOG_INF("Attempting to connect to server %s:%d...", wifi_server_host, wifi_server_port);
				if (client.connect(wifi_server_host, wifi_server_port)) {
					server_connected = true;
				} else {
					k_msleep(5000);
				}
			} else {
				// We are connected to server, send some dummy telemetry or read sensors
				struct sensor_value acc[3];
				struct sensor_value gyro[3];

				if (sensor_sample_fetch(imu) == 0) {
					sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc);
					sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);

					LOG_DBG("Accel: X=%f Y=%f Z=%f", 
							sensor_value_to_double(&acc[0]),
							sensor_value_to_double(&acc[1]),
							sensor_value_to_double(&acc[2]));
					
					proto_handler.send_imu_data(
						k_uptime_get_32(),
						(float)sensor_value_to_double(&acc[0]),
						(float)sensor_value_to_double(&acc[1]),
						(float)sensor_value_to_double(&acc[2]),
						(float)sensor_value_to_double(&gyro[0]),
						(float)sensor_value_to_double(&gyro[1]),
						(float)sensor_value_to_double(&gyro[2])
					);
				} else {
					// Even if IMU fails, send a heartbeat
					proto_handler.send_heartbeat(k_uptime_get_32());
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
		}
		
		k_msleep(10); // 100Hz loop
	}

	return 0;
}
