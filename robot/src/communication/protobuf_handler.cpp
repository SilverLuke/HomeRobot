#include "protobuf_handler.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(protobuf_handler, LOG_LEVEL_INF);

ProtobufHandler::ProtobufHandler(NetClient& client) 
    : client_(client), rx_idx_(0), expected_size_(0), reading_header_(true) {}

bool ProtobufHandler::send_imu_data(uint32_t millis, float acc_x, float acc_y, float acc_z, 
                                 float gyro_x, float gyro_y, float gyro_z) {
    homerobot_RobotToServerMessage message = homerobot_RobotToServerMessage_init_default;
    message.sequence_millis = millis;
    message.which_payload = homerobot_RobotToServerMessage_imu_tag;
    message.payload.imu.has_acceleration = true;
    message.payload.imu.acceleration.x = acc_x;
    message.payload.imu.acceleration.y = acc_y;
    message.payload.imu.acceleration.z = acc_z;
    message.payload.imu.has_gyroscope = true;
    message.payload.imu.gyroscope.x = gyro_x;
    message.payload.imu.gyroscope.y = gyro_y;
    message.payload.imu.gyroscope.z = gyro_z;

    return encode_and_send(message);
}

bool ProtobufHandler::send_battery_status(uint32_t millis, uint32_t percentage, uint32_t voltage_mv, uint32_t raw_value) {
    homerobot_RobotToServerMessage message = homerobot_RobotToServerMessage_init_default;
    message.sequence_millis = millis;
    message.which_payload = homerobot_RobotToServerMessage_battery_tag;
    message.payload.battery.percentage = percentage;
    message.payload.battery.voltage_mv = voltage_mv;
    message.payload.battery.raw_value = raw_value;

    return encode_and_send(message);
}

bool ProtobufHandler::send_encoders_data(uint32_t millis, int32_t left, int32_t right) {
    homerobot_RobotToServerMessage message = homerobot_RobotToServerMessage_init_default;
    message.sequence_millis = millis;
    message.which_payload = homerobot_RobotToServerMessage_encoders_tag;
    message.payload.encoders.left_encoder = left;
    message.payload.encoders.right_encoder = right;

    return encode_and_send(message);
}

bool ProtobufHandler::send_robot_config(uint32_t millis, float kp_l, float ki_l, float kd_l, float kp_r, float ki_r, float kd_r) {
    homerobot_RobotToServerMessage message = homerobot_RobotToServerMessage_init_default;
    message.sequence_millis = millis;
    message.which_payload = homerobot_RobotToServerMessage_config_tag;
    
    message.payload.config.has_left_motor = true;
    message.payload.config.left_motor.kp = kp_l;
    message.payload.config.left_motor.ki = ki_l;
    message.payload.config.left_motor.kd = kd_l;
    message.payload.config.left_motor.max_speed = 255;

    message.payload.config.has_right_motor = true;
    message.payload.config.right_motor.kp = kp_r;
    message.payload.config.right_motor.ki = ki_r;
    message.payload.config.right_motor.kd = kd_r;
    message.payload.config.right_motor.max_speed = 255;

    message.payload.config.lidar_frequency = 5.0;

    return encode_and_send(message);
}

bool ProtobufHandler::send_heartbeat(uint32_t millis) {
    homerobot_RobotToServerMessage message = homerobot_RobotToServerMessage_init_default;
    message.sequence_millis = millis;
    message.which_payload = homerobot_RobotToServerMessage_heartbeat_tag;
    message.payload.heartbeat = true;

    return encode_and_send(message);
}

struct LidarScanContext {
    const ProtobufHandler::LidarPointData* points;
    size_t count;
};

static bool encode_lidar_scan_points(pb_ostream_t *stream, const pb_field_t *field, void * const *arg) {
    LidarScanContext* ctx = (LidarScanContext*)*arg;

    for (size_t i = 0; i < ctx->count; i++) {
        if (!pb_encode_tag_for_field(stream, field)) return false;

        homerobot_LidarPoint point = homerobot_LidarPoint_init_default;
        point.distance_mm = ctx->points[i].distance_mm;
        point.angle_deg = ctx->points[i].angle_deg;
        point.quality = ctx->points[i].quality;
        point.scan_completed = ctx->points[i].scan_completed;

        if (!pb_encode_submessage(stream, homerobot_LidarPoint_fields, &point)) return false;
    }

    return true;
}

bool ProtobufHandler::send_lidar_scan(uint32_t millis, const LidarPointData* points, size_t count) {
    homerobot_RobotToServerMessage message = homerobot_RobotToServerMessage_init_default;
    message.sequence_millis = millis;
    message.which_payload = homerobot_RobotToServerMessage_lidar_tag;
    
    LidarScanContext ctx = { points, count };
    message.payload.lidar.points.funcs.encode = encode_lidar_scan_points;
    message.payload.lidar.points.arg = &ctx;

    return encode_and_send(message);
}

bool ProtobufHandler::send_rpc_response(uint32_t millis, uint32_t call_id, const uint8_t* payload, size_t payload_len, const char* error) {
    homerobot_RobotToServerMessage message = homerobot_RobotToServerMessage_init_default;
    message.sequence_millis = millis;
    message.which_payload = homerobot_RobotToServerMessage_rpc_response_tag;
    
    message.payload.rpc_response.call_id = call_id;
    
    if (payload && payload_len > 0) {
        message.payload.rpc_response.payload.size = payload_len;
        memcpy(message.payload.rpc_response.payload.bytes, payload, payload_len);
    }
    
    if (error) {
        strncpy(message.payload.rpc_response.error, error, sizeof(message.payload.rpc_response.error) - 1);
        message.payload.rpc_response.error[sizeof(message.payload.rpc_response.error) - 1] = '\0';
    }
    
    return encode_and_send(message);
}

bool ProtobufHandler::receive_and_decode(homerobot_ServerToRobotMessage& message) {
    if (reading_header_) {
        uint8_t header_byte;
        while (client_.read(&header_byte, 1) == 1) {
            rx_buffer_[rx_idx_++] = header_byte;
            if (rx_idx_ == 2) {
                expected_size_ = (uint16_t)((rx_buffer_[0] << 8) | rx_buffer_[1]);
                if (expected_size_ > sizeof(rx_buffer_)) {
                    LOG_ERR( "Incoming message too large: %u", expected_size_);
                    rx_idx_ = 0;
                    return false;
                }
                reading_header_ = false;
                rx_idx_ = 0;
                break;
            }
        }
    }

    if (!reading_header_) {
        uint8_t data_byte;
        while (client_.read(&data_byte, 1) == 1) {
            rx_buffer_[rx_idx_++] = data_byte;
            if (rx_idx_ == expected_size_) {
                pb_istream_t stream = pb_istream_from_buffer(rx_buffer_, expected_size_);
                bool status = pb_decode(&stream, homerobot_ServerToRobotMessage_fields, &message);
                
                reading_header_ = true;
                rx_idx_ = 0;
                
                if (!status) {
                    LOG_ERR( "Decoding failed: %s", PB_GET_ERROR(&stream));
                    return false;
                }
                return true;
            }
        }
    }

    return false;
}

bool ProtobufHandler::encode_and_send(const homerobot_RobotToServerMessage& message) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer_, sizeof(buffer_));
    if (!pb_encode(&stream, homerobot_RobotToServerMessage_fields, &message)) {
        LOG_ERR( "Encoding failed: %s", PB_GET_ERROR(&stream));
        return false;
    }

    size_t encoded_size = stream.bytes_written;
    LOG_DBG( "Encoded %zu bytes", encoded_size);

    // TODO: Framing. For now, just send it directly.
    // However, without framing, the server cannot know where one message ends and the next begins.
    // I'll add a simple length prefix (2 bytes for size) to match the legacy protocol but simplified.
    // Actually, I'll send it as [size: 2 bytes, data: N bytes]
    
    uint8_t header[2];
    header[0] = (uint8_t)((encoded_size >> 8) & 0xFF);
    header[1] = (uint8_t)(encoded_size & 0xFF);

    if (client_.write(header, 2) != 2) {
        LOG_ERR( "Failed to send header");
        return false;
    }

    if (client_.write(buffer_, encoded_size) != encoded_size) {
        LOG_ERR( "Failed to send message body");
        return false;
    }

    return true;
}
