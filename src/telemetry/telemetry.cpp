#include "../../Global.h"

#if defined(TELEMETRY_SERIAL)
#include "telemetry.h"
#include "../gyros/gyro.h"
#include "../receiver/receiver.h"
#include "../barometer/barometer.h"
//#include "mavlink/ardupilotmega/mavlink.h"
#include "mavlink/common/mavlink.h"

#ifdef TELEMETRY_SERIAL
    #define teleSerial TELEMETRY_SERIAL
#endif


unsigned long previousMillisHeartbeat = 0;
unsigned heartbeat_interval = 1000;

#ifdef TELEMETRY_SEND_SYS_STATUS
unsigned long previousMillisSysStatus = 0;
unsigned sys_status_interval = 2000;
unsigned long last_execution_millis = 0;
#endif

#ifdef TELEMETRY_SEND_RADIO_STATUS
unsigned long previousMillisRadioStatus = 0;
unsigned radio_status_interval = 2000;
#endif

#ifdef TELEMETRY_SEND_ATTITUDE
unsigned long previousMillisAttitude = 0;
unsigned attitude_interval = 100;
#endif

#ifdef TELEMETRY_SEND_HW_STATUS
unsigned long previousMillisHwStatus = 0;
unsigned hw_status_interval = 2000;
#endif

#ifdef TELEMETRY_SEND_BATTERY_STATUS
unsigned long previousMillisBatStatus = 0;
unsigned bat_status_interval = 4000;
#endif

#ifdef TELEMETRY_SEND_COMPUTER
unsigned long previousMillisMCU = 0;
unsigned mcu_interval = 2000;
#endif

#ifdef TELEMETRY_SEND_RC_CHANNELS
unsigned long previousMillisRCChannels = 0;
unsigned rc_channels_interval = 10;
#endif

#ifdef TELEMETRY_SEND_RC_CHANNELS_SCALED
unsigned long previousMillisRCChannelsScaled = 0;
unsigned rc_channels_scaled_interval = 10;
#endif

#ifdef TELEMETRY_SEND_RAW_IMU
unsigned long previousMillisRawIMU = 0;
unsigned raw_imu_interval = 100;
#endif

#ifdef TELEMETRY_SEND_RAW_PRESSURE
unsigned long previousMillisRawPressure = 0;
unsigned raw_pressure_interval = 100;
#endif

#ifdef TELEMETRY_SEND_ALTITUDE
unsigned long previousMillisAltitude = 0;
unsigned altitude_interval = 100;
#endif

uint8_t system_id = 1;
void telemetry_setup() {
    indicator_do();
    teleSerial.begin(TELEMETRY_LINK_SPEED);
    indicator_done();
}

void telemetry_loop() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_status_t status;
    status.packet_rx_success_count = 0;
    status.packet_rx_drop_count = 0;

    // Receive
    while(teleSerial.available()>0) {
        uint8_t c = teleSerial.read();
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            // Handle message
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
                {
                    // E.g. read GCS heartbeat and go into
                    // comm lost mode if timer times out
                }
                break;

                case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
                {
                    /* Message decoding: PRIMITIVE
                    *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
                    */
                    //mavlink_message_t* msg;
                    mavlink_sys_status_t sys_status;
                    mavlink_msg_sys_status_decode(&msg, &sys_status);
                }
                break;

                case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
                {
                    /* Message decoding: PRIMITIVE
                    *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
                    */
                    //mavlink_message_t* msg;
                    mavlink_param_value_t param_value;
                    mavlink_msg_param_value_decode(&msg, &param_value);
                }
                break;

                // Overrides receiver channels
                case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                {
                    mavlink_rc_channels_override_t rc_channels;
                    mavlink_msg_rc_channels_override_decode(&msg, &rc_channels);
                    channel[1] = rc_channels.chan1_raw;
                    channel[2] = rc_channels.chan2_raw;
                    channel[3] = rc_channels.chan3_raw;
                    channel[4] = rc_channels.chan4_raw;
                    channel[5] = rc_channels.chan5_raw;
                    channel[6] = rc_channels.chan6_raw;
                    channel[7] = rc_channels.chan7_raw;
                    for (uint8_t ch = 1; ch <= CHANNEL_COUNT; ++ch) {
                        if (ch == CHANNEL_THROTTLE) {
                            channel_scaled[ch] = scale_channel(channel[ch], 0, 0, false);
                        } else {
                            channel_scaled[ch] = scale_channel(channel[ch], 0, 0, true);
                        }
                    }
                }
                break;
                
            default:
                break;
            }
        }
    }


    // Send
    unsigned long currentMillisMAVLink = millis();
    if (currentMillisMAVLink - previousMillisHeartbeat >= heartbeat_interval) {
        // Timing variables
        previousMillisHeartbeat = currentMillisMAVLink;

        mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_QUADROTOR;
        heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
        heartbeat.base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        if (state == STATE_ARMED)
            heartbeat.base_mode |= MAV_MODE_MANUAL_ARMED;

        // Heartbeat
        mavlink_msg_heartbeat_encode(system_id, 1, &msg, &heartbeat);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }


#ifdef TELEMETRY_SEND_SYS_STATUS
    if (currentMillisMAVLink - previousMillisSysStatus >= sys_status_interval) {
        previousMillisSysStatus = currentMillisMAVLink;
        mavlink_sys_status_t sys_status;
        //sys_status = (mavlink_sys_status_t *) malloc(sizeof(mavlink_sys_status_t));
        memset(&sys_status, 0, sizeof(mavlink_sys_status_t));
        sys_status.voltage_battery = battery.voltage*1000;
        sys_status.drop_rate_comm = (status.packet_rx_drop_count * 100) / status.packet_rx_success_count;
        sys_status.battery_remaining = -1;
        sys_status.load = currentMillisMAVLink - last_execution_millis;

        mavlink_msg_sys_status_encode(system_id, 1, &msg, &sys_status);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
    last_execution_millis = currentMillisMAVLink;
#endif

#ifdef TELEMETRY_SEND_RADIO_STATUS
    if (currentMillisMAVLink - previousMillisRadioStatus >= radio_status_interval) {
        previousMillisRadioStatus = currentMillisMAVLink;
        mavlink_radio_status_t mavlink_radio_status;
        //sys_status = (mavlink_sys_status_t *) malloc(sizeof(mavlink_sys_status_t));
        memset(&mavlink_radio_status, 0, sizeof(mavlink_radio_status_t));
        mavlink_radio_status.rssi = receiver_rssi;

        mavlink_msg_radio_status_encode(system_id, 1, &msg, &mavlink_radio_status);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
    last_execution_millis = currentMillisMAVLink;
#endif


#ifdef TELEMETRY_SEND_ATTITUDE
    if (currentMillisMAVLink - previousMillisAttitude >= attitude_interval) {
        previousMillisAttitude = currentMillisMAVLink;
        mavlink_attitude_t mavlink_attitude;
        memset(&mavlink_attitude, 0, sizeof(mavlink_attitude));
        mavlink_attitude.time_boot_ms = millis();
        mavlink_attitude.roll = attitude.roll;
        mavlink_attitude.pitch = attitude.pitch;
        mavlink_attitude.yaw = attitude.yaw;
        mavlink_attitude.rollspeed = attitude.rollspeed;
        mavlink_attitude.pitchspeed = attitude.pitchspeed;
        mavlink_attitude.yawspeed = attitude.yawspeed;

        mavlink_msg_attitude_encode(system_id, 1, &msg, &mavlink_attitude);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
#endif

#ifdef TELEMETRY_SEND_HW_STATUS
    if (currentMillisMAVLink - previousMillisHwStatus >= hw_status_interval) {
        previousMillisHwStatus = currentMillisMAVLink;
        mavlink_hwstatus_t *hw_status;
        hw_status = (mavlink_hwstatus_t *) malloc(sizeof(mavlink_hwstatus_t));
        memset(hw_status, 0, sizeof(mavlink_hwstatus_t));
        hw_status->Vcc = 5000;
        hw_status->I2Cerr = 0;

        mavlink_msg_hwstatus_encode(system_id, 1, &msg, hw_status);
        free(hw_status);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
#endif

#ifdef TELEMETRY_SEND_BATTERY_STATUS
    if (currentMillisMAVLink - previousMillisBatStatus >= bat_status_interval) {
        previousMillisBatStatus = currentMillisMAVLink;
        mavlink_battery_status_t *bat_status;
        bat_status = (mavlink_battery_status_t *) malloc(sizeof(mavlink_battery_status_t));
        memset(bat_status, 0, sizeof(mavlink_battery_status_t));
        memset(bat_status->voltages, UINT16_MAX, sizeof(bat_status->voltages));
        bat_status->voltages[0] = BATTERY_MILLI_VOLTS;
        bat_status->current_battery = -1;
        bat_status->current_consumed = -1;
        bat_status->energy_consumed = -1;
        bat_status->battery_remaining = -1;

        mavlink_msg_battery_status_encode(system_id, 1, &msg, bat_status);
        free(bat_status);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
#endif

#ifdef TELEMETRY_SEND_COMPUTER
    if (currentMillisMAVLink - previousMillisMCU >= mcu_interval) {
        previousMillisMCU = currentMillisMAVLink;
        mavlink_onboard_computer_status_t *computer_status;
        computer_status = (mavlink_onboard_computer_status_t *) malloc(sizeof(mavlink_onboard_computer_status_t));
        memset(computer_status, 255, sizeof(mavlink_onboard_computer_status_t));
        computer_status->time_usec = micros();
        computer_status->uptime = millis();
        computer_status->type = 0;

        memset(computer_status->link_tx_max, 0, sizeof(computer_status->link_tx_max));
        computer_status->link_tx_max[5] = TELEMETRY_LINK_SPEED;
        memset(computer_status->link_rx_max, 0, sizeof(computer_status->link_rx_max));
        computer_status->link_rx_max[5] = TELEMETRY_LINK_SPEED;

        mavlink_msg_onboard_computer_status_encode(system_id, 1, &msg, computer_status);
        free(computer_status);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        telSerial.write(buf, len);
    }
#endif


#ifdef TELEMETRY_SEND_RC_CHANNELS
    if (currentMillisMAVLink - previousMillisRCChannels >= rc_channels_interval) {
        previousMillisRCChannels = currentMillisMAVLink;
        mavlink_rc_channels_t rc_channels;
        memset(&rc_channels, 0, sizeof(mavlink_rc_channels_t));
        rc_channels.time_boot_ms = millis();
        rc_channels.chancount = CHANNEL_COUNT;
        rc_channels.chan1_raw = channel[1];
        rc_channels.chan2_raw = channel[2];
        rc_channels.chan3_raw = channel[3];
        rc_channels.chan4_raw = channel[4];
        rc_channels.chan5_raw = channel[5];
        rc_channels.chan6_raw = channel[6];
        rc_channels.chan7_raw = channel[7];
        rc_channels.rssi = receiver_rssi;

        mavlink_msg_rc_channels_encode(system_id, 1, &msg, &rc_channels);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
#endif

#ifdef TELEMETRY_SEND_RC_CHANNELS_SCALED
    if (currentMillisMAVLink - previousMillisRCChannelsScaled >= rc_channels_scaled_interval) {
        previousMillisRCChannelsScaled = currentMillisMAVLink;
        mavlink_rc_channels_scaled_t rc_channels;
        memset(&rc_channels, 0, sizeof(mavlink_rc_channels_scaled_t));
        rc_channels.time_boot_ms = millis();
        rc_channels.chan1_scaled = channel_scaled[1];
        rc_channels.chan2_scaled = channel_scaled[2];
        rc_channels.chan3_scaled = channel_scaled[3];
        rc_channels.chan4_scaled = channel_scaled[4];
        rc_channels.chan5_scaled = channel_scaled[5];
        rc_channels.chan6_scaled = channel_scaled[6];
        rc_channels.chan7_scaled = channel_scaled[7];
        rc_channels.port = 0;
        rc_channels.rssi = receiver_rssi;

        mavlink_msg_rc_channels_scaled_encode(system_id, 1, &msg, &rc_channels);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
#endif


#ifdef TELEMETRY_SEND_RAW_IMU
    if (currentMillisMAVLink - previousMillisRawIMU >= raw_imu_interval) {
        previousMillisRawIMU = currentMillisMAVLink;
        mavlink_raw_imu_t raw_imu;
        memset(&raw_imu, 0, sizeof(raw_imu));
        raw_imu.time_usec = micros();
        raw_imu.xacc = raw_gyro_data.Xaccel;
        raw_imu.yacc = raw_gyro_data.Yaccel;
        raw_imu.zacc = raw_gyro_data.Zaccel;
        raw_imu.xgyro = raw_gyro_data.Xgyro;
        raw_imu.ygyro = raw_gyro_data.Ygyro;
        raw_imu.zgyro = raw_gyro_data.Zgyro;
        raw_imu.temperature = raw_gyro_data.Temp;

        mavlink_msg_raw_imu_encode(system_id, 1, &msg, &raw_imu);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
#endif


#ifdef TELEMETRY_SEND_RAW_PRESSURE
    if (currentMillisMAVLink - previousMillisRawPressure >= raw_pressure_interval) {
        previousMillisRawPressure = currentMillisMAVLink;
        mavlink_raw_pressure_t raw_pressure;
        memset(&raw_pressure, 0, sizeof(raw_pressure));
        raw_pressure.time_usec = micros();
        raw_pressure.press_abs = barometer_data.pressure / 100;
        raw_pressure.press_diff2 = barometer_data.pressure_reference / 100;
        raw_pressure.temperature = barometer_data.temperature;

        mavlink_msg_raw_pressure_encode(system_id, 1, &msg, &raw_pressure);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
#endif

#ifdef TELEMETRY_SEND_ALTITUDE
    if (currentMillisMAVLink - previousMillisAltitude >= altitude_interval) {
        previousMillisAltitude = currentMillisMAVLink;
        mavlink_altitude_t mavlink_altitude;
        memset(&mavlink_altitude, 0, sizeof(mavlink_altitude));
        mavlink_altitude.time_usec = micros();
        mavlink_altitude.altitude_monotonic = altitude.altitude;
        mavlink_altitude.altitude_local = (altitude.altitude + 300);
        mavlink_altitude.altitude_terrain = -1000;
        mavlink_altitude.bottom_clearance = -1;

        mavlink_msg_altitude_encode(system_id, 1, &msg, &mavlink_altitude);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        teleSerial.write(buf, len);
    }
#endif


}

#endif