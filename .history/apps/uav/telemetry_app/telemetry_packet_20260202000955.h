/****************************************************************************
 * apps/uav/telemetry_app/telemetry_packet.h
 *
 * TELEMETRY BINARY PACKET - Definition
 *
 * MỤC ĐÍCH:
 * - Định nghĩa cấu trúc packet nhị phân 212 bytes
 * - Dùng chung cho MCU (C) và PC tool (Python decode)
 * - Fixed-size, CRC16 protected
 * - Hỗ trợ 4 IMU riêng lẻ (raw data từ chip)
 *
 * PACKET LAYOUT:
 *   [Header 8B][IMU0 28B][IMU1 28B][IMU2 28B][IMU3 28B][Mag 12B][Baro 8B]
 *   [GPS 24B][Att 32B][Status 12B][Footer 4B]
 *   Total: 212 bytes
 *
 ****************************************************************************/

#ifndef __UAV_TELEMETRY_PACKET_H
#define __UAV_TELEMETRY_PACKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Magic numbers */
#define TELEM_MAGIC_START       0x55AA
#define TELEM_MAGIC_END         0xAA55

/* Number of IMUs */
#define TELEM_NUM_IMUS          4

/* Packet size */
#define TELEM_PACKET_SIZE       212

/* Offsets - updated for 4 IMUs */
#define TELEM_HEADER_OFFSET     0
#define TELEM_IMU0_OFFSET       8
#define TELEM_IMU1_OFFSET       36
#define TELEM_IMU2_OFFSET       64
#define TELEM_IMU3_OFFSET       92
#define TELEM_MAG_OFFSET        120
#define TELEM_BARO_OFFSET       132
#define TELEM_GPS_OFFSET        140
#define TELEM_ATTITUDE_OFFSET   164
#define TELEM_STATUS_OFFSET     196
#define TELEM_FOOTER_OFFSET     208

/* GPS fix types */
#define GPS_FIX_NONE            0
#define GPS_FIX_2D              2
#define GPS_FIX_3D              3

/* Health levels */
#define HEALTH_GOOD             0
#define HEALTH_WARNING          1
#define HEALTH_CRITICAL         2
#define HEALTH_FAILED           3

/* Sensor status flags (bitfield) */
#define SENSOR_IMU_OK           (1 << 0)
#define SENSOR_BARO_OK          (1 << 1)
#define SENSOR_MAG_OK           (1 << 2)
#define SENSOR_GPS_OK           (1 << 3)
#define SENSOR_EKF_OK           (1 << 4)
#define SENSOR_TIMEBASE_OK      (1 << 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Packet header (8 bytes)
 */
struct __attribute__((packed)) telem_header_s
{
    uint16_t magic;             /* 0x55AA */
    uint16_t sequence;          /* Packet sequence number */
    uint32_t timestamp_us;      /* Timestamp in microseconds */
};

/**
 * @brief IMU data (28 bytes)
 */
struct __attribute__((packed)) telem_imu_s
{
    float gyro_x;               /* rad/s */
    float gyro_y;               /* rad/s */
    float gyro_z;               /* rad/s */
    float accel_x;              /* m/s² */
    float accel_y;              /* m/s² */
    float accel_z;              /* m/s² */
    float temperature;          /* °C */
};

/**
 * @brief Magnetometer data (12 bytes)
 */
struct __attribute__((packed)) telem_mag_s
{
    float mag_x;                /* Gauss */
    float mag_y;                /* Gauss */
    float mag_z;                /* Gauss */
};

/**
 * @brief Barometer data (8 bytes)
 */
struct __attribute__((packed)) telem_baro_s
{
    float pressure;             /* Pa */
    float altitude;             /* m */
};

/**
 * @brief GPS data (24 bytes)
 */
struct __attribute__((packed)) telem_gps_s
{
    int32_t  latitude;          /* degrees × 1e7 */
    int32_t  longitude;         /* degrees × 1e7 */
    int32_t  altitude_msl;      /* mm */
    uint32_t ground_speed;      /* cm/s */
    int16_t  heading;           /* degrees × 100 */
    uint8_t  fix_type;          /* 0=none, 2=2D, 3=3D */
    uint8_t  satellites;        /* Number of satellites */
    uint16_t hdop;              /* × 100 */
    uint16_t vdop;              /* × 100 */
};

/**
 * @brief Attitude / EKF output (32 bytes)
 */
struct __attribute__((packed)) telem_attitude_s
{
    float qw;                   /* Quaternion W */
    float qx;                   /* Quaternion X */
    float qy;                   /* Quaternion Y */
    float qz;                   /* Quaternion Z */
    float roll;                 /* rad */
    float pitch;                /* rad */
    float yaw;                  /* rad */
    float innovation_var;       /* EKF innovation variance */
};

/**
 * @brief System status (12 bytes - padded to align)
 */
struct __attribute__((packed)) telem_status_s
{
    uint8_t  health_level;      /* 0=good, 1=warn, 2=crit, 3=fail */
    uint8_t  healthy_imus;      /* Bitmask of healthy IMUs */
    uint8_t  sensor_flags;      /* Sensor status bitfield */
    uint8_t  reserved;
    uint16_t cpu_load;          /* 0-1000 = 0.0-100.0% */
    uint16_t battery_mv;        /* Battery voltage in mV */
    uint32_t loop_count;        /* Telemetry loop counter */
};

/**
 * @brief Packet footer (4 bytes)
 */
struct __attribute__((packed)) telem_footer_s
{
    uint16_t crc16;             /* CRC16-CCITT */
    uint16_t magic;             /* 0xAA55 */
};

/**
 * @brief Complete telemetry packet (128 bytes)
 */
struct __attribute__((packed)) telemetry_packet_s
{
    struct telem_header_s   header;     /*   8 bytes */
    struct telem_imu_s      imu;        /*  28 bytes */
    struct telem_mag_s      mag;        /*  12 bytes */
    struct telem_baro_s     baro;       /*   8 bytes */
    struct telem_gps_s      gps;        /*  24 bytes */
    struct telem_attitude_s attitude;   /*  32 bytes */
    struct telem_status_s   status;     /*  12 bytes */
    struct telem_footer_s   footer;     /*   4 bytes */
};                                      /* Total: 128 bytes */

/* Verify packet size at compile time */
#ifdef __cplusplus
static_assert(sizeof(struct telemetry_packet_s) == TELEM_PACKET_SIZE,
               "Telemetry packet must be 128 bytes");
#else
_Static_assert(sizeof(struct telemetry_packet_s) == TELEM_PACKET_SIZE,
               "Telemetry packet must be 128 bytes");
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief Calculate CRC16-CCITT
 *
 * @param data Pointer to data
 * @param len Data length
 * @return CRC16 value
 */
uint16_t telem_crc16(const uint8_t *data, size_t len);

/**
 * @brief Initialize packet with magic numbers
 *
 * @param pkt Pointer to packet
 */
void telem_packet_init(struct telemetry_packet_s *pkt);

/**
 * @brief Finalize packet (calculate CRC and set end marker)
 *
 * @param pkt Pointer to packet
 * @param seq Sequence number
 * @param timestamp_us Timestamp in microseconds
 */
void telem_packet_finalize(struct telemetry_packet_s *pkt,
                           uint16_t seq,
                           uint32_t timestamp_us);

/**
 * @brief Validate packet CRC
 *
 * @param pkt Pointer to packet
 * @return 0 if valid, -1 if invalid
 */
int telem_packet_validate(const struct telemetry_packet_s *pkt);

#ifdef __cplusplus
}
#endif

#endif /* __UAV_TELEMETRY_PACKET_H */
