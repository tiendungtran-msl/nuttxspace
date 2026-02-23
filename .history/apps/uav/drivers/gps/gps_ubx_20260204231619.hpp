/****************************************************************************
 * apps/uav/drivers/gps/gps_ubx.hpp
 *
 * u-blox M10N GPS Driver - UBX Protocol Handler
 *
 * Supports:
 * - UBX binary protocol parsing
 * - NAV-PVT message decoding (primary position/velocity/time message)
 * - Automatic baudrate detection
 * - Configuration via CFG-VALSET (protocol 27+)
 *
 ****************************************************************************/

#ifndef __UAV_DRIVERS_GPS_UBX_HPP
#define __UAV_DRIVERS_GPS_UBX_HPP

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * UBX Protocol Definitions
 ****************************************************************************/

/* Sync bytes */
#define UBX_SYNC1                   0xB5
#define UBX_SYNC2                   0x62

/* Message Classes */
#define UBX_CLASS_NAV               0x01
#define UBX_CLASS_RXM               0x02
#define UBX_CLASS_INF               0x04
#define UBX_CLASS_ACK               0x05
#define UBX_CLASS_CFG               0x06
#define UBX_CLASS_MON               0x0A

/* NAV Message IDs */
#define UBX_ID_NAV_POSLLH           0x02
#define UBX_ID_NAV_STATUS           0x03
#define UBX_ID_NAV_DOP              0x04
#define UBX_ID_NAV_SOL              0x06
#define UBX_ID_NAV_PVT              0x07
#define UBX_ID_NAV_VELNED           0x12
#define UBX_ID_NAV_TIMEUTC          0x21
#define UBX_ID_NAV_SAT              0x35

/* ACK Message IDs */
#define UBX_ID_ACK_NAK              0x00
#define UBX_ID_ACK_ACK              0x01

/* CFG Message IDs */
#define UBX_ID_CFG_PRT              0x00
#define UBX_ID_CFG_MSG              0x01
#define UBX_ID_CFG_RST              0x04
#define UBX_ID_CFG_RATE             0x08
#define UBX_ID_CFG_CFG              0x09
#define UBX_ID_CFG_NAV5             0x24
#define UBX_ID_CFG_VALSET           0x8A
#define UBX_ID_CFG_VALGET           0x8B

/* MON Message IDs */
#define UBX_ID_MON_VER              0x04
#define UBX_ID_MON_HW               0x09

/* Combined message identifiers (class << 8 | id) */
#define UBX_MSG_NAV_PVT             ((UBX_CLASS_NAV << 8) | UBX_ID_NAV_PVT)
#define UBX_MSG_NAV_STATUS          ((UBX_CLASS_NAV << 8) | UBX_ID_NAV_STATUS)
#define UBX_MSG_NAV_DOP             ((UBX_CLASS_NAV << 8) | UBX_ID_NAV_DOP)
#define UBX_MSG_ACK_ACK             ((UBX_CLASS_ACK << 8) | UBX_ID_ACK_ACK)
#define UBX_MSG_ACK_NAK             ((UBX_CLASS_ACK << 8) | UBX_ID_ACK_NAK)
#define UBX_MSG_CFG_VALSET          ((UBX_CLASS_CFG << 8) | UBX_ID_CFG_VALSET)
#define UBX_MSG_CFG_PRT             ((UBX_CLASS_CFG << 8) | UBX_ID_CFG_PRT)
#define UBX_MSG_CFG_MSG             ((UBX_CLASS_CFG << 8) | UBX_ID_CFG_MSG)
#define UBX_MSG_CFG_RATE            ((UBX_CLASS_CFG << 8) | UBX_ID_CFG_RATE)
#define UBX_MSG_MON_VER             ((UBX_CLASS_MON << 8) | UBX_ID_MON_VER)

/* NAV-PVT validity flags */
#define UBX_NAV_PVT_VALID_DATE      0x01
#define UBX_NAV_PVT_VALID_TIME      0x02
#define UBX_NAV_PVT_VALID_FULLY_RESOLVED 0x04

/* NAV-PVT fix flags */
#define UBX_NAV_PVT_FLAGS_GNSS_FIX_OK   0x01
#define UBX_NAV_PVT_FLAGS_DIFF_SOLN     0x02
#define UBX_NAV_PVT_FLAGS_CARR_SOLN     0xC0

/* Fix types */
#define UBX_FIX_TYPE_NO_FIX         0
#define UBX_FIX_TYPE_DEAD_RECKONING 1
#define UBX_FIX_TYPE_2D             2
#define UBX_FIX_TYPE_3D             3
#define UBX_FIX_TYPE_GNSS_DR        4
#define UBX_FIX_TYPE_TIME_ONLY      5

/* Configuration Key IDs for CFG-VALSET (Protocol 27+, M10) */
#define UBX_CFG_KEY_UART1_BAUDRATE      0x40520001
#define UBX_CFG_KEY_UART1INPROT_UBX     0x10730001
#define UBX_CFG_KEY_UART1INPROT_NMEA    0x10730002
#define UBX_CFG_KEY_UART1OUTPROT_UBX    0x10740001
#define UBX_CFG_KEY_UART1OUTPROT_NMEA   0x10740002
#define UBX_CFG_KEY_RATE_MEAS           0x30210001
#define UBX_CFG_KEY_RATE_NAV            0x30210002
#define UBX_CFG_KEY_MSGOUT_NAV_PVT_UART1  0x20910007
#define UBX_CFG_KEY_NAVSPG_DYNMODEL     0x20110021

/* Dynamic models */
#define UBX_DYN_MODEL_PORTABLE      0
#define UBX_DYN_MODEL_STATIONARY    2
#define UBX_DYN_MODEL_PEDESTRIAN    3
#define UBX_DYN_MODEL_AUTOMOTIVE    4
#define UBX_DYN_MODEL_SEA           5
#define UBX_DYN_MODEL_AIRBORNE_1G   6
#define UBX_DYN_MODEL_AIRBORNE_2G   7
#define UBX_DYN_MODEL_AIRBORNE_4G   8

/* CFG-VALSET layers */
#define UBX_CFG_LAYER_RAM           (1 << 0)
#define UBX_CFG_LAYER_BBR           (1 << 1)
#define UBX_CFG_LAYER_FLASH         (1 << 2)

/* Buffer sizes */
#define UBX_RX_BUFFER_SIZE          256
#define UBX_TX_BUFFER_SIZE          128
#define UBX_PAYLOAD_MAX_SIZE        200

/* Timeouts */
#define UBX_ACK_TIMEOUT_MS          500
#define UBX_CONFIG_TIMEOUT_MS       1000

/****************************************************************************
 * UBX Message Structures
 ****************************************************************************/

#pragma pack(push, 1)

/* UBX Header */
struct ubx_header_s
{
    uint8_t  sync1;
    uint8_t  sync2;
    uint8_t  msg_class;
    uint8_t  msg_id;
    uint16_t length;
};

/* UBX Checksum */
struct ubx_checksum_s
{
    uint8_t ck_a;
    uint8_t ck_b;
};

/* NAV-PVT Payload (92 bytes) - Primary position/velocity/time message */
struct ubx_nav_pvt_s
{
    uint32_t iTOW;          /* GPS time of week [ms] */
    uint16_t year;          /* Year (UTC) */
    uint8_t  month;         /* Month 1..12 */
    uint8_t  day;           /* Day 1..31 */
    uint8_t  hour;          /* Hour 0..23 */
    uint8_t  min;           /* Minute 0..59 */
    uint8_t  sec;           /* Second 0..60 */
    uint8_t  valid;         /* Validity flags */
    uint32_t tAcc;          /* Time accuracy [ns] */
    int32_t  nano;          /* Fraction of second [-1e9..1e9] ns */
    uint8_t  fixType;       /* Fix type */
    uint8_t  flags;         /* Fix status flags */
    uint8_t  flags2;        /* Additional flags */
    uint8_t  numSV;         /* Number of satellites */
    int32_t  lon;           /* Longitude [1e-7 deg] */
    int32_t  lat;           /* Latitude [1e-7 deg] */
    int32_t  height;        /* Height above ellipsoid [mm] */
    int32_t  hMSL;          /* Height above mean sea level [mm] */
    uint32_t hAcc;          /* Horizontal accuracy [mm] */
    uint32_t vAcc;          /* Vertical accuracy [mm] */
    int32_t  velN;          /* NED north velocity [mm/s] */
    int32_t  velE;          /* NED east velocity [mm/s] */
    int32_t  velD;          /* NED down velocity [mm/s] */
    int32_t  gSpeed;        /* Ground speed [mm/s] */
    int32_t  headMot;       /* Heading of motion [1e-5 deg] */
    uint32_t sAcc;          /* Speed accuracy [mm/s] */
    uint32_t headAcc;       /* Heading accuracy [1e-5 deg] */
    uint16_t pDOP;          /* Position DOP [0.01] */
    uint16_t flags3;        /* Additional flags */
    uint8_t  reserved1[4];
    int32_t  headVeh;       /* Heading of vehicle [1e-5 deg] */
    int16_t  magDec;        /* Magnetic declination [1e-2 deg] */
    uint16_t magAcc;        /* Magnetic declination accuracy [1e-2 deg] */
};

/* ACK-ACK/NAK Payload */
struct ubx_ack_s
{
    uint8_t clsID;
    uint8_t msgID;
};

/* CFG-PRT Payload (for UART) */
struct ubx_cfg_prt_s
{
    uint8_t  portID;
    uint8_t  reserved1;
    uint16_t txReady;
    uint32_t mode;
    uint32_t baudRate;
    uint16_t inProtoMask;
    uint16_t outProtoMask;
    uint16_t flags;
    uint8_t  reserved2[2];
};

/* CFG-RATE Payload */
struct ubx_cfg_rate_s
{
    uint16_t measRate;      /* Measurement rate [ms] */
    uint16_t navRate;       /* Navigation rate (cycles) */
    uint16_t timeRef;       /* Time system: 0=UTC, 1=GPS */
};

/* CFG-MSG Payload */
struct ubx_cfg_msg_s
{
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;           /* Rate on current port */
};

/* CFG-VALSET Header (Protocol 27+) */
struct ubx_cfg_valset_hdr_s
{
    uint8_t version;        /* 0x00 */
    uint8_t layers;         /* Layers to apply */
    uint8_t reserved[2];
    /* Key-value pairs follow */
};

#pragma pack(pop)

/****************************************************************************
 * GPS Data Structure (parsed from NAV-PVT)
 ****************************************************************************/

struct gps_data_s
{
    uint64_t timestamp_us;   /* System timestamp [us] */

    /* Position */
    double   lat;            /* Latitude [deg] */
    double   lon;            /* Longitude [deg] */
    float    alt_msl;        /* Altitude MSL [m] */
    float    alt_ellipsoid;  /* Altitude above ellipsoid [m] */

    /* Velocity NED */
    float    vel_n;          /* Velocity North [m/s] */
    float    vel_e;          /* Velocity East [m/s] */
    float    vel_d;          /* Velocity Down [m/s] */
    float    ground_speed;   /* Ground speed [m/s] */

    /* Accuracy */
    float    hacc;           /* Horizontal accuracy [m] */
    float    vacc;           /* Vertical accuracy [m] */
    float    sacc;           /* Speed accuracy [m/s] */

    /* Status */
    uint8_t  fix_type;       /* Fix type: 0=no, 2=2D, 3=3D */
    uint8_t  num_sats;       /* Number of satellites */
    float    pdop;           /* Position DOP */

    /* Time */
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    bool     time_valid;

    /* Flags */
    bool     position_valid;
    bool     velocity_valid;
};

/****************************************************************************
 * Decoder State Machine
 ****************************************************************************/

enum ubx_decode_state_e
{
    UBX_DECODE_SYNC1 = 0,
    UBX_DECODE_SYNC2,
    UBX_DECODE_CLASS,
    UBX_DECODE_ID,
    UBX_DECODE_LENGTH1,
    UBX_DECODE_LENGTH2,
    UBX_DECODE_PAYLOAD,
    UBX_DECODE_CHECKSUM1,
    UBX_DECODE_CHECKSUM2
};

/****************************************************************************
 * GPSUbx Class
 ****************************************************************************/

class GPSUbx
{
public:
    GPSUbx();
    ~GPSUbx();

    /**
     * @brief Initialize GPS driver
     * @param uart_path Path to UART device (e.g., "/dev/ttyS3")
     * @return 0 on success, negative on error
     */
    int init(const char *uart_path);

    /**
     * @brief Deinitialize and close UART
     */
    void deinit();

    /**
     * @brief Configure GPS module (baudrate, message rates, dynamic model)
     * @return 0 on success, negative on error
     */
    int configure();

    /**
     * @brief Poll for new GPS data
     * @param timeout_ms Read timeout in milliseconds
     * @return 1 if new data available, 0 if no data, negative on error
     */
    int poll(int timeout_ms);

    /**
     * @brief Get latest GPS data
     * @return Pointer to GPS data structure
     */
    const gps_data_s* getData() const { return &_gps_data; }

    /**
     * @brief Check if GPS is configured and running
     */
    bool isConfigured() const { return _configured; }

    /**
     * @brief Get number of messages received
     */
    uint32_t getMessageCount() const { return _msg_count; }

    /**
     * @brief Get number of parse errors
     */
    uint32_t getErrorCount() const { return _error_count; }

private:
    /* UART */
    int         _uart_fd;
    const char* _uart_path;

    /* State machine */
    ubx_decode_state_e _decode_state;
    uint8_t     _rx_buffer[UBX_RX_BUFFER_SIZE];
    uint8_t     _payload[UBX_PAYLOAD_MAX_SIZE];
    uint16_t    _payload_idx;
    uint16_t    _payload_len;
    uint8_t     _msg_class;
    uint8_t     _msg_id;
    uint8_t     _ck_a;
    uint8_t     _ck_b;

    /* ACK handling */
    volatile bool _ack_received;
    volatile bool _nak_received;
    uint8_t     _ack_class;
    uint8_t     _ack_id;

    /* GPS data */
    gps_data_s  _gps_data;
    bool        _configured;
    uint32_t    _msg_count;
    uint32_t    _error_count;

    /* Private methods */
    int  parseChar(uint8_t c);
    void resetDecoder();
    void addChecksum(uint8_t c);
    int  processMessage();
    int  parseNavPvt(const uint8_t *payload, uint16_t len);
    int  parseAck(const uint8_t *payload, uint16_t len);

    /* Message sending */
    int  sendMessage(uint8_t msg_class, uint8_t msg_id,
                     const uint8_t *payload, uint16_t len);
    void calcChecksum(const uint8_t *data, uint16_t len,
                      uint8_t *ck_a, uint8_t *ck_b);
    int  waitForAck(uint8_t msg_class, uint8_t msg_id, int timeout_ms);

    /* Configuration helpers */
    int  setBaudrate(uint32_t baudrate);
    int  setMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    int  setMeasurementRate(uint16_t rate_ms);
    int  setDynamicModel(uint8_t model);
    int  sendCfgValset(uint32_t key, const void *value, uint8_t size);
};

#endif /* __UAV_DRIVERS_GPS_UBX_HPP */
