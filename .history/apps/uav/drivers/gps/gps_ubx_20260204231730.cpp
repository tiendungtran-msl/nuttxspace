/****************************************************************************
 * apps/uav/drivers/gps/gps_ubx.cpp
 *
 * u-blox M10N GPS Driver - UBX Protocol Implementation
 *
 * This driver:
 * - Opens UART and configures for GPS communication
 * - Parses UBX binary protocol messages
 * - Decodes NAV-PVT for position, velocity, time
 * - Supports u-blox M10 series (protocol version 27+)
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "gps_ubx.hpp"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPS_DEBUG   0

#if GPS_DEBUG
#define GPS_LOG(fmt, ...) printf("[GPS] " fmt "\n", ##__VA_ARGS__)
#else
#define GPS_LOG(fmt, ...)
#endif

#define GPS_ERR(fmt, ...) printf("[GPS ERR] " fmt "\n", ##__VA_ARGS__)
#define GPS_INFO(fmt, ...) printf("[GPS] " fmt "\n", ##__VA_ARGS__)

/****************************************************************************
 * GPSUbx Implementation
 ****************************************************************************/

GPSUbx::GPSUbx() :
    _uart_fd(-1),
    _uart_path(nullptr),
    _decode_state(UBX_DECODE_SYNC1),
    _payload_idx(0),
    _payload_len(0),
    _msg_class(0),
    _msg_id(0),
    _ck_a(0),
    _ck_b(0),
    _ack_received(false),
    _nak_received(false),
    _ack_class(0),
    _ack_id(0),
    _configured(false),
    _msg_count(0),
    _error_count(0)
{
    memset(&_gps_data, 0, sizeof(_gps_data));
    memset(_rx_buffer, 0, sizeof(_rx_buffer));
    memset(_payload, 0, sizeof(_payload));
}

GPSUbx::~GPSUbx()
{
    deinit();
}

/****************************************************************************
 * init - Open UART and prepare for GPS communication
 ****************************************************************************/

int GPSUbx::init(const char *uart_path)
{
    if (_uart_fd >= 0)
    {
        /* Already initialized */
        return 0;
    }

    _uart_path = uart_path;

    /* Open UART */
    _uart_fd = open(uart_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_uart_fd < 0)
    {
        GPS_ERR("Failed to open %s: %d", uart_path, errno);
        return -errno;
    }

    /* Configure UART: 9600 baud initially (u-blox default) */
    struct termios tio;
    memset(&tio, 0, sizeof(tio));

    if (tcgetattr(_uart_fd, &tio) != 0)
    {
        GPS_ERR("tcgetattr failed: %d", errno);
        close(_uart_fd);
        _uart_fd = -1;
        return -errno;
    }

    /* Set baud rate to 9600 (u-blox M10 default) */
    cfsetispeed(&tio, B9600);
    cfsetospeed(&tio, B9600);

    /* 8N1, no flow control */
    tio.c_cflag &= ~PARENB;     /* No parity */
    tio.c_cflag &= ~CSTOPB;     /* 1 stop bit */
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;         /* 8 data bits */
    tio.c_cflag |= CLOCAL;      /* Ignore modem control lines */
    tio.c_cflag |= CREAD;       /* Enable receiver */
    tio.c_cflag &= ~CRTSCTS;    /* No hardware flow control */

    /* Raw input/output */
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tio.c_oflag &= ~OPOST;

    /* Non-blocking read */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(_uart_fd, TCSANOW, &tio) != 0)
    {
        GPS_ERR("tcsetattr failed: %d", errno);
        close(_uart_fd);
        _uart_fd = -1;
        return -errno;
    }

    /* Flush any pending data */
    tcflush(_uart_fd, TCIOFLUSH);

    GPS_INFO("Opened %s at 9600 baud", uart_path);

    resetDecoder();

    return 0;
}

/****************************************************************************
 * deinit - Close UART
 ****************************************************************************/

void GPSUbx::deinit()
{
    if (_uart_fd >= 0)
    {
        close(_uart_fd);
        _uart_fd = -1;
    }
    _configured = false;
}

/****************************************************************************
 * configure - Configure GPS module for optimal operation
 ****************************************************************************/

int GPSUbx::configure()
{
    if (_uart_fd < 0)
    {
        return -ENODEV;
    }

    GPS_INFO("Configuring GPS module...");

    /* Try to auto-detect baudrate and switch to 115200 */
    static const uint32_t baudrates[] = {9600, 38400, 57600, 115200};
    bool found = false;

    for (int i = 0; i < 4; i++)
    {
        /* Set local baudrate */
        struct termios tio;
        tcgetattr(_uart_fd, &tio);

        speed_t speed;
        switch (baudrates[i])
        {
            case 9600:   speed = B9600;   break;
            case 38400:  speed = B38400;  break;
            case 57600:  speed = B57600;  break;
            case 115200: speed = B115200; break;
            default:     speed = B9600;   break;
        }

        cfsetispeed(&tio, speed);
        cfsetospeed(&tio, speed);
        tcsetattr(_uart_fd, TCSANOW, &tio);
        tcflush(_uart_fd, TCIOFLUSH);

        GPS_LOG("Trying baudrate %u...", baudrates[i]);

        /* Wait a bit and try to receive something */
        usleep(100000);  /* 100ms */

        /* Poll for data */
        int ret = poll(50);
        if (ret > 0)
        {
            GPS_INFO("GPS responding at %u baud", baudrates[i]);
            found = true;

            /* If not already at 115200, switch to it */
            if (baudrates[i] != 115200)
            {
                ret = setBaudrate(115200);
                if (ret == 0)
                {
                    /* Update local baudrate */
                    cfsetispeed(&tio, B115200);
                    cfsetospeed(&tio, B115200);
                    tcsetattr(_uart_fd, TCSANOW, &tio);
                    tcflush(_uart_fd, TCIOFLUSH);
                    usleep(100000);
                    GPS_INFO("Switched to 115200 baud");
                }
            }
            break;
        }
    }

    if (!found)
    {
        GPS_ERR("GPS not responding at any baudrate");
        return -ENODEV;
    }

    /* Configure NAV-PVT message output at 5Hz */
    int ret = setMeasurementRate(200);  /* 200ms = 5Hz */
    if (ret < 0)
    {
        GPS_ERR("Failed to set measurement rate");
    }

    ret = setMessageRate(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1);
    if (ret < 0)
    {
        GPS_ERR("Failed to enable NAV-PVT");
    }

    /* Set dynamic model to Airborne <2g for UAV */
    ret = setDynamicModel(UBX_DYN_MODEL_AIRBORNE_2G);
    if (ret < 0)
    {
        GPS_ERR("Failed to set dynamic model");
    }

    _configured = true;
    GPS_INFO("GPS configuration complete");

    return 0;
}

/****************************************************************************
 * poll - Read and parse GPS data
 ****************************************************************************/

int GPSUbx::poll(int timeout_ms)
{
    if (_uart_fd < 0)
    {
        return -ENODEV;
    }

    struct pollfd fds;
    fds.fd = _uart_fd;
    fds.events = POLLIN;

    int ret = ::poll(&fds, 1, timeout_ms);
    if (ret < 0)
    {
        return -errno;
    }

    if (ret == 0)
    {
        return 0;  /* Timeout, no data */
    }

    if (fds.revents & POLLIN)
    {
        /* Read available data */
        ssize_t nread = read(_uart_fd, _rx_buffer, sizeof(_rx_buffer));
        if (nread < 0)
        {
            return -errno;
        }

        int msg_received = 0;

        /* Parse each byte */
        for (ssize_t i = 0; i < nread; i++)
        {
            if (parseChar(_rx_buffer[i]) > 0)
            {
                msg_received = 1;
            }
        }

        return msg_received;
    }

    return 0;
}

/****************************************************************************
 * resetDecoder - Reset UBX state machine
 ****************************************************************************/

void GPSUbx::resetDecoder()
{
    _decode_state = UBX_DECODE_SYNC1;
    _payload_idx = 0;
    _payload_len = 0;
    _ck_a = 0;
    _ck_b = 0;
}

/****************************************************************************
 * addChecksum - Update running checksum with new byte
 ****************************************************************************/

void GPSUbx::addChecksum(uint8_t c)
{
    _ck_a += c;
    _ck_b += _ck_a;
}

/****************************************************************************
 * parseChar - Process one byte through UBX state machine
 ****************************************************************************/

int GPSUbx::parseChar(uint8_t c)
{
    int ret = 0;

    switch (_decode_state)
    {
    case UBX_DECODE_SYNC1:
        if (c == UBX_SYNC1)
        {
            _decode_state = UBX_DECODE_SYNC2;
        }
        break;

    case UBX_DECODE_SYNC2:
        if (c == UBX_SYNC2)
        {
            _decode_state = UBX_DECODE_CLASS;
            _ck_a = 0;
            _ck_b = 0;
        }
        else
        {
            resetDecoder();
        }
        break;

    case UBX_DECODE_CLASS:
        _msg_class = c;
        addChecksum(c);
        _decode_state = UBX_DECODE_ID;
        break;

    case UBX_DECODE_ID:
        _msg_id = c;
        addChecksum(c);
        _decode_state = UBX_DECODE_LENGTH1;
        break;

    case UBX_DECODE_LENGTH1:
        _payload_len = c;
        addChecksum(c);
        _decode_state = UBX_DECODE_LENGTH2;
        break;

    case UBX_DECODE_LENGTH2:
        _payload_len |= (uint16_t)c << 8;
        addChecksum(c);

        if (_payload_len > UBX_PAYLOAD_MAX_SIZE)
        {
            GPS_LOG("Payload too large: %u", _payload_len);
            _error_count++;
            resetDecoder();
        }
        else if (_payload_len == 0)
        {
            _decode_state = UBX_DECODE_CHECKSUM1;
        }
        else
        {
            _payload_idx = 0;
            _decode_state = UBX_DECODE_PAYLOAD;
        }
        break;

    case UBX_DECODE_PAYLOAD:
        _payload[_payload_idx++] = c;
        addChecksum(c);

        if (_payload_idx >= _payload_len)
        {
            _decode_state = UBX_DECODE_CHECKSUM1;
        }
        break;

    case UBX_DECODE_CHECKSUM1:
        if (c == _ck_a)
        {
            _decode_state = UBX_DECODE_CHECKSUM2;
        }
        else
        {
            GPS_LOG("Checksum A mismatch: got 0x%02x, expected 0x%02x", c, _ck_a);
            _error_count++;
            resetDecoder();
        }
        break;

    case UBX_DECODE_CHECKSUM2:
        if (c == _ck_b)
        {
            /* Valid message received */
            ret = processMessage();
            _msg_count++;
        }
        else
        {
            GPS_LOG("Checksum B mismatch: got 0x%02x, expected 0x%02x", c, _ck_b);
            _error_count++;
        }
        resetDecoder();
        break;

    default:
        resetDecoder();
        break;
    }

    return ret;
}

/****************************************************************************
 * processMessage - Handle complete UBX message
 ****************************************************************************/

int GPSUbx::processMessage()
{
    GPS_LOG("MSG: class=0x%02x id=0x%02x len=%u", _msg_class, _msg_id, _payload_len);

    switch (_msg_class)
    {
    case UBX_CLASS_NAV:
        switch (_msg_id)
        {
        case UBX_ID_NAV_PVT:
            return parseNavPvt(_payload, _payload_len);

        default:
            break;
        }
        break;

    case UBX_CLASS_ACK:
        return parseAck(_payload, _payload_len);

    default:
        break;
    }

    return 0;
}

/****************************************************************************
 * parseNavPvt - Parse NAV-PVT message (main position/velocity/time)
 ****************************************************************************/

int GPSUbx::parseNavPvt(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(ubx_nav_pvt_s))
    {
        GPS_LOG("NAV-PVT too short: %u < %zu", len, sizeof(ubx_nav_pvt_s));
        return 0;
    }

    const ubx_nav_pvt_s *pvt = reinterpret_cast<const ubx_nav_pvt_s *>(payload);

    /* Update timestamp */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    _gps_data.timestamp_us = (uint64_t)ts.tv_sec * 1000000ULL +
                             (uint64_t)ts.tv_nsec / 1000ULL;

    /* Position (convert from 1e-7 degrees to degrees) */
    _gps_data.lat = pvt->lat * 1e-7;
    _gps_data.lon = pvt->lon * 1e-7;
    _gps_data.alt_msl = pvt->hMSL * 1e-3f;          /* mm to m */
    _gps_data.alt_ellipsoid = pvt->height * 1e-3f;  /* mm to m */

    /* Velocity NED (convert from mm/s to m/s) */
    _gps_data.vel_n = pvt->velN * 1e-3f;
    _gps_data.vel_e = pvt->velE * 1e-3f;
    _gps_data.vel_d = pvt->velD * 1e-3f;
    _gps_data.ground_speed = pvt->gSpeed * 1e-3f;

    /* Accuracy (convert from mm to m, mm/s to m/s) */
    _gps_data.hacc = pvt->hAcc * 1e-3f;
    _gps_data.vacc = pvt->vAcc * 1e-3f;
    _gps_data.sacc = pvt->sAcc * 1e-3f;

    /* Fix status */
    _gps_data.fix_type = pvt->fixType;
    _gps_data.num_sats = pvt->numSV;
    _gps_data.pdop = pvt->pDOP * 0.01f;

    /* Time */
    _gps_data.year = pvt->year;
    _gps_data.month = pvt->month;
    _gps_data.day = pvt->day;
    _gps_data.hour = pvt->hour;
    _gps_data.minute = pvt->min;
    _gps_data.second = pvt->sec;
    _gps_data.time_valid = (pvt->valid & UBX_NAV_PVT_VALID_DATE) &&
                           (pvt->valid & UBX_NAV_PVT_VALID_TIME);

    /* Validity flags */
    _gps_data.position_valid = (pvt->flags & UBX_NAV_PVT_FLAGS_GNSS_FIX_OK) &&
                               (pvt->fixType >= UBX_FIX_TYPE_2D);
    _gps_data.velocity_valid = _gps_data.position_valid;

    GPS_LOG("NAV-PVT: fix=%u sats=%u lat=%.6f lon=%.6f alt=%.1f",
            pvt->fixType, pvt->numSV, _gps_data.lat, _gps_data.lon, _gps_data.alt_msl);

    return 1;  /* New data available */
}

/****************************************************************************
 * parseAck - Parse ACK-ACK or ACK-NAK message
 ****************************************************************************/

int GPSUbx::parseAck(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(ubx_ack_s))
    {
        return 0;
    }

    const ubx_ack_s *ack = reinterpret_cast<const ubx_ack_s *>(payload);

    _ack_class = ack->clsID;
    _ack_id = ack->msgID;

    if (_msg_id == UBX_ID_ACK_ACK)
    {
        _ack_received = true;
        GPS_LOG("ACK for class=0x%02x id=0x%02x", ack->clsID, ack->msgID);
    }
    else if (_msg_id == UBX_ID_ACK_NAK)
    {
        _nak_received = true;
        GPS_LOG("NAK for class=0x%02x id=0x%02x", ack->clsID, ack->msgID);
    }

    return 0;
}

/****************************************************************************
 * calcChecksum - Calculate UBX checksum
 ****************************************************************************/

void GPSUbx::calcChecksum(const uint8_t *data, uint16_t len,
                          uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

/****************************************************************************
 * sendMessage - Send UBX message
 ****************************************************************************/

int GPSUbx::sendMessage(uint8_t msg_class, uint8_t msg_id,
                        const uint8_t *payload, uint16_t len)
{
    if (_uart_fd < 0)
    {
        return -ENODEV;
    }

    uint8_t header[6];
    header[0] = UBX_SYNC1;
    header[1] = UBX_SYNC2;
    header[2] = msg_class;
    header[3] = msg_id;
    header[4] = len & 0xFF;
    header[5] = (len >> 8) & 0xFF;

    /* Calculate checksum over class, id, length, payload */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++)
    {
        ck_a += header[i];
        ck_b += ck_a;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        ck_a += payload[i];
        ck_b += ck_a;
    }

    /* Send header */
    ssize_t ret = write(_uart_fd, header, 6);
    if (ret != 6)
    {
        return -errno;
    }

    /* Send payload */
    if (len > 0 && payload)
    {
        ret = write(_uart_fd, payload, len);
        if (ret != len)
        {
            return -errno;
        }
    }

    /* Send checksum */
    uint8_t checksum[2] = {ck_a, ck_b};
    ret = write(_uart_fd, checksum, 2);
    if (ret != 2)
    {
        return -errno;
    }

    GPS_LOG("TX: class=0x%02x id=0x%02x len=%u", msg_class, msg_id, len);

    return 0;
}

/****************************************************************************
 * waitForAck - Wait for ACK/NAK response
 ****************************************************************************/

int GPSUbx::waitForAck(uint8_t msg_class, uint8_t msg_id, int timeout_ms)
{
    _ack_received = false;
    _nak_received = false;

    uint64_t start_time;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    start_time = (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;

    while (true)
    {
        /* Check timeout */
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now = (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;

        if (now - start_time > (uint64_t)timeout_ms)
        {
            GPS_LOG("ACK timeout for class=0x%02x id=0x%02x", msg_class, msg_id);
            return -ETIMEDOUT;
        }

        /* Poll for data */
        poll(10);

        /* Check for ACK/NAK */
        if (_ack_received && _ack_class == msg_class && _ack_id == msg_id)
        {
            _ack_received = false;
            return 0;
        }

        if (_nak_received && _ack_class == msg_class && _ack_id == msg_id)
        {
            _nak_received = false;
            return -EPROTO;
        }
    }
}

/****************************************************************************
 * setBaudrate - Configure GPS baudrate using CFG-PRT
 ****************************************************************************/

int GPSUbx::setBaudrate(uint32_t baudrate)
{
    /* Use CFG-PRT message for older protocol compatibility */
    ubx_cfg_prt_s cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.portID = 1;  /* UART1 */
    cfg.mode = 0x000008D0;  /* 8N1 */
    cfg.baudRate = baudrate;
    cfg.inProtoMask = 0x01;   /* UBX only */
    cfg.outProtoMask = 0x01;  /* UBX only */

    int ret = sendMessage(UBX_CLASS_CFG, UBX_ID_CFG_PRT,
                         (uint8_t *)&cfg, sizeof(cfg));
    if (ret < 0)
    {
        return ret;
    }

    /* Don't wait for ACK as we're changing baudrate */
    usleep(50000);  /* 50ms for message to be sent */

    return 0;
}

/****************************************************************************
 * setMessageRate - Configure message output rate using CFG-MSG
 ****************************************************************************/

int GPSUbx::setMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    ubx_cfg_msg_s cfg;
    cfg.msgClass = msg_class;
    cfg.msgID = msg_id;
    cfg.rate = rate;

    int ret = sendMessage(UBX_CLASS_CFG, UBX_ID_CFG_MSG,
                         (uint8_t *)&cfg, sizeof(cfg));
    if (ret < 0)
    {
        return ret;
    }

    return waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_MSG, UBX_ACK_TIMEOUT_MS);
}

/****************************************************************************
 * setMeasurementRate - Configure measurement/navigation rate using CFG-RATE
 ****************************************************************************/

int GPSUbx::setMeasurementRate(uint16_t rate_ms)
{
    ubx_cfg_rate_s cfg;
    cfg.measRate = rate_ms;
    cfg.navRate = 1;
    cfg.timeRef = 0;  /* UTC */

    int ret = sendMessage(UBX_CLASS_CFG, UBX_ID_CFG_RATE,
                         (uint8_t *)&cfg, sizeof(cfg));
    if (ret < 0)
    {
        return ret;
    }

    return waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_RATE, UBX_ACK_TIMEOUT_MS);
}

/****************************************************************************
 * setDynamicModel - Configure navigation dynamic model
 ****************************************************************************/

int GPSUbx::setDynamicModel(uint8_t model)
{
    /* Use CFG-VALSET for M10 (protocol 27+) */
    uint8_t buf[12];
    ubx_cfg_valset_hdr_s *hdr = (ubx_cfg_valset_hdr_s *)buf;

    hdr->version = 0x00;
    hdr->layers = UBX_CFG_LAYER_RAM;
    hdr->reserved[0] = 0;
    hdr->reserved[1] = 0;

    /* Key-Value pair: NAVSPG_DYNMODEL */
    uint32_t key = UBX_CFG_KEY_NAVSPG_DYNMODEL;
    memcpy(&buf[4], &key, 4);
    buf[8] = model;

    int ret = sendMessage(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, buf, 9);
    if (ret < 0)
    {
        /* Try legacy CFG-NAV5 if CFG-VALSET not supported */
        uint8_t nav5[36];
        memset(nav5, 0, sizeof(nav5));
        nav5[0] = 0x05;  /* mask: dynModel */
        nav5[1] = 0x00;
        nav5[2] = model; /* dynModel */
        nav5[3] = 3;     /* fixMode: auto 2D/3D */

        ret = sendMessage(UBX_CLASS_CFG, UBX_ID_CFG_NAV5, nav5, sizeof(nav5));
        if (ret < 0)
        {
            return ret;
        }
        return waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_NAV5, UBX_ACK_TIMEOUT_MS);
    }

    return waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, UBX_ACK_TIMEOUT_MS);
}

/****************************************************************************
 * sendCfgValset - Send CFG-VALSET message with single key-value
 ****************************************************************************/

int GPSUbx::sendCfgValset(uint32_t key, const void *value, uint8_t size)
{
    uint8_t buf[16];
    ubx_cfg_valset_hdr_s *hdr = (ubx_cfg_valset_hdr_s *)buf;

    hdr->version = 0x00;
    hdr->layers = UBX_CFG_LAYER_RAM;
    hdr->reserved[0] = 0;
    hdr->reserved[1] = 0;

    memcpy(&buf[4], &key, 4);
    memcpy(&buf[8], value, size);

    return sendMessage(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, buf, 8 + size);
}
