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
    _gpio_initialized(false),
    _pps_detected(false),
    _pps_timestamp_us(0),
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
    _error_count(0),
    _proto_ver_27_or_higher(false),
    _board_generation(0)
{
    memset(&_gps_data, 0, sizeof(_gps_data));
    memset(_rx_buffer, 0, sizeof(_rx_buffer));
    memset(_payload, 0, sizeof(_payload));
    memset(_gps_data.module_name, 0, sizeof(_gps_data.module_name));
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
 * hardwareReset - Reset GPS module via GPIO (if available)
 * Note: This requires GPIO_GPS_RESET_N to be configured in board.h
 * For now, we send a software reset command via UBX-CFG-RST
 ****************************************************************************/

int GPSUbx::hardwareReset()
{
    if (_uart_fd < 0)
    {
        return -ENODEV;
    }

    /* Send UBX-CFG-RST (software reset) */
    uint8_t rst[4];
    rst[0] = 0x00;  /* navBbrMask LSB - hot start */
    rst[1] = 0x00;  /* navBbrMask MSB */
    rst[2] = 0x02;  /* resetMode: controlled software reset (GNSS only) */
    rst[3] = 0x00;  /* reserved */

    int ret = sendMessage(UBX_CLASS_CFG, UBX_ID_CFG_RST, rst, sizeof(rst));
    if (ret < 0)
    {
        GPS_ERR("Failed to send reset command");
        return ret;
    }

    /* Wait for GPS to restart */
    usleep(500000);  /* 500ms */

    /* Flush UART buffer */
    tcflush(_uart_fd, TCIOFLUSH);

    GPS_INFO("GPS reset complete");
    _configured = false;

    return 0;
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

    /* ------------------------------------------------------------------ *
     * Step 1 – Auto-detect baudrate by reading RAW bytes.                *
     *                                                                     *
     * u-blox M10 factory default = NMEA at 9600.  Previous code only     *
     * checked for parsed UBX frames, so a factory-default (NMEA-only)    *
     * module was invisible.  We now look for ANY incoming bytes and       *
     * classify them as UBX (0xB5 0x62) or NMEA ('$').                    *
     * The window is 1.5 s per baudrate (covers the 1 Hz default rate).   *
     * ------------------------------------------------------------------ */

    static const uint32_t baudrates[] = {9600, 115200, 38400, 57600};
    bool found = false;
    uint32_t found_baud = 0;
    bool detected_ubx  = false;
    bool detected_nmea = false;

    for (int i = 0; i < 4; i++)
    {
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

        GPS_INFO("Probing at %lu baud...", (unsigned long)baudrates[i]);

        uint8_t probe_buf[128];
        bool got_data = false;
        detected_ubx  = false;
        detected_nmea = false;
        int total_bytes = 0;

        /* Read raw bytes for up to 1.5 s */
        for (int attempt = 0; attempt < 15; attempt++)
        {
            struct pollfd pfd;
            pfd.fd = _uart_fd;
            pfd.events = POLLIN;

            int pret = ::poll(&pfd, 1, 100);
            if (pret > 0 && (pfd.revents & POLLIN))
            {
                ssize_t n = read(_uart_fd, probe_buf, sizeof(probe_buf));
                if (n > 0)
                {
                    got_data = true;
                    total_bytes += (int)n;

                    for (ssize_t j = 0; j < n; j++)
                    {
                        if (probe_buf[j] == '$')
                        {
                            detected_nmea = true;
                        }

                        if (j + 1 < n &&
                            probe_buf[j] == 0xB5 &&
                            probe_buf[j + 1] == 0x62)
                        {
                            detected_ubx = true;
                        }
                    }

                    if (detected_ubx || detected_nmea)
                    {
                        break;
                    }
                }
            }
        }

        if (got_data)
        {
            GPS_INFO("GPS found at %lu baud (%d bytes, ubx=%s, nmea=%s)",
                     (unsigned long)baudrates[i], total_bytes,
                     detected_ubx ? "yes" : "no",
                     detected_nmea ? "yes" : "no");
            found = true;
            found_baud = baudrates[i];
            break;
        }
    }

    if (!found)
    {
        GPS_ERR("GPS not responding at any baudrate");
        return -ENODEV;
    }

    /* ------------------------------------------------------------------ *
     * Step 2 – If only NMEA detected (factory default), enable UBX       *
     * output via CFG-VALSET.  The M10 accepts UBX commands on input      *
     * even when outputting only NMEA.                                    *
     * ------------------------------------------------------------------ */

    if (detected_nmea && !detected_ubx)
    {
        GPS_INFO("Factory-default NMEA detected – switching to UBX output");

        /* Enable UBX output on UART1 */
        uint8_t val_on = 1;
        if (sendCfgValset(UBX_CFG_KEY_UART1OUTPROT_UBX, &val_on, 1) == 0)
        {
            waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, UBX_ACK_TIMEOUT_SLOW_MS);
        }
        usleep(50000);

        /* Disable NMEA output on UART1 */
        uint8_t val_off = 0;
        if (sendCfgValset(UBX_CFG_KEY_UART1OUTPROT_NMEA, &val_off, 1) == 0)
        {
            waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, UBX_ACK_TIMEOUT_SLOW_MS);
        }
        usleep(50000);

        /* Enable UBX input protocol */
        if (sendCfgValset(UBX_CFG_KEY_UART1INPROT_UBX, &val_on, 1) == 0)
        {
            waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, UBX_ACK_TIMEOUT_SLOW_MS);
        }
        usleep(50000);

        /* Enable NAV-PVT output so we get position data */
        if (sendCfgValset(UBX_CFG_KEY_MSGOUT_NAV_PVT_UART1, &val_on, 1) == 0)
        {
            waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, UBX_ACK_TIMEOUT_SLOW_MS);
        }
        usleep(50000);

        /* Flush leftover NMEA and wait for first UBX frame */
        tcflush(_uart_fd, TCIFLUSH);
        GPS_INFO("Waiting for UBX data after protocol switch...");

        bool ubx_verified = false;
        for (int attempt = 0; attempt < 30; attempt++)
        {
            int ret = poll(100);
            if (ret > 0)
            {
                ubx_verified = true;
                GPS_INFO("UBX output confirmed after protocol switch");
                break;
            }
        }

        if (!ubx_verified)
        {
            GPS_ERR("UBX protocol switch failed – module may not support "
                    "CFG-VALSET; trying legacy fallback");

            /* Legacy fallback: CFG-PRT to set UBX-only */
            ubx_cfg_prt_s cfg;
            memset(&cfg, 0, sizeof(cfg));
            cfg.portID = 1;
            cfg.mode = 0x000008D0;
            cfg.baudRate = found_baud;
            cfg.inProtoMask = 0x03;   /* UBX + NMEA in */
            cfg.outProtoMask = 0x01;  /* UBX only out */
            sendMessage(UBX_CLASS_CFG, UBX_ID_CFG_PRT,
                        (uint8_t *)&cfg, sizeof(cfg));
            usleep(200000);
            tcflush(_uart_fd, TCIFLUSH);

            /* Legacy: enable NAV-PVT via CFG-MSG */
            setMessageRate(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1);
        }
    }

    /* Query module + protocol version (PX4 style capability detection) */
    if (requestMonVer(1500) < 0)
    {
        GPS_INFO("MON-VER unavailable, continuing with generic config path");
    }

    /* ------------------------------------------------------------------ *
     * Step 3 – Switch to 115200 baud (if not already there)              *
     * Try M10-native CFG-VALSET, then legacy CFG-PRT.  VERIFY the       *
     * switch worked before continuing – revert if it didn't.             *
     * ------------------------------------------------------------------ */

    if (found_baud != 115200)
    {
        GPS_INFO("Switching to 115200 baud...");

        /* M10 CFG-VALSET */
        uint32_t target_baud = 115200;
        int ret = sendCfgValset(UBX_CFG_KEY_UART1_BAUDRATE,
                                &target_baud, sizeof(target_baud));
        if (ret < 0)
        {
            /* Legacy CFG-PRT fallback */
            setBaudrate(115200);
        }

        /* Allow module to apply new baudrate */
        usleep(100000);

        /* Switch local UART */
        struct termios tio;
        tcgetattr(_uart_fd, &tio);
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tcsetattr(_uart_fd, TCSANOW, &tio);
        tcflush(_uart_fd, TCIOFLUSH);

        /* Verify communication at 115200 */
        bool baud_ok = false;
        for (int attempt = 0; attempt < 20; attempt++)
        {
            usleep(100000);
            ret = poll(100);
            if (ret > 0)
            {
                baud_ok = true;
                break;
            }
        }

        if (!baud_ok)
        {
            GPS_ERR("No data at 115200 – reverting to %lu",
                    (unsigned long)found_baud);

            speed_t speed;
            switch (found_baud)
            {
                case 9600:   speed = B9600;   break;
                case 38400:  speed = B38400;  break;
                case 57600:  speed = B57600;  break;
                default:     speed = B9600;   break;
            }

            tcgetattr(_uart_fd, &tio);
            cfsetispeed(&tio, speed);
            cfsetospeed(&tio, speed);
            tcsetattr(_uart_fd, TCSANOW, &tio);
            tcflush(_uart_fd, TCIOFLUSH);

            GPS_INFO("Continuing at %lu baud", (unsigned long)found_baud);
        }
        else
        {
            GPS_INFO("115200 baud verified OK");
        }
    }

    /* ------------------------------------------------------------------ *
     * Step 4 – Configure message output & rates                          *
     * Prefer CFG-VALSET (M10), fall back to legacy commands.             *
     * ------------------------------------------------------------------ */

    /* 4a – UBX output ON, NMEA output OFF */
    {
        uint8_t v = 1;
        if (sendCfgValset(UBX_CFG_KEY_UART1OUTPROT_UBX, &v, 1) == 0)
        {
            waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET,
                       UBX_ACK_TIMEOUT_MS);
        }

        v = 0;
        if (sendCfgValset(UBX_CFG_KEY_UART1OUTPROT_NMEA, &v, 1) == 0)
        {
            if (waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET,
                           UBX_ACK_TIMEOUT_MS) == 0)
            {
                GPS_INFO("NMEA output disabled");
            }
        }
    }

    /* 4b – Enable NAV-PVT output */
    {
        uint8_t rate = 1;
        int ret = sendCfgValset(UBX_CFG_KEY_MSGOUT_NAV_PVT_UART1,
                                &rate, 1);
        if (ret == 0)
        {
            ret = waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET,
                             UBX_ACK_TIMEOUT_MS);
        }

        if (ret != 0)
        {
            GPS_INFO("CFG-VALSET NAV-PVT failed – trying legacy CFG-MSG");
            ret = setMessageRate(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1);
        }

        if (ret == 0)
        {
            GPS_INFO("NAV-PVT output enabled");
        }
        else
        {
            GPS_ERR("Failed to enable NAV-PVT output");
        }
    }

    /* 4c – Set measurement rate to 5 Hz (200 ms) */
    {
        uint16_t meas_ms = 200;
        int ret = sendCfgValset(UBX_CFG_KEY_RATE_MEAS,
                                &meas_ms, sizeof(meas_ms));
        if (ret == 0)
        {
            ret = waitForAck(UBX_CLASS_CFG, UBX_ID_CFG_VALSET,
                             UBX_ACK_TIMEOUT_MS);
        }

        if (ret != 0)
        {
            GPS_INFO("CFG-VALSET rate failed – trying legacy CFG-RATE");
            ret = setMeasurementRate(200);
        }

        if (ret == 0)
        {
            GPS_INFO("Measurement rate set to 5 Hz");
        }
        else
        {
            GPS_ERR("Failed to set measurement rate");
        }
    }

    /* 4d – Set dynamic model to Airborne <2G (already uses CFG-VALSET) */
    {
        int ret = setDynamicModel(UBX_DYN_MODEL_AIRBORNE_2G);
        if (ret == 0)
        {
            GPS_INFO("Dynamic model: Airborne <2G");
        }
        else
        {
            GPS_ERR("Failed to set dynamic model");
        }
    }

    _configured = true;
    GPS_INFO("GPS configuration complete");

    return 0;
}

/****************************************************************************
 * requestMonVer - Poll MON-VER and wait until parser updates diagnostics
 ****************************************************************************/

int GPSUbx::requestMonVer(int timeout_ms)
{
    _gps_data.mon_ver_valid = false;

    int ret = sendMessage(UBX_CLASS_MON, UBX_ID_MON_VER, nullptr, 0);
    if (ret < 0)
    {
        return ret;
    }

    int elapsed_ms = 0;
    while (elapsed_ms < timeout_ms)
    {
        poll(50);

        if (_gps_data.mon_ver_valid)
        {
            return 0;
        }

        elapsed_ms += 50;
    }

    return -ETIMEDOUT;
}

/****************************************************************************
 * requestMonRf - Poll MON-RF and wait until parser updates RF diagnostics
 ****************************************************************************/

int GPSUbx::requestMonRf(int timeout_ms)
{
    _gps_data.mon_rf_valid = false;

    int ret = sendMessage(UBX_CLASS_MON, UBX_ID_MON_RF, nullptr, 0);
    if (ret < 0)
    {
        return ret;
    }

    int elapsed_ms = 0;
    while (elapsed_ms < timeout_ms)
    {
        poll(50);

        if (_gps_data.mon_rf_valid)
        {
            return 0;
        }

        elapsed_ms += 50;
    }

    return -ETIMEDOUT;
}

/****************************************************************************
 * refreshDiagnostics - update MON-VER and MON-RF snapshot
 ****************************************************************************/

int GPSUbx::refreshDiagnostics(int timeout_ms)
{
    if (_uart_fd < 0)
    {
        return -ENODEV;
    }

    int ret1 = requestMonVer(timeout_ms);
    int ret2 = requestMonRf(timeout_ms);
    int ret3 = requestNavSat(timeout_ms);

    if (ret1 < 0 && ret2 < 0 && ret3 < 0)
    {
        return ret1;
    }

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

        case UBX_ID_NAV_SAT:
            return parseNavSat(_payload, _payload_len);

        default:
            break;
        }
        break;

    case UBX_CLASS_ACK:
        return parseAck(_payload, _payload_len);

    case UBX_CLASS_MON:
        switch (_msg_id)
        {
        case UBX_ID_MON_VER:
            return parseMonVer(_payload, _payload_len);

        case UBX_ID_MON_RF:
            return parseMonRf(_payload, _payload_len);

        default:
            break;
        }
        break;

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
    const bool fix_ok = (pvt->flags & UBX_NAV_PVT_FLAGS_GNSS_FIX_OK) &&
                        (pvt->fixType >= UBX_FIX_TYPE_2D) &&
                        (pvt->numSV > 0);
    const bool hacc_valid = (pvt->hAcc != UINT32_MAX);
    const bool vacc_valid = (pvt->vAcc != UINT32_MAX);
    const bool sacc_valid = (pvt->sAcc != UINT32_MAX);

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

    /* Accuracy (convert from mm to m, mm/s to m/s); guard invalid sentinels */
    _gps_data.hacc = hacc_valid ? (pvt->hAcc * 1e-3f) : NAN;
    _gps_data.vacc = vacc_valid ? (pvt->vAcc * 1e-3f) : NAN;
    _gps_data.sacc = sacc_valid ? (pvt->sAcc * 1e-3f) : NAN;

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
    _gps_data.position_valid = fix_ok;
    _gps_data.velocity_valid = fix_ok;

    if (!fix_ok)
    {
        _gps_data.hacc = NAN;
        _gps_data.vacc = NAN;
        _gps_data.sacc = NAN;
        _gps_data.pdop = NAN;
    }

    GPS_LOG("NAV-PVT: fix=%u sats=%u lat=%.6f lon=%.6f alt=%.1f",
            pvt->fixType, pvt->numSV, _gps_data.lat, _gps_data.lon, _gps_data.alt_msl);

    return 1;  /* New data available */
}

/****************************************************************************
 * parseNavSat - Parse NAV-SAT message and compute robust summary metrics
 *
 * Why summary metrics instead of storing all satellites?
 * - Keep memory footprint low for embedded target.
 * - Still expose the most useful health indicators for diagnosis:
 *   total SV, used SV, mean/max C/N0, strongest satellite identity.
 ****************************************************************************/

int GPSUbx::parseNavSat(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(ubx_nav_sat_hdr_s))
    {
        return 0;
    }

    const ubx_nav_sat_hdr_s *hdr = reinterpret_cast<const ubx_nav_sat_hdr_s *>(payload);
    const uint16_t expect_len = (uint16_t)(sizeof(ubx_nav_sat_hdr_s) +
                                           hdr->numSvs * sizeof(ubx_nav_sat_block_s));

    if (len < expect_len)
    {
        GPS_LOG("NAV-SAT truncated: got %u expected %u", len, expect_len);
        return 0;
    }

    const ubx_nav_sat_block_s *blocks =
        reinterpret_cast<const ubx_nav_sat_block_s *>(payload + sizeof(ubx_nav_sat_hdr_s));

    uint32_t cno_sum = 0;
    uint8_t cno_count = 0;
    uint8_t cno_max = 0;
    uint8_t best_gnss = 0;
    uint8_t best_svid = 0;
    uint8_t used_count = 0;

    /*
     * UBX-NAV-SAT flags bit 3 (0x08) indicates svUsed in navigation solution.
     * We use this to estimate whether receiver sees satellites but cannot use them.
     */
    for (uint8_t i = 0; i < hdr->numSvs; i++)
    {
        const ubx_nav_sat_block_s *sv = &blocks[i];

        if (sv->flags & (1u << 3))
        {
            used_count++;
        }

        /*
         * cno==0 means no meaningful signal lock for this SV.
         * Skip from averaging to avoid biasing quality toward zero.
         */
        if (sv->cno > 0)
        {
            cno_sum += sv->cno;
            cno_count++;

            if (sv->cno > cno_max)
            {
                cno_max = sv->cno;
                best_gnss = sv->gnssId;
                best_svid = sv->svId;
            }
        }
    }

    _gps_data.nav_sat_valid = true;
    _gps_data.nav_sat_num_svs = hdr->numSvs;
    _gps_data.nav_sat_used_svs = used_count;
    _gps_data.nav_sat_cno_max = cno_max;
    _gps_data.nav_sat_cno_mean = (cno_count > 0) ?
                                 ((float)cno_sum / (float)cno_count) : NAN;
    _gps_data.nav_sat_best_gnss = best_gnss;
    _gps_data.nav_sat_best_svid = best_svid;

    return 0;
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
 * parseMonVer - Parse MON-VER (module + protocol version)
 ****************************************************************************/

int GPSUbx::parseMonVer(const uint8_t *payload, uint16_t len)
{
    if (len < 40)
    {
        return 0;
    }

    const char *sw = reinterpret_cast<const char *>(payload);
    const char *ext = reinterpret_cast<const char *>(payload + 40);
    uint16_t ext_len = len - 40;

    _gps_data.mon_ver_valid = true;

    /* Parse PROTVER=xx.yy from extension blocks */
    _gps_data.proto_major = 0;
    _gps_data.proto_minor = 0;

    for (uint16_t i = 0; i + 8 < ext_len; i += 30)
    {
        char linebuf[31];
        memset(linebuf, 0, sizeof(linebuf));
        memcpy(linebuf, &ext[i], (ext_len - i >= 30) ? 30 : (ext_len - i));
        const char *line = linebuf;

        if (strncmp(line, "PROTVER=", 8) == 0)
        {
            int major = 0;
            int minor = 0;
            if (sscanf(line + 8, "%d.%d", &major, &minor) >= 1)
            {
                _gps_data.proto_major = (uint16_t)major;
                _gps_data.proto_minor = (uint16_t)minor;
                break;
            }
        }
    }

    _proto_ver_27_or_higher = (_gps_data.proto_major >= 27);
    _gps_data.proto_ver_27_or_higher = _proto_ver_27_or_higher;

    /* Parse module name MOD=... */
    memset(_gps_data.module_name, 0, sizeof(_gps_data.module_name));

    for (uint16_t i = 0; i + 4 < ext_len; i += 30)
    {
        char linebuf[31];
        memset(linebuf, 0, sizeof(linebuf));
        memcpy(linebuf, &ext[i], (ext_len - i >= 30) ? 30 : (ext_len - i));
        const char *line = linebuf;
        if (strncmp(line, "MOD=", 4) == 0)
        {
            size_t out = 0;
            const char *m = line + 4;
            while (out + 1 < sizeof(_gps_data.module_name) &&
                   m[out] != '\0' && m[out] != ' ')
            {
                _gps_data.module_name[out] = m[out];
                out++;
            }
            _gps_data.module_name[out] = '\0';
            break;
        }
    }

    /* Infer generation from module/protocol */
    _board_generation = 0;

    if (strstr(_gps_data.module_name, "M10") != nullptr ||
        strstr(_gps_data.module_name, "F10") != nullptr)
    {
        _board_generation = 10;
    }
    else if (strstr(_gps_data.module_name, "M9") != nullptr ||
             strstr(_gps_data.module_name, "F9") != nullptr)
    {
        _board_generation = 9;
    }
    else if (strstr(_gps_data.module_name, "M8") != nullptr)
    {
        _board_generation = 8;
    }
    else if (_gps_data.proto_major >= 27)
    {
        _board_generation = 10;
    }

    _gps_data.hw_generation = _board_generation;

    GPS_INFO("MON-VER: sw=%.30s module=%s prot=%u.%u (>=27=%s)",
             sw,
             _gps_data.module_name[0] ? _gps_data.module_name : "unknown",
             _gps_data.proto_major,
             _gps_data.proto_minor,
             _proto_ver_27_or_higher ? "yes" : "no");

    return 0;
}

/****************************************************************************
 * parseMonRf - Parse MON-RF (antenna/jamming/noise diagnostics)
 ****************************************************************************/

int GPSUbx::parseMonRf(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(ubx_mon_rf_hdr_s) + sizeof(ubx_mon_rf_block_s))
    {
        return 0;
    }

    const ubx_mon_rf_hdr_s *hdr = reinterpret_cast<const ubx_mon_rf_hdr_s *>(payload);
    if (hdr->nBlocks == 0)
    {
        return 0;
    }

    const ubx_mon_rf_block_s *b =
        reinterpret_cast<const ubx_mon_rf_block_s *>(payload + sizeof(ubx_mon_rf_hdr_s));

    _gps_data.mon_rf_valid = true;
    _gps_data.rf_blocks = hdr->nBlocks;
    _gps_data.rf_ant_status = b->antStatus;
    _gps_data.rf_ant_power = b->antPower;
    _gps_data.rf_jam_ind = b->jamInd;
    _gps_data.rf_noise_per_ms = b->noisePerMS;
    _gps_data.rf_agc_cnt = b->agcCnt;

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
 * requestNavSat - Poll NAV-SAT and wait for summary update
 ****************************************************************************/

int GPSUbx::requestNavSat(int timeout_ms)
{
    _gps_data.nav_sat_valid = false;

    int ret = sendMessage(UBX_CLASS_NAV, UBX_ID_NAV_SAT, nullptr, 0);
    if (ret < 0)
    {
        return ret;
    }

    int elapsed_ms = 0;
    while (elapsed_ms < timeout_ms)
    {
        poll(50);

        if (_gps_data.nav_sat_valid)
        {
            return 0;
        }

        elapsed_ms += 50;
    }

    return -ETIMEDOUT;
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
