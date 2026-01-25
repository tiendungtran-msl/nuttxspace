#include "spi_device.hpp"
#include "../../platforms/nuttx/hrt/hrt.h"
#include "../../platforms/nuttx/board/spi_config.h"

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <unistd.h>

#ifndef CONFIG_SPI_EXCHANGE
# error "This driver requires CONFIG_SPI_EXCHANGE"
#endif

namespace drivers {
    namespace spi {

        Device::Device(const Config &config, uint8_t dev_type)
            : _config(config)
            , _device_id()
            , _dev(nullptr)
            , _irq_flags(0)
        {
            // Build device ID
            _device_id.fields.devtype = dev_type;
            _device_id.fields.bus_type = static_cast<uint8_t>(BusType::SPI);
            _device_id.fields.bus = config.bus;
            _device_id.fields.address = static_cast<uint8_t>(config.devid & 0xFF);
        }

        Device::~Device()
        {
            // NuttX doesn't provide SPI de-initialization
            // Just set pointer to null
            _dev = nullptr;
        }

        int Device::init()
        {
            if (_dev != nullptr) {
                DRIVER_WARN("SPI device already initialized");
                return 0;
            }
            
            // Check if board has this bus
            if (!board_has_spi_bus(_config.bus)) {
                DRIVER_ERR("SPI bus %d not available on this board", _config. bus);
                return -ENODEV;
            }
            
            // Initialize SPI bus
            _dev = stm32_spibus_initialize(_config.bus);
            
            if (_dev == nullptr) {
                DRIVER_ERR("Failed to initialize SPI bus %d", _config.bus);
                return -ENXIO;
            }
            
            // Deselect device (ensure high-to-low transition on next select)
            SPI_SELECT(_dev, _config.devid, false);
            hrt_usleep(10);  // Small delay
            
            // Probe device
            int ret = probe();
            if (ret != 0) {
                DRIVER_ERR("Device probe failed on SPI%d:  %d", _config.bus, ret);
                return ret;
            }
            
            DRIVER_INFO("SPI%d device 0x%02X initialized @ %u Hz", 
                        _config.bus, 
                        _device_id.fields.devtype,
                        _config.frequency);
            
            return 0;
        }

        int Device::transfer(const uint8_t *send, uint8_t *recv, size_t len)
        {
            if (!_dev) {
                return -ENODEV;
            }
            
            if (len == 0 || (send == nullptr && recv == nullptr)) {
                return -EINVAL;
            }
            
            _lock();
            int ret = _transfer(send, recv, len, 8);
            _unlock();
            
            return ret;
        }

        int Device::transfer_word(const uint16_t *send, uint16_t *recv, size_t len)
        {
            if (!_dev) {
                return -ENODEV;
            }
            
            if (len == 0 || (send == nullptr && recv == nullptr)) {
                return -EINVAL;
            }
            
            _lock();
            int ret = _transfer(reinterpret_cast<const uint8_t*>(send), 
                                reinterpret_cast<uint8_t*>(recv), 
                                len, 16);
            _unlock();
            
            return ret;
        }

        int Device::_transfer(const uint8_t *send, uint8_t *recv, size_t len, uint8_t bits)
        {
            // Configure SPI parameters
            _configure();
            
            // Set bit width
            SPI_SETBITS(_dev, bits);
            
            // Select device
            SPI_SELECT(_dev, _config.devid, true);
            
            // Perform transfer
            SPI_EXCHANGE(_dev, send, recv, len);
            
            // Deselect device
            SPI_SELECT(_dev, _config.devid, false);
            
            return 0;
        }

        void Device::_configure()
        {
            SPI_SETFREQUENCY(_dev, _config. frequency);
            SPI_SETMODE(_dev, _config. mode);
        }

        void Device::_lock()
        {
            // Don't lock in interrupt context
            if (utils::in_interrupt()) {
                return;
            }
            
            switch (_config.lock_mode) {
            case LockMode::PREEMPTION:
                _irq_flags = enter_critical_section();
                break;
                
            case LockMode:: THREADS:
                SPI_LOCK(_dev, true);
                break;
                
            case LockMode::NONE: 
            default:
                break;
            }
        }

        void Device::_unlock()
        {
            if (utils::in_interrupt()) {
                return;
            }
            
            switch (_config.lock_mode) {
            case LockMode::PREEMPTION: 
                leave_critical_section(_irq_flags);
                break;
                
            case LockMode:: THREADS:
                SPI_LOCK(_dev, false);
                break;
                
            case LockMode::NONE:
            default:
                break;
            }
        }

        // ==================== Register Access Functions ====================

        int Device::read_reg(uint8_t reg, uint8_t &value)
        {
            return read_reg(reg, &value, 1);
        }

        int Device::read_reg(uint8_t reg, uint8_t *data, size_t len)
        {
            if (!_dev || ! data) {
                return -EINVAL;
            }
            
            // Most SPI sensors use MSB=1 for read
            uint8_t cmd = reg | 0x80;
            
            _lock();
            _configure();
            SPI_SELECT(_dev, _config.devid, true);
            
            // Send register address
            SPI_SEND(_dev, cmd);
            
            // Read data
            SPI_RECVBLOCK(_dev, data, len);
            
            SPI_SELECT(_dev, _config.devid, false);
            _unlock();
            
            return 0;
        }

        int Device::write_reg(uint8_t reg, uint8_t value)
        {
            uint8_t cmd[2] = { 
                static_cast<uint8_t>(reg & 0x7F),  // Clear MSB for write
                value 
            };
            
            return transfer(cmd, nullptr, 2);
        }

        int Device::write_reg_verified(uint8_t reg, uint8_t value, uint8_t mask)
        {
            // Write register
            int ret = write_reg(reg, value);
            if (ret != 0) {
                return ret;
            }
            
            // Wait for write to complete
            hrt_usleep(100);
            
            // Read back
            uint8_t readback;
            ret = read_reg(reg, readback);
            if (ret != 0) {
                return ret;
            }
            
            // Verify
            if ((readback & mask) != (value & mask)) {
                DRIVER_ERR("Register 0x%02X verify failed:  wrote 0x%02X, read 0x%02X", 
                        reg, value, readback);
                return -EIO;
            }
            
            return 0;
        }

        int Device::modify_reg(uint8_t reg, uint8_t clear_bits, uint8_t set_bits)
        {
            uint8_t value;
            
            // Read current value
            int ret = read_reg(reg, value);
            if (ret != 0) {
                return ret;
            }
            
            // Modify
            value &= ~clear_bits;
            value |= set_bits;
            
            // Write back
            return write_reg(reg, value);
        }

        int Device::read_burst(uint8_t start_reg, uint8_t *data, size_t len)
        {
            if (!_dev || !data || len == 0) {
                return -EINVAL;
            }
            
            // Use auto-increment (device-dependent)
            // For MPU6050, just set MSB for read
            uint8_t cmd = start_reg | 0x80;
            
            _lock();
            _configure();
            SPI_SELECT(_dev, _config.devid, true);
            
            // Send start register
            SPI_SEND(_dev, cmd);
            
            // Read burst
            SPI_RECVBLOCK(_dev, data, len);
            
            SPI_SELECT(_dev, _config.devid, false);
            _unlock();
            
            return 0;
        }

    } // namespace spi
} // namespace drivers