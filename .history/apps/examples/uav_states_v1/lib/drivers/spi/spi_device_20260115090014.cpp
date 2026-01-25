#include "spi_device.hpp"
#include "../../platforms/nuttx/hrt/hrt.h"
#include <nuttx/arch.h>
#include <px4_platform_common/spi.h>

#ifndef CONFIG_SPI_EXCHANGE
# error "This driver requires CONFIG_SPI_EXCHANGE"
#endif

namespace drivers {
namespace spi {

Device::Device(const Config &config, uint8_t dev_type)
    : _config(config)
    , _device_id()
    , _dev(nullptr)
{
    // Build device ID
    _device_id.fields.devtype = dev_type;
    _device_id.fields.bus_type = static_cast<uint8_t>(BusType::SPI);
    _device_id.fields. bus = config.bus;
    _device_id.fields.address = static_cast<uint8_t>(config.devid >> 8);
}

Device::~Device()
{
    // Note: NuttX doesn't provide spi_uninitialize
    _dev = nullptr;
}

int Device::init()
{
    if (_dev != nullptr) {
        return 0; // Already initialized
    }
    
    // Initialize SPI bus
    _dev = px4_spibus_initialize(_config.bus);
    
    if (_dev == nullptr) {
        PX4_ERR("SPI%d init failed", _config.bus);
        return -ENODEV;
    }
    
    // Deselect device (ensure high-to-low transition)
    SPI_SELECT(_dev, _config.devid, false);
    up_udelay(1);
    
    // Probe device
    int ret = probe();
    if (ret != 0) {
        PX4_ERR("SPI%d probe failed: %d", _config.bus, ret);
        return ret;
    }
    
    PX4_INFO("SPI%d device 0x%02X init OK @ %u Hz", 
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
    _configure();
    
    // Select device
    SPI_SELECT(_dev, _config.devid, true);
    
    // Set bit width
    SPI_SETBITS(_dev, bits);
    
    // Perform transfer
    SPI_EXCHANGE(_dev, send, recv, len);
    
    // Deselect device
    SPI_SELECT(_dev, _config.devid, false);
    
    return 0;
}

void Device::_configure()
{
    SPI_SETFREQUENCY(_dev, _config.frequency);
    SPI_SETMODE(_dev, _config.mode);
}

void Device::_lock()
{
    // Check if in interrupt context
    if (up_interrupt_context()) {
        return; // No locking in ISR
    }
    
    switch (_config.lock_mode) {
    case LockMode::PREEMPTION:  {
        irqstate_t flags = px4_enter_critical_section();
        // Store flags if needed for nested calls
        (void)flags;
        break;
    }
    
    case LockMode::THREADS: 
        SPI_LOCK(_dev, true);
        break;
        
    case LockMode:: NONE:
    default:
        break;
    }
}

void Device::_unlock()
{
    if (up_interrupt_context()) {
        return;
    }
    
    switch (_config.lock_mode) {
    case LockMode:: PREEMPTION:
        px4_leave_critical_section(0); // Should restore saved flags
        break;
        
    case LockMode:: THREADS:
        SPI_LOCK(_dev, false);
        break;
        
    case LockMode::NONE:
    default:
        break;
    }
}

// ========== Convenience Functions ==========

int Device::read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    // Most SPI sensors:  set MSB for read
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

int Device:: write_reg(uint8_t reg, uint8_t value)
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
    up_udelay(100);
    
    // Read back and verify
    uint8_t readback;
    ret = read_reg(reg, &readback, 1);
    if (ret != 0) {
        return ret;
    }
    
    if ((readback & mask) != (value & mask)) {
        PX4_ERR("Reg 0x%02X verify failed: wrote 0x%02X, read 0x%02X", 
                reg, value, readback);
        return -EIO;
    }
    
    return 0;
}

int Device::modify_reg(uint8_t reg, uint8_t clear_bits, uint8_t set_bits)
{
    uint8_t value;
    
    // Read current value
    int ret = read_reg(reg, &value, 1);
    if (ret != 0) {
        return ret;
    }
    
    // Modify
    value &= ~clear_bits;
    value |= set_bits;
    
    // Write back
    return write_reg(reg, value);
}

} // namespace spi
} // namespace drivers