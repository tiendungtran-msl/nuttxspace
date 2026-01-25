#pragma once

#include "spi_types.hpp"
#include "../drivers_framework/device_id.hpp"
#include <nuttx/spi/spi.h>
#include <px4_platform_common/defines.h>
#include <errno.h>

namespace drivers {
namespace spi {

/**
 * @brief Base class for SPI devices
 * 
 * Features:
 * - Thread-safe transfers with configurable locking
 * - Dynamic frequency switching
 * - 8-bit and 16-bit transfers
 * - Device ID management
 * - Timeout support
 * - Error handling
 */
class Device {
public:
    /** 
     * Constructor
     * 
     * @param config    SPI configuration
     * @param dev_type  Device type identifier
     */
    Device(const Config &config, uint8_t dev_type);
    
    virtual ~Device();
    
    // Delete copy/move
    Device(const Device &) = delete;
    Device &operator=(const Device &) = delete;
    
    /**
     * Initialize SPI device
     * 
     * @return 0 on success, -errno on failure
     */
    int init();
    
    /**
     * Check if device is initialized
     */
    bool is_initialized() const { return _dev != nullptr; }
    
    /**
     * Get device ID
     */
    DeviceId device_id() const { return _device_id; }
    
    /**
     * Set SPI frequency
     * 
     * @param freq_hz   Frequency in Hz
     */
    void set_frequency(uint32_t freq_hz) { _config.frequency = freq_hz; }
    
    uint32_t get_frequency() const { return _config.frequency; }
    
    /**
     * Set locking mode
     */
    void set_lock_mode(LockMode mode) { _config.lock_mode = mode; }
    
    /**
     * 8-bit transfer
     * 
     * @param send      Data to send (nullptr = send zeros)
     * @param recv      Receive buffer (nullptr = discard)
     * @param len       Number of bytes
     * @return          0 on success, -errno on failure
     */
    int transfer(const uint8_t *send, uint8_t *recv, size_t len);
    
    /**
     * 16-bit transfer
     */
    int transfer_word(const uint16_t *send, uint16_t *recv, size_t len);
    
    /**
     * Read register (convenience function)
     * 
     * @param reg       Register address
     * @param data      Receive buffer
     * @param len       Number of bytes to read
     */
    int read_reg(uint8_t reg, uint8_t *data, size_t len);
    
    /**
     * Write register (convenience function)
     */
    int write_reg(uint8_t reg, uint8_t value);
    
    /**
     * Write register with verify
     */
    int write_reg_verified(uint8_t reg, uint8_t value, uint8_t mask = 0xFF);
    
    /**
     * Modify register bits
     */
    int modify_reg(uint8_t reg, uint8_t clear_bits, uint8_t set_bits);
    
protected:
    /**
     * Probe function - override to check device presence
     * 
     * @return 0 if device detected, -errno otherwise
     */
    virtual int probe() { return 0; }
    
    /**
     * Get bus number
     */
    uint8_t get_bus() const { return _config.bus; }
    
private:
    Config _config;
    DeviceId _device_id;
    struct spi_dev_s *_dev;
    
    /** Internal transfer implementation */
    int _transfer(const uint8_t *send, uint8_t *recv, size_t len, uint8_t bits);
    
    /** Lock/unlock helpers */
    void _lock();
    void _unlock();
    
    /** Configure SPI before transfer */
    void _configure();
};

} // namespace spi
} // namespace drivers