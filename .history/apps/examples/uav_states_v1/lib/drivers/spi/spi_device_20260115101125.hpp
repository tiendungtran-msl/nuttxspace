#pragma once

#include "spi_types.hpp"
#include "../drivers_framework/device_id.hpp"
#include "../utils/debug.hpp"
#include "../utils/critical.hpp"

#include <nuttx/spi/spi.h>
#include <errno.h>

namespace drivers {
    namespace spi {

        /**
         * @brief Base class for SPI devices (Pure NuttX)
         * 
         * Features:
         * - Thread-safe transfers
         * - Dynamic frequency switching
         * - 8-bit and 16-bit transfers
         * - Register read/write helpers
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
             * Get device type
             */
            uint8_t device_type() const { return _device_id.fields.devtype; }
            
            /**
             * Set SPI frequency
             * 
             * @param freq_hz   Frequency in Hz
             */
            void set_frequency(uint32_t freq_hz) { 
                _config.frequency = freq_hz; 
            }
            
            uint32_t get_frequency() const { 
                return _config.frequency; 
            }
            
            /**
             * Set locking mode
             */
            void set_lock_mode(LockMode mode) { 
                _config.lock_mode = mode; 
            }
            
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
             * Read single register
             */
            int read_reg(uint8_t reg, uint8_t &value);
            
            /**
             * Read multiple registers
             */
            int read_reg(uint8_t reg, uint8_t *data, size_t len);
            
            /**
             * Write single register
             */
            int write_reg(uint8_t reg, uint8_t value);
            
            /**
             * Write and verify
             */
            int write_reg_verified(uint8_t reg, uint8_t value, uint8_t mask = 0xFF);
            
            /**
             * Modify register bits (read-modify-write)
             */
            int modify_reg(uint8_t reg, uint8_t clear_bits, uint8_t set_bits);
            
            /**
             * Read multiple bytes (burst read)
             */
            int read_burst(uint8_t start_reg, uint8_t *data, size_t len);
            
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
            
            /**
             * Get device handle (for advanced users)
             */
            struct spi_dev_s* get_device() { return _dev; }
            
        private:
            Config _config;
            DeviceId _device_id;
            struct spi_dev_s *_dev;
            irqstate_t _irq_flags;  // For critical section
            
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