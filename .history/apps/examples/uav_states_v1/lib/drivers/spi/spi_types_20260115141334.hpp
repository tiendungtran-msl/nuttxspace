#pragma once

#include <stdint.h>
#include <nuttx/spi/spi.h>

namespace drivers {
    namespace spi {

        /**
         * @brief SPI information types
         * 
         * 1. SPI transfer modes
         * 2. Lock modes
         */

        /** SPI transfer mode */
        enum class TransferMode :  uint8_t {
            BLOCKING,           // Synchronous transfer
            DMA,                // Asynchronous DMA transfer
            INTERRUPT           // Interrupt-driven
        };

        /** Lock mode for thread safety */
        enum class LockMode : uint8_t {
            NONE,               // No locking (single-threaded or ISR)
            THREADS,            // Mutex locking (default)
            PREEMPTION          // Critical section (disable IRQ)
        };

        /** SPI configuration */
        struct Config {
            uint8_t bus;                    // SPI bus number (1, 2, 3...)
            uint32_t devid;                 // Device select ID
            spi_mode_e mode;                // SPI mode (CPOL/CPHA)
            uint32_t frequency;             // Clock frequency (Hz)
            LockMode lock_mode;             // Thread safety mode
            
            // Defaults
            Config() 
                : bus(1)
                , devid(0)
                , mode(SPIDEV_MODE3)
                , frequency(1000000)
                , lock_mode(LockMode::THREADS)
            {}
        };

        /** Transfer result */
        struct TransferResult {
            int status;                     // 0 = OK, <0 = error code
            uint32_t bytes_transferred;
            uint64_t timestamp_us;          // HRT timestamp
        };

    } // namespace spi
} // namespace drivers