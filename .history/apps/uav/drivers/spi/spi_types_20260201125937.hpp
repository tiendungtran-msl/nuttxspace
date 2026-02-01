/****************************************************************************
 * apps/uav/drivers/spi/spi_types.hpp
 *
 * SPI types và configuration structures
 ****************************************************************************/

#pragma once

#include <stdint.h>
#include <nuttx/spi/spi.h>

namespace drivers {
    namespace spi {

        /**
         * @brief SPI transfer mode
         */
        enum class TransferMode : uint8_t {
            BLOCKING,           /**< Synchronous transfer */
            DMA,                /**< Asynchronous DMA transfer */
            INTERRUPT           /**< Interrupt-driven */
        };

        /**
         * @brief Lock mode cho thread safety
         */
        enum class LockMode : uint8_t {
            NONE,               /**< No locking (single-threaded hoặc ISR) */
            THREADS,            /**< Mutex locking (default) */
            PREEMPTION          /**< Critical section (disable IRQ) */
        };

        /**
         * @brief SPI configuration
         */
        struct Config {
            uint8_t bus;                    /**< SPI bus number (1, 2, 3...) */
            uint32_t devid;                 /**< Device select ID */
            spi_mode_e mode;                /**< SPI mode (CPOL/CPHA) */
            uint32_t frequency;             /**< Clock frequency (Hz) */
            LockMode lock_mode;             /**< Thread safety mode */

            /* Defaults */
            Config()
                : bus(1)
                , devid(0)
                , mode(SPIDEV_MODE3)
                , frequency(1000000)
                , lock_mode(LockMode::THREADS)
            {}

            Config(uint8_t bus_, uint32_t devid_, spi_mode_e mode_,
                   uint32_t frequency_, LockMode lock_mode_)
                : bus(bus_)
                , devid(devid_)
                , mode(mode_)
                , frequency(frequency_)
                , lock_mode(lock_mode_)
            {}
        };

        /**
         * @brief Transfer result
         */
        struct TransferResult {
            int status;                     /**< 0 = OK, <0 = error code */
            uint32_t bytes_transferred;
            uint64_t timestamp_us;          /**< HRT timestamp */
        };

    } // namespace spi
} // namespace drivers
