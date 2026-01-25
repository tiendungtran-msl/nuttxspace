#pragma once
extern "C" {
#include <nuttx/spi/spi.h>
}
#include <cstdint>
#include <cstddef>

class SPIDevice {
public:
  SPIDevice(spi_dev_s *spi, uint32_t devid) : _spi(spi), _devid(devid) {}

  void lock()   { SPI_LOCK(_spi, true); }
  void unlock() { SPI_LOCK(_spi, false); }

  void set_bus_config(uint32_t freq_hz, uint8_t mode, uint8_t nbits=8)
  {
    SPI_SETFREQUENCY(_spi, freq_hz);
    SPI_SETMODE(_spi, mode);
    SPI_SETBITS(_spi, nbits);
  }

  void select(bool on) { SPI_SELECT(_spi, _devid, on); }

  int transfer(const uint8_t *tx, uint8_t *rx, size_t len)
  {
    SPI_EXCHANGE(_spi, tx, rx, len);
    return 0;
  }

private:
  spi_dev_s *_spi;
  uint32_t _devid;
};