#pragma once

#include <SoapySDR/ConverterRegistry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>

#include <iio.h>

#include "stream_format.hpp"

/* RX Streamer */
class rx_streamer {
public:
  rx_streamer(iio_context *ctx,
              // const iio_device *dev,
              const plutosdrStreamFormat format,
              const std::vector<size_t> &channels,
              const SoapySDR::Kwargs &args);

  ~rx_streamer();

  size_t recv(void *const *buffs, const size_t numElems, int &flags,
              long long &timeNs, const long timeoutUs = 100000);

  int start(const int flags, const long long timeNs, const size_t numElems);

  int stop(const int flags, const long long timeNs = 100000);

  void set_buffer_size_by_samplerate(const size_t _samplerate);

  size_t get_mtu_size();

private:
  void set_buffer_size(const size_t _buffer_size);
  void set_mtu_size(const size_t mtu_size);

  bool has_direct_copy();

  // Converter function:
  SoapySDR::ConverterRegistry::ConverterFunction converter_;

  const iio_device *dev_;

  iio_context *ctx_;

  iio_channel *rx0_i;
  iio_channel *rx0_q;

  size_t buffer_size_;

  size_t byte_offset_;
  size_t items_in_buffer_;
  iio_buffer *buf_;
  const plutosdrStreamFormat format_;
  bool direct_copy_;
  size_t mtu_size_;
};
