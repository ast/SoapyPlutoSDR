#pragma once

#include <SoapySDR/Types.hpp>
#include <iio.h>

#include "stream_format.hpp"


/* TX Streamer */
class tx_streamer {

public:
  tx_streamer(const iio_device *dev, const plutosdrStreamFormat format,
              const std::vector<size_t> &channels,
              const SoapySDR::Kwargs &args);
  ~tx_streamer();
  int send(const void *const *buffs, const size_t numElems, int &flags,
           const long long timeNs, const long timeoutUs);
  int flush();

private:
  int send_buf();
  bool has_direct_copy();

  std::vector<iio_channel *> channel_list;
  const iio_device *dev;
  const plutosdrStreamFormat format;

  iio_buffer *buf;
  size_t buf_size;
  size_t items_in_buf;
  bool direct_copy;
};
