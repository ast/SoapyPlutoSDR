#include "rx_streamer.hpp"

#include <SoapySDR/ConverterRegistry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>

#include <cstring>
#include <stdexcept>

#define DEFAULT_RX_BUFFER_SIZE (1 << 16)

void rx_streamer::set_buffer_size_by_samplerate(const size_t samplerate) {
  SoapySDR::logf(SOAPY_SDR_INFO, "Set buffer size by samplerate: %lu",
                 (unsigned long)samplerate);
}

void rx_streamer::set_mtu_size(const size_t mtu_size) {
  SoapySDR::logf(SOAPY_SDR_INFO, "Set MTU Size: %lu", (unsigned long)mtu_size);
}

rx_streamer::rx_streamer(iio_context *cloned_ctx,
                         const plutosdrStreamFormat _format,
                         const std::vector<size_t> &channels,
                         const SoapySDR::Kwargs &args)
    : ctx_(cloned_ctx), buffer_size_(DEFAULT_RX_BUFFER_SIZE), buf_(nullptr),
      format_(_format), mtu_size_(DEFAULT_RX_BUFFER_SIZE)

{
  // TODO: maybe better to clone here?

  if (format_ == PLUTO_SDR_CF32) {
  } else {
    throw std::runtime_error("unsupported format");
  }

  // Get converter function
  // TODO: need to register custom functions I think.
  converter_ = SoapySDR::ConverterRegistry::getFunction("CS16", "CF32");
  if (converter_ == nullptr) {
    // TODO: check error handling
    throw std::runtime_error("converter not found");
  }

  // Find RX device on our cloned context.
  dev_ = iio_context_find_device(ctx_, "cf-ad9361-lpc");
  if (dev_ == nullptr) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "cf-ad9361-lpc not found!");
    throw std::runtime_error("cf-ad9361-lpc not found!");
  }

  // Get channels
  rx0_i = iio_device_find_channel(dev_, "voltage0", 0);
  rx0_q = iio_device_find_channel(dev_, "voltage1", 0);

  // Check ok
  if (rx0_i == nullptr or rx0_q == nullptr) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "iio_device_get_channel failed!");
    throw std::runtime_error("iio_device_get_channel failed!");
  }
  // Enable channels
  iio_channel_enable(rx0_i);
  iio_channel_enable(rx0_q);

  // Buffer will be created when we start streaming.
  items_in_buffer_ = 0;
  byte_offset_ = 0;

  // TODO fix
  buffer_size_ = DEFAULT_RX_BUFFER_SIZE;
  mtu_size_ = buffer_size_;
}

rx_streamer::~rx_streamer() {

  // Stop buffer
  if (buf_) {
    // Stop anyone waiting
    iio_buffer_cancel(buf_);
    iio_buffer_destroy(buf_);
    buf_ = nullptr;
  }

  // Disable channels
  iio_channel_disable(rx0_i);
  iio_channel_disable(rx0_q);

  if (ctx_) {
    // This is the cloned ctx
    iio_context_destroy(ctx_);
    ctx_ = nullptr;
  }
}

size_t rx_streamer::recv(void *const *buffs, const size_t numElems, int &flags,
                         long long &timeNs, const long timeoutUs) {
  // TODO: handle timeout and conversion

  if (items_in_buffer_ <= 0) {
    ssize_t ret;
    ret = iio_buffer_refill(buf_);
    if (ret < 0) {
      // TODO: find how to set timeout
      SoapySDR::logf(SOAPY_SDR_ERROR, "refill failed");
      return static_cast<size_t>(SOAPY_SDR_TIMEOUT);
    }

    // TODO: fix cast
    items_in_buffer_ = (size_t)ret / (size_t)iio_buffer_step(buf_);
    byte_offset_ = 0;
  }

  // How many elems to copy.
  size_t elems_to_copy = std::min(items_in_buffer_, numElems);
  uint8_t *src = (uint8_t *)iio_buffer_start(buf_) + byte_offset_;

  // Compensate for the fact that our samples are 12bit but the
  // default converter scales to 16 bits.
  const auto scaler = 32768. / 2048.;
  converter_(src, buffs[0], elems_to_copy, scaler);

  // Subtract what we have read.
  items_in_buffer_ -= elems_to_copy;
  // Advance into the buffer
  byte_offset_ += elems_to_copy * iio_buffer_step(buf_);

  // Return how many elements we copied.
  return elems_to_copy;
}

int rx_streamer::start(const int flags, const long long timeNs,
                       const size_t numElems) {

  // TODO: assert stopped
  SoapySDR::logf(SOAPY_SDR_DEBUG, "start(): numElems: %lu", numElems);

  // re-create buffer
  buf_ = iio_device_create_buffer(dev_, buffer_size_,
                                  false); // cyclic

  if (buf_ == nullptr) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "start(): unable to create buffer");
    throw std::runtime_error("start(): unable to create buffer");
  }

  direct_copy_ = has_direct_copy();
  if (not direct_copy_) {
    throw std::runtime_error("buffer doesn't support direct copy");
  }

  SoapySDR::logf(SOAPY_SDR_INFO, "Has direct RX copy: %d", (int)direct_copy_);

  return 0;
}

int rx_streamer::stop(const int flags, const long long timeNs) {
  // cancel first
  if (buf_) {
    iio_buffer_cancel(buf_);
    iio_buffer_destroy(buf_);
    buf_ = nullptr;
  }

  items_in_buffer_ = 0;
  byte_offset_ = 0;

  return 0;
}

bool rx_streamer::has_direct_copy() {

  const auto buf_step = iio_buffer_step(buf_);

  // Check if samples are interleaved
  if (buf_step != (2 * sizeof(int16_t))) {
    return false;
  }

  // Check that start point to the first sample.
  if (iio_buffer_start(buf_) != iio_buffer_first(buf_, rx0_i)) {
    return false;
  }

  // TODO: Not sure about this purpose of this test.
  int16_t test_dst, test_src = 0x1234;
  iio_channel_convert(rx0_i, &test_dst, (const void *)&test_src);
  if (test_src != test_dst) {
    return false;
  }

  return true;
}
