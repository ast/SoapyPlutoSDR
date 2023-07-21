#include "SoapyPlutoSDR.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>

std::vector<std::string>
SoapyPlutoSDR::getStreamFormats(const int direction,
                                const size_t channel) const {
  std::vector<std::string> formats;

  formats.push_back(SOAPY_SDR_CS8);
  formats.push_back(SOAPY_SDR_CS12);
  formats.push_back(SOAPY_SDR_CS16);
  formats.push_back(SOAPY_SDR_CF32);

  return formats;
}

// #include <SoapySDR/ConverterRegistry.hpp>

std::string SoapyPlutoSDR::getNativeStreamFormat(const int direction,
                                                 const size_t channel,
                                                 double &fullScale) const {
  // TODO: double check these values

  if (direction == SOAPY_SDR_RX) {
    fullScale = 2048; // RX expects 12 bit samples LSB aligned
  } else if (direction == SOAPY_SDR_TX) {
    fullScale = 32768; // TX expects 12 bit samples MSB aligned
  }

  return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList
SoapyPlutoSDR::getStreamArgsInfo(const int direction,
                                 const size_t channel) const {
  SoapySDR::ArgInfoList streamArgs;

  return streamArgs;
}

bool SoapyPlutoSDR::IsValidRxStreamHandle(SoapySDR::Stream *handle) const {
  if (handle == nullptr) {
    return false;
  }

  // handle is an opaque pointer hiding either rx_stream or tx_streamer:
  // check that the handle matches one of them, consistently with direction:
  if (rx_stream_) {
    // test if these handles really belong to us:
    if (reinterpret_cast<rx_streamer *>(handle) == rx_stream_.get()) {
      return true;
    }
  }

  return false;
}

bool SoapyPlutoSDR::IsValidTxStreamHandle(SoapySDR::Stream *handle) const {
  if (handle == nullptr) {
    return false;
  }

  // handle is an opaque pointer hiding either rx_stream or tx_streamer:
  // check that the handle matches one of them, consistently with direction:
  if (tx_stream_) {
    // test if these handles really belong to us:
    if (reinterpret_cast<tx_streamer *>(handle) == tx_stream_.get()) {
      return true;
    }
  }

  return false;
}

SoapySDR::Stream *
SoapyPlutoSDR::setupStream(const int direction, const std::string &format,
                           const std::vector<size_t> &channels,
                           const SoapySDR::Kwargs &args) {

  // TODO: check the format
  plutosdrStreamFormat streamFormat;

  // Assert that the channel is valid
  if(channels.size() != 1 and channels[0] == 0) {
    throw std::runtime_error("setupStream: invalid channel");
  }

  for (const auto &channel : channels) {
    SoapySDR::logf(SOAPY_SDR_DEBUG,
                   "setupStream: direction=%d, format=%s, channel=%d",
                   direction, format.c_str(), channel);
  }

  if (format == SOAPY_SDR_CF32) {
    SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
    streamFormat = PLUTO_SDR_CF32;
  } else if (format == SOAPY_SDR_CS16) {
    SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
    streamFormat = PLUTO_SDR_CS16;
  } else if (format == SOAPY_SDR_CS12) {
    SoapySDR_log(SOAPY_SDR_INFO, "Using format CS12.");
    streamFormat = PLUTO_SDR_CS12;
  } else if (format == SOAPY_SDR_CS8) {
    SoapySDR_log(SOAPY_SDR_INFO, "Using format CS8.");
    streamFormat = PLUTO_SDR_CS8;
  } else {
    throw std::runtime_error("setupStream invalid format '" + format +
                             "' -- Only CS8, CS12, CS16 and CF32 are supported "
                             "by SoapyPlutoSDR module.");
  }

  if (direction == SOAPY_SDR_RX) {

    const auto chan = iio_device_find_channel(device_, "altvoltage0", true);
    iio_channel_attr_write_bool(chan, "powerdown",
                                false); // Turn ON RX LO

    const auto cloned_ctx = iio_context_clone(ctx_);
    if (cloned_ctx == nullptr) {
      throw std::runtime_error("iio_context_clone(ctx) == nullptr");
    }

    rx_stream_ = std::unique_ptr<rx_streamer>(
        new rx_streamer(cloned_ctx, streamFormat, channels, args));

    return reinterpret_cast<SoapySDR::Stream *>(this->rx_stream_.get());
  } else if (direction == SOAPY_SDR_TX) {

    iio_channel_attr_write_bool(
        iio_device_find_channel(device_, "altvoltage1", true), "powerdown",
        false); // Turn ON TX LO

    this->tx_stream_ = std::unique_ptr<tx_streamer>(
        new tx_streamer(tx_dev_, streamFormat, channels, args));

    return reinterpret_cast<SoapySDR::Stream *>(this->tx_stream_.get());
  }

  return nullptr;
}

void SoapyPlutoSDR::closeStream(SoapySDR::Stream *handle) {
  // scope lock:
  {
    // std::lock_guard<std::mutex> lock(rx_device_mutex);

    if (IsValidRxStreamHandle(handle)) {
      this->rx_stream_.reset();

      iio_channel_attr_write_bool(
          iio_device_find_channel(device_, "altvoltage0", true), "powerdown",
          true); // Turn OFF RX LO
    }
  }

  // scope lock :
  {
    // std::lock_guard<std::mutex> lock(tx_device_mutex);

    if (IsValidTxStreamHandle(handle)) {
      this->tx_stream_.reset();

      iio_channel_attr_write_bool(
          iio_device_find_channel(device_, "altvoltage1", true), "powerdown",
          true); // Turn OFF TX LO
    }
  }
}

size_t SoapyPlutoSDR::getStreamMTU(SoapySDR::Stream *handle) const {

  if (IsValidRxStreamHandle(handle)) {
    return rx_stream_->get_mtu_size();
  }

  if (IsValidTxStreamHandle(handle)) {
    return 4096;
  }

  return 0;
}

int SoapyPlutoSDR::activateStream(SoapySDR::Stream *handle, const int flags,
                                  const long long timeNs,
                                  const size_t numElems) {
  if (flags & ~SOAPY_SDR_END_BURST)
    return SOAPY_SDR_NOT_SUPPORTED;

  if (IsValidRxStreamHandle(handle)) {
    return this->rx_stream_->start(flags, timeNs, numElems);
  }

  return 0;
}

int SoapyPlutoSDR::deactivateStream(SoapySDR::Stream *handle, const int flags,
                                    const long long timeNs) {

  if (IsValidRxStreamHandle(handle)) {
    return this->rx_stream_->stop(flags, timeNs);
  }

  if (IsValidTxStreamHandle(handle)) {
    this->tx_stream_->flush();
    return 0;
  }

  return 0;
}

int SoapyPlutoSDR::readStream(SoapySDR::Stream *handle, void *const *buffs,
                              const size_t numElems, int &flags,
                              long long &timeNs, const long timeoutUs) {

  // std::lock_guard<std::mutex> lock(rx_device_mutex);

  if (IsValidRxStreamHandle(handle)) {
    int ret = (int)rx_stream_->recv(buffs, numElems, flags, timeNs, timeoutUs);
    return ret;
  } else {
    return SOAPY_SDR_NOT_SUPPORTED;
  }
}

int SoapyPlutoSDR::writeStream(SoapySDR::Stream *handle,
                               const void *const *buffs, const size_t numElems,
                               int &flags, const long long timeNs,
                               const long timeoutUs) {

  if (IsValidTxStreamHandle(handle)) {
    return this->tx_stream_->send(buffs, numElems, flags, timeNs, timeoutUs);
    ;
  } else {
    return SOAPY_SDR_NOT_SUPPORTED;
  }
}

int SoapyPlutoSDR::readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask,
                                    int &flags, long long &timeNs,
                                    const long timeoutUs) {

  return SOAPY_SDR_NOT_SUPPORTED;
}
