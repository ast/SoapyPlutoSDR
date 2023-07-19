#include "rx_streamer.hpp"

#include <SoapySDR/Types.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Device.hpp>
#include <stdexcept>
#include <cstring>

#define DEFAULT_RX_BUFFER_SIZE (1 << 16)

void rx_streamer::set_buffer_size_by_samplerate(const size_t samplerate) {

  // Adapt buffer size (= MTU) as a tradeoff to minimize readStream overhead but
  // at the same time allow realtime applications. Keep it a power of 2 which
  // seems to be better. so try to target very roughly 60fps [30 .. 100]
  // readStream calls / s for realtime applications.
  int rounded_nb_samples_per_call = (int)::round(samplerate / 60.0);

  int power_of_2_nb_samples = 0;

  while (rounded_nb_samples_per_call > (1 << power_of_2_nb_samples)) {
    power_of_2_nb_samples++;
  }

  this->set_buffer_size(1 << power_of_2_nb_samples);

  SoapySDR_logf(SOAPY_SDR_INFO, "Auto setting Buffer Size: %lu",
                (unsigned long)buffer_size);

  // Recompute MTU from buffer size change.
  // We always set MTU size = Buffer Size.
  // On buffer size adjustment to sample rate,
  // MTU can be changed accordingly safely here.
  set_mtu_size(this->buffer_size);
}

void rx_streamer::set_mtu_size(const size_t mtu_size) {

  this->mtu_size = mtu_size;

  SoapySDR_logf(SOAPY_SDR_INFO, "Set MTU Size: %lu", (unsigned long)mtu_size);
}

rx_streamer::rx_streamer(const iio_device *_dev,
                         const plutosdrStreamFormat _format,
                         const std::vector<size_t> &channels,
                         const SoapySDR::Kwargs &args)
    : dev(_dev), buffer_size(DEFAULT_RX_BUFFER_SIZE), buf(nullptr),
      format(_format), mtu_size(DEFAULT_RX_BUFFER_SIZE)

{
  if (dev == nullptr) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-lpc not found!");
    throw std::runtime_error("cf-ad9361-lpc not found!");
  }
  unsigned int nb_channels = iio_device_get_channels_count(dev), i;
  for (i = 0; i < nb_channels; i++)
    iio_channel_disable(iio_device_get_channel(dev, i));

  // default to channel 0, if none were specified
  const std::vector<size_t> &channelIDs =
      channels.empty() ? std::vector<size_t>{0} : channels;

  for (i = 0; i < channelIDs.size() * 2; i++) {
    struct iio_channel *chn = iio_device_get_channel(dev, i);
    iio_channel_enable(chn);
    channel_list.push_back(chn);
  }

  if (args.count("bufflen") != 0) {

    try {
      size_t bufferLength = std::stoi(args.at("bufflen"));
      if (bufferLength > 0)
        this->set_buffer_size(bufferLength);
    } catch (const std::invalid_argument &) {
    }

  } else {

    long long samplerate;

    iio_channel_attr_read_longlong(
        iio_device_find_channel(dev, "voltage0", false), "sampling_frequency",
        &samplerate);

    this->set_buffer_size_by_samplerate(samplerate);
  }
}

rx_streamer::~rx_streamer() {
  if (buf) {
    iio_buffer_cancel(buf);
    iio_buffer_destroy(buf);
  }

  for (unsigned int i = 0; i < channel_list.size(); ++i) {
    iio_channel_disable(channel_list[i]);
  }
}

size_t rx_streamer::recv(void *const *buffs, const size_t numElems, int &flags,
                         long long &timeNs, const long timeoutUs) {
  //
  if (items_in_buffer <= 0) {

    // auto before =
    // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    if (!buf) {
      return 0;
    }

    ssize_t ret = iio_buffer_refill(buf);

    // auto after =
    // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    if (ret < 0)
      return SOAPY_SDR_TIMEOUT;

    items_in_buffer = (unsigned long)ret / iio_buffer_step(buf);

    // SoapySDR_logf(SOAPY_SDR_INFO, "iio_buffer_refill took %d ms to refill %d
    // items", (int)(after - before), items_in_buffer);

    byte_offset = 0;
  }

  size_t items = std::min(items_in_buffer, numElems);

  ptrdiff_t buf_step = iio_buffer_step(buf);

  if (direct_copy) {
    // optimize for single RX, 2 channel (I/Q), same endianess direct copy
    // note that RX is 12 bits LSB aligned, i.e. fullscale 2048
    uint8_t *src = (uint8_t *)iio_buffer_start(buf) + byte_offset;
    int16_t const *src_ptr = (int16_t *)src;

    if (format == PLUTO_SDR_CS16) {

      ::memcpy(buffs[0], src_ptr, 2 * sizeof(int16_t) * items);

    } else if (format == PLUTO_SDR_CF32) {

      float *dst_cf32 = (float *)buffs[0];

      for (size_t index = 0; index < items * 2; ++index) {
        *dst_cf32 = float(*src_ptr) / 2048.0f;
        src_ptr++;
        dst_cf32++;
      }

    } else if (format == PLUTO_SDR_CS12) {

      int8_t *dst_cs12 = (int8_t *)buffs[0];

      for (size_t index = 0; index < items; ++index) {
        int16_t i = *src_ptr++;
        int16_t q = *src_ptr++;
        // produce 24 bit (iiqIQQ), note the input is LSB aligned, scale=2048
        // note: byte0 = i[7:0]; byte1 = {q[3:0], i[11:8]}; byte2 = q[11:4];
        *dst_cs12++ = uint8_t(i);
        *dst_cs12++ = uint8_t((q << 4) | ((i >> 8) & 0x0f));
        *dst_cs12++ = uint8_t(q >> 4);
      }
    } else if (format == PLUTO_SDR_CS8) {

      int8_t *dst_cs8 = (int8_t *)buffs[0];

      for (size_t index = 0; index < items * 2; index++) {
        *dst_cs8 = int8_t(*src_ptr >> 4);
        src_ptr++;
        dst_cs8++;
      }
    }
  } else {
    int16_t conv = 0, *conv_ptr = &conv;

    for (unsigned int i = 0; i < channel_list.size(); i++) {
      iio_channel *chn = channel_list[i];
      unsigned int index = i / 2;

      uint8_t *src = (uint8_t *)iio_buffer_first(buf, chn) + byte_offset;
      int16_t const *src_ptr = (int16_t *)src;

      if (format == PLUTO_SDR_CS16) {

        int16_t *dst_cs16 = (int16_t *)buffs[index];

        for (size_t j = 0; j < items; ++j) {
          iio_channel_convert(chn, conv_ptr, src_ptr);
          src_ptr += buf_step;
          dst_cs16[j * 2 + i] = conv;
        }
      } else if (format == PLUTO_SDR_CF32) {

        float *dst_cf32 = (float *)buffs[index];

        for (size_t j = 0; j < items; ++j) {
          iio_channel_convert(chn, conv_ptr, src_ptr);
          src_ptr += buf_step;
          dst_cf32[j * 2 + i] = float(conv) / 2048.0f;
        }
      } else if (format == PLUTO_SDR_CS8) {

        int8_t *dst_cs8 = (int8_t *)buffs[index];

        for (size_t j = 0; j < items; ++j) {
          iio_channel_convert(chn, conv_ptr, src_ptr);
          src_ptr += buf_step;
          dst_cs8[j * 2 + i] = int8_t(conv >> 4);
        }
      }
    }
  }

  items_in_buffer -= items;
  byte_offset += items * iio_buffer_step(buf);

  return (items);
}

int rx_streamer::start(const int flags, const long long timeNs,
                       const size_t numElems) {
  // force proper stop before
  stop(flags, timeNs);

  // re-create buffer
  buf = iio_device_create_buffer(dev, buffer_size, false);

  if (!buf) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
    throw std::runtime_error("Unable to create buffer!\n");
  }

  direct_copy = has_direct_copy();

  SoapySDR_logf(SOAPY_SDR_INFO, "Has direct RX copy: %d", (int)direct_copy);

  return 0;
}

int rx_streamer::stop(const int flags, const long long timeNs) {
  // cancel first
  if (buf) {
    iio_buffer_cancel(buf);
  }
  // then destroy
  if (buf) {
    iio_buffer_destroy(buf);
    buf = nullptr;
  }

  items_in_buffer = 0;
  byte_offset = 0;

  return 0;
}

void rx_streamer::set_buffer_size(const size_t _buffer_size) {

  if (!buf || this->buffer_size != _buffer_size) {
    // cancel first
    if (buf) {
      iio_buffer_cancel(buf);
    }
    // then destroy
    if (buf) {
      iio_buffer_destroy(buf);
    }

    items_in_buffer = 0;
    byte_offset = 0;

    buf = iio_device_create_buffer(dev, _buffer_size, false);
    if (!buf) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
      throw std::runtime_error("Unable to create buffer!\n");
    }
  }

  this->buffer_size = _buffer_size;
}

size_t rx_streamer::get_mtu_size() { return this->mtu_size; }

// return wether can we optimize for single RX, 2 channel (I/Q), same endianess
// direct copy
bool rx_streamer::has_direct_copy() {
  if (channel_list.size() != 2) // one RX with I + Q
    return false;

  ptrdiff_t buf_step = iio_buffer_step(buf);

  if (buf_step != 2 * sizeof(int16_t))
    return false;

  if (iio_buffer_start(buf) != iio_buffer_first(buf, channel_list[0]))
    return false;

  int16_t test_dst, test_src = 0x1234;
  iio_channel_convert(channel_list[0], &test_dst, (const void *)&test_src);

  return test_src == test_dst;
}
