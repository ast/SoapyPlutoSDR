#include "tx_streamer.hpp"

#include <SoapySDR/Types.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Device.hpp>
#include <stdexcept>
#include <cstring>

tx_streamer::tx_streamer(const iio_device *_dev,
                         const plutosdrStreamFormat _format,
                         const std::vector<size_t> &channels,
                         const SoapySDR::Kwargs &args)
    : dev(_dev), format(_format), buf(nullptr) {

  if (dev == nullptr) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-dds-core-lpc not found!");
    throw std::runtime_error("cf-ad9361-dds-core-lpc not found!");
  }

  unsigned int nb_channels = iio_device_get_channels_count(dev), i;
  for (i = 0; i < nb_channels; i++)
    iio_channel_disable(iio_device_get_channel(dev, i));

  // default to channel 0, if none were specified
  const std::vector<size_t> &channelIDs =
      channels.empty() ? std::vector<size_t>{0} : channels;

  for (i = 0; i < channelIDs.size() * 2; i++) {
    iio_channel *chn = iio_device_get_channel(dev, i);
    iio_channel_enable(chn);
    channel_list.push_back(chn);
  }

  buf_size = 4096;
  items_in_buf = 0;
  buf = iio_device_create_buffer(dev, buf_size, false);
  if (!buf) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
    throw std::runtime_error("Unable to create buffer!");
  }

  direct_copy = has_direct_copy();

  SoapySDR_logf(SOAPY_SDR_INFO, "Has direct TX copy: %d", (int)direct_copy);
}

tx_streamer::~tx_streamer() {

  if (buf) {
    iio_buffer_destroy(buf);
  }

  for (unsigned int i = 0; i < channel_list.size(); ++i)
    iio_channel_disable(channel_list[i]);
}

int tx_streamer::send(const void *const *buffs, const size_t numElems,
                      int &flags, const long long timeNs, const long timeoutUs)

{
  if (!buf) {
    return 0;
  }

  size_t items = std::min(buf_size - items_in_buf, numElems);

  int16_t src = 0;
  int16_t const *src_ptr = &src;
  ptrdiff_t buf_step = iio_buffer_step(buf);

  if (direct_copy && format == PLUTO_SDR_CS16) {
    // optimize for single TX, 2 channel (I/Q), same endianess direct copy
    int16_t *dst_ptr = (int16_t *)iio_buffer_start(buf) + items_in_buf * 2;
    memcpy(dst_ptr, buffs[0], 2 * sizeof(int16_t) * items);
  } else if (direct_copy && format == PLUTO_SDR_CS12) {

    int16_t *dst_ptr = (int16_t *)iio_buffer_start(buf) + items_in_buf * 2;
    uint8_t const *samples_cs12 = (uint8_t *)buffs[0];

    for (size_t index = 0; index < items; ++index) {
      // consume 24 bit (iiqIQQ)
      uint16_t src0 = uint16_t(*(samples_cs12++));
      uint16_t src1 = uint16_t(*(samples_cs12++));
      uint16_t src2 = uint16_t(*(samples_cs12++));
      // produce 2x 16 bit, note the output is MSB aligned, scale=32768
      // note: byte0 = i[11:4]; byte1 = {q[7:4], i[15:12]}; byte2 = q[15:8];
      *dst_ptr = int16_t((src1 << 12) | (src0 << 4));
      dst_ptr++;
      *dst_ptr = int16_t((src2 << 8) | (src1 & 0xf0));
      dst_ptr++;
    }
  } else if (direct_copy && format == PLUTO_SDR_CS8) {

    int16_t *dst_ptr = (int16_t *)iio_buffer_start(buf) + items_in_buf * 2;
    int8_t const *samples_cs8 = (int8_t *)buffs[0];

    for (size_t index = 0; index < items * 2; ++index) {
      // consume (2x) 8bit (IQ)
      // produce (2x) 16 bit, note the output is MSB aligned, scale=32768
      *dst_ptr = int16_t(*samples_cs8) << 8;
      samples_cs8++;
      dst_ptr++;
    }
  } else if (format == PLUTO_SDR_CS12) {
    SoapySDR_logf(SOAPY_SDR_ERROR,
                  "CS12 not available with this endianess or channel layout");
    throw std::runtime_error(
        "CS12 not available with this endianess or channel layout");
  } else

    for (unsigned int k = 0; k < channel_list.size(); k++) {
      iio_channel *chn = channel_list[k];
      unsigned int index = k / 2;

      uint8_t *dst_ptr =
          (uint8_t *)iio_buffer_first(buf, chn) + items_in_buf * buf_step;

      // note that TX expects samples MSB aligned, unlike RX which is LSB
      // aligned
      if (format == PLUTO_SDR_CS16) {

        int16_t *samples_cs16 = (int16_t *)buffs[index];

        for (size_t j = 0; j < items; ++j) {
          src = samples_cs16[j * 2 + k];
          iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
          dst_ptr += buf_step;
        }
      } else if (format == PLUTO_SDR_CF32) {

        float *samples_cf32 = (float *)buffs[index];

        for (size_t j = 0; j < items; ++j) {
          src = (int16_t)(samples_cf32[j * 2 + k] *
                          32767.999f); // 32767.999f (0x46ffffff) will ensure
                                       // better distribution
          iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
          dst_ptr += buf_step;
        }
      } else if (format == PLUTO_SDR_CS8) {

        int8_t *samples_cs8 = (int8_t *)buffs[index];

        for (size_t j = 0; j < items; ++j) {
          src = (int16_t)(samples_cs8[j * 2 + k] << 8);
          iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
          dst_ptr += buf_step;
        }
      }
    }

  items_in_buf += items;

  if (items_in_buf == buf_size ||
      (flags & SOAPY_SDR_END_BURST && numElems == items)) {
    int ret = send_buf();

    if (ret < 0) {
      return SOAPY_SDR_ERROR;
    }

    if ((size_t)ret != buf_size) {
      return SOAPY_SDR_ERROR;
    }
  }

  return items;
}

int tx_streamer::flush() { return send_buf(); }

int tx_streamer::send_buf() {
  if (!buf) {
    return 0;
  }

  if (items_in_buf > 0) {
    if (items_in_buf < buf_size) {
      ptrdiff_t buf_step = iio_buffer_step(buf);
      uint8_t *buf_ptr =
          (uint8_t *)iio_buffer_start(buf) + items_in_buf * buf_step;
      uint8_t *buf_end = (uint8_t *)iio_buffer_end(buf);

      memset(buf_ptr, 0, buf_end - buf_ptr);
    }

    ssize_t ret = iio_buffer_push(buf);
    items_in_buf = 0;

    if (ret < 0) {
      return ret;
    }

    return int(ret / iio_buffer_step(buf));
  }

  return 0;
}

// return wether can we optimize for single TX, 2 channel (I/Q), same endianess
// direct copy
bool tx_streamer::has_direct_copy() {

  if (channel_list.size() != 2) // one TX with I/Q
    return false;

  ptrdiff_t buf_step = iio_buffer_step(buf);

  if (buf_step != 2 * sizeof(int16_t))
    return false;

  if (iio_buffer_start(buf) != iio_buffer_first(buf, channel_list[0]))
    return false;

  int16_t test_dst, test_src = 0x1234;
  iio_channel_convert_inverse(channel_list[0], &test_dst,
                              (const void *)&test_src);

  return test_src == test_dst;
}
