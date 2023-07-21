#include "SoapyPlutoSDR.hpp"

// libiio-ad9361
#include <ad9361.h>
#include <cstring>

SoapyPlutoSDR::SoapyPlutoSDR(const SoapySDR::Kwargs &args)
    : device_(nullptr), rx_dev_(nullptr), tx_dev_(nullptr), gainMode_(false),
      decimation_(false), interpolation_(false), rx_stream_(nullptr) {

  if (args.count("label") != 0) {
    SoapySDR::logf(SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());
  }

  if (args.count("uri") != 0) {
    // Context from URI
    ctx_ = iio_create_context_from_uri(args.at("uri").c_str());
  } else if (args.count("hostname") != 0) {
    // Network context from hostname
    ctx_ = iio_create_network_context(args.at("hostname").c_str());
  } else {
    // Default context
    ctx_ = iio_create_default_context();
  }

  if (ctx_ == nullptr) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "no device context found.");
    throw std::runtime_error("no device context found");
  }

  // Find devices
  device_ = iio_context_find_device(ctx_, "ad9361-phy");
  rx_dev_ = iio_context_find_device(ctx_, "cf-ad9361-lpc");
  tx_dev_ = iio_context_find_device(ctx_, "cf-ad9361-dds-core-lpc");

  if (device_ == nullptr or rx_dev_ == nullptr or tx_dev_ == nullptr) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "no device found in this context.");
    throw std::runtime_error("no device found in this context");
  }

  // TODO: fix
  setAntenna(SOAPY_SDR_RX, 0, "A_BALANCED");
  setGainMode(SOAPY_SDR_RX, 0, false);
  setAntenna(SOAPY_SDR_TX, 0, "A");
}

SoapyPlutoSDR::~SoapyPlutoSDR(void) {
  // TODO: need to stop devices?

  // Destroy context
  if (ctx_) {
    iio_context_destroy(ctx_);
    ctx_ = nullptr;
  }
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyPlutoSDR::getDriverKey(void) const { return "PlutoSDR"; }

std::string SoapyPlutoSDR::getHardwareKey(void) const { return "ADALM-PLUTO"; }

SoapySDR::Kwargs SoapyPlutoSDR::getHardwareInfo(void) const {
  SoapySDR::Kwargs info;

  unsigned int major = 0;
  unsigned int minor = 0;
  char git_tag[8] = {0};

  // Get library version
  iio_library_get_version(&major, &minor, git_tag);
  char lib_ver[100];
  snprintf(lib_ver, 100, "%u.%u (git tag: %s)", major, minor, git_tag);
  info["library_version"] = lib_ver;

  // Get backend version
  iio_context_get_version(ctx_, &major, &minor, git_tag);
  char backend_ver[100];
  snprintf(backend_ver, 100, "%u.%u (git tag: %s)", major, minor, git_tag);
  info["backend_version"] = backend_ver;

  unsigned int nb_ctx_attrs = iio_context_get_attrs_count(ctx_);
  for (unsigned int i = 0; i < nb_ctx_attrs; i++) {
    const char *key, *value;
    iio_context_get_attr(ctx_, i, &key, &value);
    info[key] = value;
  }

  return info;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyPlutoSDR::getNumChannels(const int dir) const { return 1; }

bool SoapyPlutoSDR::getFullDuplex(const int direction,
                                  const size_t channel) const {
  return true;
}

/*******************************************************************
 * Sensor API
 ******************************************************************/

// bool SoapyPlutoSDR::is_sensor_channel(struct iio_channel *chn) const {
//   return (not iio_channel_is_output(chn) and
//           (iio_channel_find_attr(chn, "raw") or
//            iio_channel_find_attr(chn, "input")));
// }

double SoapyPlutoSDR::double_from_buf(const char *const buf) const {
  double val = 0.0;
  std::istringstream val_as_string(buf);

  // val_as_string.imbue(std::locale::classic()); // ignore global C++ locale
  val_as_string >> val;

  return val;
}

double SoapyPlutoSDR::get_sensor_value(const struct iio_channel *chn) const {

  // TODO: check if this is correct
  char buf[32];
  double val = 0.0;

  if (iio_channel_find_attr(chn, "input")) {
    if (iio_channel_attr_read(chn, "input", buf, sizeof(buf)) > 0) {
      val = double_from_buf(buf);
    }
  } else {
    if (iio_channel_attr_read(chn, "raw", buf, sizeof(buf)) > 0) {
      val = double_from_buf(buf);
    }

    if (iio_channel_find_attr(chn, "offset")) {
      if (iio_channel_attr_read(chn, "offset", buf, sizeof(buf)) > 0) {
        val += double_from_buf(buf);
      }
    }

    if (iio_channel_find_attr(chn, "scale")) {
      if (iio_channel_attr_read(chn, "scale", buf, sizeof(buf)) > 0) {
        val *= double_from_buf(buf);
      }
    }
  }

  return val / 1000.0;
}

std::string SoapyPlutoSDR::id_to_unit(const std::string &id) const {
  const static std::map<std::string, std::string> id_to_unit_table = {
      {"current", "A"},
      {"power", "W"},
      {"temp", "C"},
      {"voltage", "V"},
  };

  for (const auto &it_match : id_to_unit_table) {
    // TODO: use modern api
    // If the id starts with a known prefix, retreive its unit.
    if (id.substr(0, it_match.first.size()) == it_match.first) {
      return it_match.second;
    }
  }

  return "unknown";
}

std::vector<std::string> SoapyPlutoSDR::listSensors(void) const {
  std::vector<std::string> sensors;

  sensors.push_back("xadc_temp0");
  sensors.push_back("xadc_voltage0");
  sensors.push_back("xadc_voltage1");
  sensors.push_back("xadc_voltage2");
  sensors.push_back("xadc_voltage3");
  sensors.push_back("xadc_voltage4");
  sensors.push_back("xadc_voltage5");
  sensors.push_back("xadc_voltage6");
  sensors.push_back("xadc_voltage7");
  sensors.push_back("xadc_voltage8");
  sensors.push_back("adm1177_current0");
  sensors.push_back("adm1177_voltage0");
  sensors.push_back("ad9361-phy_temp0");
  sensors.push_back("ad9361-phy_voltage2");

  return sensors;
}

SoapySDR::ArgInfo SoapyPlutoSDR::getSensorInfo(const std::string &key) const {
  SoapySDR::ArgInfo info;

  std::size_t dash = key.find("_");
  if (dash < std::string::npos) {
    std::string deviceStr = key.substr(0, dash);
    std::string channelStr = key.substr(dash + 1);

    iio_device *dev = iio_context_find_device(ctx_, deviceStr.c_str());
    if (!dev) {
      return info;
    }

    iio_channel *chn = iio_device_find_channel(dev, channelStr.c_str(), false);
    if (!chn) {
      return info;
    }

    const char *name = iio_channel_get_name(chn);
    info.key = key;
    if (name) {
      info.name = name;
    }

    info.type = SoapySDR::ArgInfo::FLOAT;
    info.value = "0.0";
    info.units = id_to_unit(channelStr);
  }

  return info;
}

std::string SoapyPlutoSDR::readSensor(const std::string &key) const {
  std::string sensorValue;

  std::size_t dash = key.find("_");
  if (dash < std::string::npos) {
    std::string deviceStr = key.substr(0, dash);
    std::string channelStr = key.substr(dash + 1);

    iio_device *dev = iio_context_find_device(ctx_, deviceStr.c_str());
    if (!dev) {
      return sensorValue;
    }

    iio_channel *chn = iio_device_find_channel(dev, channelStr.c_str(), false);
    if (!chn) {
      return sensorValue;
    }

    double value = get_sensor_value(chn);
    sensorValue.assign(std::to_string(value));
  }

  return sensorValue;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyPlutoSDR::getSettingInfo(void) const {
  SoapySDR::ArgInfoList setArgs;

  return setArgs;
}

void SoapyPlutoSDR::writeSetting(const std::string &key,
                                 const std::string &value) {}

std::string SoapyPlutoSDR::readSetting(const std::string &key) const {
  std::string info;

  return info;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string>
SoapyPlutoSDR::listAntennas(const int direction, const size_t channel) const {
  std::vector<std::string> options;

  SoapySDR::logf(SOAPY_SDR_INFO, "listAntennas: %d", direction);

  if (direction == SOAPY_SDR_RX) {
    options.push_back("A_BALANCED");
  }

  if (direction == SOAPY_SDR_TX) {
    options.push_back("A");
  }

  return (options);
}

void SoapyPlutoSDR::setAntenna(const int direction, const size_t channel,
                               const std::string &name) {
  if (direction == SOAPY_SDR_RX) {
    // std::lock_guard<std::mutex> lock(rx_device_mutex);
    iio_channel_attr_write(iio_device_find_channel(device_, "voltage0", false),
                           "rf_port_select", name.c_str());
  }

  else if (direction == SOAPY_SDR_TX) {
    // std::lock_guard<std::mutex> lock(tx_device_mutex);
    iio_channel_attr_write(iio_device_find_channel(device_, "voltage0", true),
                           "rf_port_select", name.c_str());
  }
}

std::string SoapyPlutoSDR::getAntenna(const int direction,
                                      const size_t channel) const {
  std::string options;

  if (direction == SOAPY_SDR_RX) {
    options = "A_BALANCED";
  } else if (direction == SOAPY_SDR_TX) {

    options = "A";
  }
  return options;
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyPlutoSDR::hasDCOffsetMode(const int direction,
                                    const size_t channel) const {
  return (false);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyPlutoSDR::listGains(const int direction,
                                                  const size_t channel) const {
  std::vector<std::string> options;
  options.push_back("PGA");
  return (options);
}

bool SoapyPlutoSDR::hasGainMode(const int direction,
                                const size_t channel) const {
  if (direction == SOAPY_SDR_RX)
    return true;
  return false;
}

void SoapyPlutoSDR::setGainMode(const int direction, const size_t channel,
                                const bool automatic) {

  gainMode_ = automatic;
  if (direction == SOAPY_SDR_RX) {
    // std::lock_guard<std::mutex> lock(rx_device_mutex);
    if (gainMode_) {

      iio_channel_attr_write(
          iio_device_find_channel(device_, "voltage0", false),
          "gain_control_mode", "slow_attack");

    } else {

      iio_channel_attr_write(
          iio_device_find_channel(device_, "voltage0", false),
          "gain_control_mode", "manual");
    }
  }
}

bool SoapyPlutoSDR::getGainMode(const int direction,
                                const size_t channel) const {
  return gainMode_;
}

void SoapyPlutoSDR::setGain(const int direction, const size_t channel,
                            const double value) {
  long long gain = (long long)value;
  if (direction == SOAPY_SDR_RX) {
    // std::lock_guard<std::mutex> lock(rx_device_mutex);
    iio_channel_attr_write_longlong(
        iio_device_find_channel(device_, "voltage0", false), "hardwaregain",
        gain);

  } else if (direction == SOAPY_SDR_TX) {
    // std::lock_guard<std::mutex> lock(tx_device_mutex);
    gain = gain - 89;
    iio_channel_attr_write_longlong(
        iio_device_find_channel(device_, "voltage0", true), "hardwaregain",
        gain);
  }
}

void SoapyPlutoSDR::setGain(const int direction, const size_t channel,
                            const std::string &name, const double value) {
  setGain(direction, channel, value);
}

double SoapyPlutoSDR::getGain(const int direction, const size_t channel,
                              const std::string &name) const {
  long long gain = 0;

  if (direction == SOAPY_SDR_RX) {

    // std::lock_guard<std::mutex> lock(rx_device_mutex);

    if (iio_channel_attr_read_longlong(
            iio_device_find_channel(device_, "voltage0", false), "hardwaregain",
            &gain) != 0) {
      return 0;
    }
  } else if (direction == SOAPY_SDR_TX) {

    // std::lock_guard<std::mutex> lock(tx_device_mutex);

    if (iio_channel_attr_read_longlong(
            iio_device_find_channel(device_, "voltage0", true), "hardwaregain",
            &gain) != 0)
      return 0;
    gain = gain + 89;
  }
  return double(gain);
}

SoapySDR::Range SoapyPlutoSDR::getGainRange(const int direction,
                                            const size_t channel,
                                            const std::string &name) const {

  SoapySDR::logf(SOAPY_SDR_INFO, "getGainRange()");

  if (direction == SOAPY_SDR_RX) {
    return (SoapySDR::Range(0, 73));
  } else if (direction == SOAPY_SDR_TX) {
    return (SoapySDR::Range(0, 89));
  }

  return SoapySDR::Range();
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyPlutoSDR::setFrequency(const int direction, const size_t channel,
                                 const std::string &name,
                                 const double frequency,
                                 const SoapySDR::Kwargs &args) {

  // Use rounding.
  long long freq = static_cast<long long>(round(frequency));
  int ret = 0;

  // TODO: fix tx/rx
  SoapySDR::logf(SOAPY_SDR_INFO, "setFrequency() %lld", freq);

  if (direction == SOAPY_SDR_RX) {
    // Set RX LO frequency
    const auto chan = iio_device_find_channel(device_, "altvoltage0", true);
    ret = iio_channel_attr_write_longlong(chan, "frequency", freq);
    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "setFrequency() failed %d", ret);
    }
    // TODO: check result
  } else if (direction == SOAPY_SDR_TX) {
    // Set TX LO frequency
    const auto chan = iio_device_find_channel(device_, "altvoltage1", true);
    ret = iio_channel_attr_write_longlong(chan, "frequency", freq);
    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "setFrequency() failed %d", ret);
    }
  }
}

double SoapyPlutoSDR::getFrequency(const int direction, const size_t channel,
                                   const std::string &name) const {
  long long freq = 0;
  int ret = 0;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "getFrequency()");

  if (direction == SOAPY_SDR_RX) {
    const auto chan = iio_device_find_channel(device_, "altvoltage0", true);
    ret = iio_channel_attr_read_longlong(chan, "frequency", &freq);

    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "getFrequency() failed");
      freq = 0;
    }
  } else if (direction == SOAPY_SDR_TX) {
    const auto chan = iio_device_find_channel(device_, "altvoltage1", true);
    ret = iio_channel_attr_read_longlong(chan, "frequency", &freq);

    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "getFrequency() failed");
      freq = 0;
    }
  }

  return static_cast<double>(freq);
}

SoapySDR::ArgInfoList
SoapyPlutoSDR::getFrequencyArgsInfo(const int direction,
                                    const size_t channel) const {

  SoapySDR::ArgInfoList freqArgs;

  return freqArgs;
}

std::vector<std::string>
SoapyPlutoSDR::listFrequencies(const int direction,
                               const size_t channel) const {
  std::vector<std::string> names;

  names.push_back("RF");

  return (names);
}

SoapySDR::RangeList
SoapyPlutoSDR::getFrequencyRange(const int direction, const size_t channel,
                                 const std::string &name) const {
  return (SoapySDR::RangeList(1, SoapySDR::Range(70000000, 6000000000ull)));
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/
void SoapyPlutoSDR::setSampleRate(const int direction, const size_t channel,
                                  const double rate) {

  int ret = 0;
  long long samplerate = static_cast<long long>(rate);

  SoapySDR::logf(SOAPY_SDR_INFO, "setSampleRate() %lld", samplerate);

  /*
  NOTE:
  In the minimum ADC rate is 25MSPS. Baseband rates below 2.083 MSPS
  (25MSPS/12) require FIR decimation/interpolation to be set. In other words
  the FIR filter needs to be configured and enabled. The minimum baseband rate
  with the FIR filter (decimate by 4) enabled is: 25MSPS /(4*12) = 520.83
  kSPS.
  */

  // This will set up decimation/interpolation and filters automatically.
  ret = ad9361_set_bb_rate(device_, samplerate);
  if (ret < 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "setSampleRate() failed %d", ret);
  }

  // TODO: fix this
  if (rx_stream_) {
    rx_stream_->set_buffer_size_by_samplerate(samplerate);
  }

  return;
}

double SoapyPlutoSDR::getSampleRate(const int direction,
                                    const size_t channel) const {
  long long samplerate = 0;
  int ret = 0;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "getSampleRate()");

  if (direction == SOAPY_SDR_RX) {
    // RX
    const auto chan = iio_device_find_channel(rx_dev_, "voltage0", false);
    ret =
        iio_channel_attr_read_longlong(chan, "sampling_frequency", &samplerate);
    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "getSampleRate() failed %d", ret);
      samplerate = 0;
    }
  } else if (direction == SOAPY_SDR_TX) {
    // TX
    const auto chan = iio_device_find_channel(tx_dev_, "voltage0", true);
    ret =
        iio_channel_attr_read_longlong(chan, "sampling_frequency", &samplerate);
    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "getSampleRate() failed %d", ret);
      samplerate = 0;
    }
  }

  SoapySDR::logf(SOAPY_SDR_DEBUG, "getSampleRate() %lld", samplerate);

  return static_cast<double>(samplerate);
}

std::vector<double> SoapyPlutoSDR::listSampleRates(const int direction,
                                                   const size_t channel) const {
  std::vector<double> options;

  options.push_back(65105); // 25M/48/8+1
  options.push_back(1e6);
  options.push_back(2e6);
  options.push_back(3e6);
  options.push_back(4e6);
  options.push_back(5e6);
  options.push_back(6e6);
  options.push_back(7e6);
  options.push_back(8e6);
  options.push_back(9e6);
  options.push_back(10e6);

  return (options);
}

SoapySDR::RangeList
SoapyPlutoSDR::getSampleRateRange(const int direction,
                                  const size_t channel) const {
  SoapySDR::RangeList results;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "getSampleRateRange(%d, %d)", direction,
                 channel);

  // TODO: double check values
  results.push_back(SoapySDR::Range(25e6 / 384, 61440000));

  return results;
}

void SoapyPlutoSDR::setBandwidth(const int direction, const size_t channel,
                                 const double bw) {

  long long bandwidth = 0;
  int ret = 0;

  SoapySDR::logf(SOAPY_SDR_DEBUG, "setBandwidth(%d, %d, %f)", direction,
                 channel, bw);

  // 0 means automatic bandwidth selection.
  if (bw == 0) {
    // TODO: make factor constant
    bandwidth =
        static_cast<long long>(round(0.75 * getSampleRate(direction, channel)));
  } else {
    bandwidth = static_cast<long long>(round(bw));
  }

  // Set bandwidth
  if (direction == SOAPY_SDR_RX) {
    const auto chan = iio_device_find_channel(device_, "voltage0", false);
    ret = iio_channel_attr_write_longlong(chan, "rf_bandwidth", bandwidth);
    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "setBandwidth(%d, %d, %f) failed: %s",
                     direction, channel, bw, strerror(-ret));
    }
  } else if (direction == SOAPY_SDR_TX) {

    const auto chan = iio_device_find_channel(device_, "voltage0", true);
    ret = iio_channel_attr_write_longlong(chan, "rf_bandwidth", bandwidth);
    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "setBandwidth(%d, %d, %f) failed: %s",
                     direction, channel, bw, strerror(-ret));
    }
  }
}

double SoapyPlutoSDR::getBandwidth(const int direction,
                                   const size_t channel) const {

  SoapySDR::logf(SOAPY_SDR_DEBUG, "getBandwidth(%d, %d)", direction, channel);

  int ret = 0;
  long long bandwidth = 0;

  if (direction == SOAPY_SDR_RX) {
    // RX
    const auto chan = iio_device_find_channel(device_, "voltage0", false);
    ret = iio_channel_attr_read_longlong(chan, "rf_bandwidth", &bandwidth);
    if (ret != 0) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "getBandwidth(%d, %d) failed: %s",
                     direction, channel, strerror(-ret));
      return 0;
    }

  } else if (direction == SOAPY_SDR_TX) {
    // TX
    const auto chan = iio_device_find_channel(device_, "voltage0", true);
    ret = iio_channel_attr_read_longlong(chan, "rf_bandwidth", &bandwidth);
    if (ret != 0) {
      return 0;
    }
  }

  return static_cast<double>(bandwidth);
}

std::vector<double> SoapyPlutoSDR::listBandwidths(const int direction,
                                                  const size_t channel) const {
  std::vector<double> options;

  options.push_back(0.2e6);
  options.push_back(1e6);
  options.push_back(2e6);
  options.push_back(3e6);
  options.push_back(4e6);
  options.push_back(5e6);
  options.push_back(6e6);
  options.push_back(7e6);
  options.push_back(8e6);
  options.push_back(9e6);
  options.push_back(10e6);

  return (options);
}
