#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>
#include <atomic>
#include <chrono>
#include <iio.h>
#include <memory>
#include <vector>

#include "rx_streamer.hpp"
#include "stream_format.hpp"
#include "tx_streamer.hpp"

/* SoapySDR::Device subclass */
class SoapyPlutoSDR : public SoapySDR::Device {
public:
  SoapyPlutoSDR(const SoapySDR::Kwargs &args);
  ~SoapyPlutoSDR();

  /*******************************************************************
   * Identification API
   ******************************************************************/

  std::string getDriverKey(void) const override;

  std::string getHardwareKey(void) const override;

  SoapySDR::Kwargs getHardwareInfo(void) const override;

  /*******************************************************************
   * Channels API
   ******************************************************************/

  size_t getNumChannels(const int) const override;

  bool getFullDuplex(const int direction, const size_t channel) const override;

  /*******************************************************************
   * Stream API
   ******************************************************************/

  std::vector<std::string>
  getStreamFormats(const int direction, const size_t channel) const override;

  std::string getNativeStreamFormat(const int direction, const size_t channel,
                                    double &fullScale) const override;

  SoapySDR::ArgInfoList getStreamArgsInfo(const int direction,
                                          const size_t channel) const override;

  SoapySDR::Stream *
  setupStream(const int direction, const std::string &format,
              const std::vector<size_t> &channels = std::vector<size_t>(),
              const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

  void closeStream(SoapySDR::Stream *stream) override;

  size_t getStreamMTU(SoapySDR::Stream *stream) const override;

  int activateStream(SoapySDR::Stream *stream, const int flags = 0,
                     const long long timeNs = 0,
                     const size_t numElems = 0) override;

  int deactivateStream(SoapySDR::Stream *stream, const int flags = 0,
                       const long long timeNs = 0) override;

  int readStream(SoapySDR::Stream *stream, void *const *buffs,
                 const size_t numElems, int &flags, long long &timeNs,
                 const long timeoutUs = 100000) override;

  int writeStream(SoapySDR::Stream *stream, const void *const *buffs,
                  const size_t numElems, int &flags, const long long timeNs = 0,
                  const long timeoutUs = 100000) override;

  int readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags,
                       long long &timeNs, const long timeoutUs) override;

  /*******************************************************************
   * Sensor API
   ******************************************************************/

  std::vector<std::string> listSensors(void) const override;

  SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override;

  std::string readSensor(const std::string &key) const override;

  /*******************************************************************
   * Settings API
   ******************************************************************/

  SoapySDR::ArgInfoList getSettingInfo(void) const override;

  void writeSetting(const std::string &key, const std::string &value) override;

  std::string readSetting(const std::string &key) const override;

  /*******************************************************************
   * Antenna API
   ******************************************************************/

  std::vector<std::string> listAntennas(const int direction,
                                        const size_t channel) const override;

  void setAntenna(const int direction, const size_t channel,
                  const std::string &name) override;

  std::string getAntenna(const int direction, const size_t channel) const;

  /*******************************************************************
   * Frontend corrections API
   ******************************************************************/

  bool hasDCOffsetMode(const int direction, const size_t channel) const override;

  /*******************************************************************
   * Gain API
   ******************************************************************/

  std::vector<std::string> listGains(const int direction,
                                     const size_t channel) const override;

  bool hasGainMode(const int direction, const size_t channel) const override;

  void setGainMode(const int direction, const size_t channel,
                   const bool automatic) override;

  bool getGainMode(const int direction, const size_t channel) const override;

  void setGain(const int direction, const size_t channel,
               const double value) override;

  void setGain(const int direction, const size_t channel,
               const std::string &name, const double value) override;

  double getGain(const int direction, const size_t channel,
                 const std::string &name) const override;

  SoapySDR::Range getGainRange(const int direction, const size_t channel,
                               const std::string &name) const override;

  /*******************************************************************
   * Frequency API
   ******************************************************************/

  void setFrequency(const int direction, const size_t channel,
                    const std::string &name, const double frequency,
                    const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;

  double getFrequency(const int direction, const size_t channel,
                      const std::string &name) const override;

  SoapySDR::ArgInfoList
  getFrequencyArgsInfo(const int direction,
                       const size_t channel) const override;

  std::vector<std::string> listFrequencies(const int direction,
                                           const size_t channel) const override;

  SoapySDR::RangeList getFrequencyRange(const int direction,
                                        const size_t channel,
                                        const std::string &name) const override;

  /*******************************************************************
   * Sample Rate API
   ******************************************************************/

  void setSampleRate(const int direction, const size_t channel,
                     const double rate) override;

  double getSampleRate(const int direction,
                       const size_t channel) const override;

  std::vector<double> listSampleRates(const int direction,
                                      const size_t channel) const override;

  void setBandwidth(const int direction, const size_t channel,
                    const double bw) override;

  double getBandwidth(const int direction, const size_t channel) const override;

  std::vector<double> listBandwidths(const int direction,
                                     const size_t channel) const override;

  SoapySDR::RangeList getSampleRateRange(const int direction,
                                         const size_t channel) const override;

private:


  bool IsValidRxStreamHandle(SoapySDR::Stream *handle) const;
  bool IsValidTxStreamHandle(SoapySDR::Stream *handle) const;

  //bool is_sensor_channel(struct iio_channel *chn) const;
  double double_from_buf(const char *const buf) const;
  double get_sensor_value(const struct iio_channel *chn) const;
  std::string id_to_unit(const std::string &id) const;

  iio_context *ctx_;
  iio_device *device_;
  iio_device *rx_dev_;
  iio_device *tx_dev_;

  bool gainMode_;

  bool decimation_;
  bool interpolation_;

  std::unique_ptr<rx_streamer> rx_stream_;
  std::unique_ptr<tx_streamer> tx_stream_;
};
