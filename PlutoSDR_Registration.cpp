#include "SoapyPlutoSDR.hpp"

#include <SoapySDR/Registry.hpp>
#include <chrono>
#include <sstream>
#include <string.h>
#include <thread>

#include <iio.h>

static std::vector<SoapySDR::Kwargs>
find_PlutoSDR(const SoapySDR::Kwargs &args) {

  std::vector<SoapySDR::Kwargs> results;

  ssize_t ret = 0;

  iio_context *ctx = nullptr;
  iio_scan_context *scan_ctx = nullptr;
  iio_context_info **info;

  SoapySDR::Kwargs options;

  // Set log level to debug to see backend scanning
  SoapySDR::setLogLevel(SOAPY_SDR_DEBUG);
  SoapySDR::logf(SOAPY_SDR_DEBUG, "Scanning for devices...");

  scan_ctx = iio_create_scan_context("local,usb=0456:b673,ip", 0);
  if (scan_ctx == nullptr) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "iio_create_scan_context error (%s)",
                   strerror(errno));
  }

  // Enumerate available context.
  ret = iio_scan_context_get_info_list(scan_ctx, &info);
  if (ret < 0) {
    SoapySDR::logf(SOAPY_SDR_ERROR, "iio_scan_context_get_info_list error (%d)",
                   ret);
  }

  const auto num_contexts = ret;
  // Iterate through available context.
  for (int i = 0; i < num_contexts; ++i) {
    const char *desc = iio_context_info_get_description(info[i]);
    if (desc == nullptr) {
      SoapySDR::logf(SOAPY_SDR_ERROR,
                     "iio_context_info_get_description error (%d)", ret);
      continue;
    }

    // Log find
    SoapySDR::logf(SOAPY_SDR_DEBUG, "Found device: %s", desc);

    options["device"] = "PlutoSDR";

    // Get device URI
    const auto uri = iio_context_info_get_uri(info[i]);
    SoapySDR::logf(SOAPY_SDR_DEBUG, "URI: %s", uri);

    options["uri"] = uri;

    // Create context to get additional device parameters.
    auto context = iio_create_context_from_uri(uri);
    if (context == nullptr) {
      SoapySDR::logf(SOAPY_SDR_ERROR, "iio_create_context_from_uri error (%s)",
                     strerror(errno));
      continue;
    }

    // Get device name
    const char *name = iio_context_get_name(context);
    SoapySDR::logf(SOAPY_SDR_DEBUG, "Name: %s", name);

    // Verify it's a PlutoSDR
    const iio_device *dev = iio_context_find_device(context, "ad9361-phy");
    const iio_device *rx_dev =
        iio_context_find_device(context, "cf-ad9361-lpc");
    iio_device *tx_dev =
        iio_context_find_device(context, "cf-ad9361-dds-core-lpc");

    if (dev and rx_dev and tx_dev) {
      SoapySDR::logf(SOAPY_SDR_DEBUG, "Found PlutoSDR");
    } else {
      SoapySDR::logf(SOAPY_SDR_DEBUG, "Not a PlutoSDR");
      iio_context_destroy(context);
      continue;
    }

    std::ostringstream label;
    label << "PlutoSDR (" << name << ")";
    options["label"] = label.str();

    // Destroy context
    iio_context_destroy(context);

    // Store
    results.push_back(options);
  }

  // Free info list
  iio_context_info_list_free(info);
  iio_scan_context_destroy(scan_ctx);

  return results;
}

// Create device instance
static SoapySDR::Device *make_PlutoSDR(const SoapySDR::Kwargs &args) {
  return new SoapyPlutoSDR(args);
}

// Register device
static SoapySDR::Registry register_plutosdr("plutosdr", &find_PlutoSDR,
                                            &make_PlutoSDR,
                                            SOAPY_SDR_ABI_VERSION);
