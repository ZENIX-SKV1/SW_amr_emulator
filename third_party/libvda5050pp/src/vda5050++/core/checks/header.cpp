//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//
#include "vda5050++/core/checks/header.h"

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ranges.h>

#include <string>

#include "vda5050++/core/instance.h"
#include "vda5050++/version.h"

std::list<vda5050::Error> vda5050pp::core::checks::checkHeader(
    const vda5050::HeaderVDA5050 &header) {
  const auto &desc = vda5050pp::core::Instance::ref().getConfig().getAgvDescription();

  std::list<vda5050::Error> errors;

  if (desc.manufacturer != header.manufacturer) {
    vda5050::Error err;
    err.errorType = "validationError";
    err.errorDescription = fmt::format("Header.manufacturer is \"{}\", requires \"{}\"",
                                       header.manufacturer, desc.manufacturer);
    err.errorReferences = {{{"header.headerId", std::to_string(header.headerId)},
                            {"header.manufacturer", header.manufacturer}}};
    err.errorLevel = vda5050::ErrorLevel::WARNING;
    err.errorHint = "Make sure the manufacturer is correctly configured";
    errors.push_back(std::move(err));
  }

  if (desc.serial_number != header.serialNumber) {
    vda5050::Error err;
    err.errorType = "validationError";
    err.errorDescription = fmt::format("Header.serialNumber is \"{}\", requires \"{}\"",
                                       header.serialNumber, desc.serial_number);
    err.errorReferences = {{{"header.headerId", std::to_string(header.headerId)},
                            {"header.serialNumber", header.serialNumber}}};
    err.errorLevel = vda5050::ErrorLevel::WARNING;
    err.errorHint = "Make sure the serial number is correctly configured";
    errors.push_back(std::move(err));
  }

  // This check could be replaced by a sem-ver based check instead of set member based
  if (auto &c = vda5050pp::version::getCompatible(); c.find(header.version) == c.end()) {
    vda5050::Error err;
    err.errorType = "validationError";
    err.errorDescription =
        fmt::format("Header.version is {}, requires one of {}", header.version, c);
    err.errorReferences = {
        {{"header.headerId", std::to_string(header.headerId)}, {"header.version", header.version}}};
    err.errorLevel = vda5050::ErrorLevel::WARNING;
    err.errorHint = "Make sure to use a compatible version of the VDA5050 protocol";
    errors.push_back(std::move(err));
  }

  return errors;
}
