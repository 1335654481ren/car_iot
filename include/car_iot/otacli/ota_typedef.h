//
// Created by guoxing xu on 18-03-09.
// Copyright (c) 2018 Horizon Robotics. All rights reserved.
//

#ifndef OTACLI_INCLUDE_OTACLI_OTA_TYPEDEF_H_
#define OTACLI_INCLUDE_OTACLI_OTA_TYPEDEF_H_
#include <string>

namespace iot {

class DownloadCallback {
 public:
  virtual void Process(void *p, double dTotal, double dLoaded){};
};

struct HrBuffer {
  char *buffer;
  size_t buf_len;
};

enum class ErrorCode {
  kSuccess = 0,
  kFailed,
  kInitError,
  kMemError,
  kNetFault,
  kFileOPError,
  kParamError,
  kTimeout,
  kAuthError,
  kSystemError,
  kJsonFormatError,
  kPropertyMissing,
  kServerError,
  kUnauthorized,
  kNoContent,
};
};      // namespace iot
#endif  // OTACLI_INCLUDE_OTACLI_OTA_TYPEDEF_H_
