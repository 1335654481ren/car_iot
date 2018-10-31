//
// Created by guoxing xu on 18-03-09.
// Copyright (c) 2018 Horizon Robotics. All rights reserved.
// http get/put/patch/post operations
//
#include "otacli/http_client.h"

#include <string.h>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "otacli/ota_typedef.h"

#define HTTP_RETRY_TIMES 3

extern "C" size_t OnWriteData(void *buffer, size_t size, size_t nmemb,
                              void *resp_buf) {
  std::string *str = (std::string *)resp_buf;
  (*str).append(reinterpret_cast<char *>(buffer), size * nmemb);
  return size * nmemb;
}

extern "C" size_t OnWriteFile(void *buffer, size_t size, size_t nmemb,
                              void *file) {
  if (file) {
    return fwrite(buffer, size, nmemb, (FILE *)file);
  }
  return 0;
}

iot::HttpClient::HttpClient() {
  curl_ = NULL;
  curl_headers_ = NULL;
  proxy_host_ = "";
  proxy_host_ = "";
}

void iot::HttpClient::set_curl_proxy_host(const std::string &proxy_host) {
  proxy_host_ = proxy_host;
}

void iot::HttpClient::set_curl_proxy_auth(const std::string &proxy_auth) {
  proxy_auth_ = proxy_auth;
}

void iot::HttpClient::set_curl_headers(
    const std::vector<std::string> curl_headers) {
  for (size_t i = 0; i < curl_headers.size(); ++i) {
    curl_headers_ = curl_slist_append(curl_headers_, curl_headers[i].c_str());
  }
}

void iot::HttpClient::set_process_cb(DownloadCallback *cb) { process_cb_ = cb; }

void iot::HttpClient::set_process_cb_param(void *p) { process_cb_param_ = p; }

int iot::HttpClient::HttpGet(const std::string &url, std::string &resp,
                             long *statusCode) {
  int ret = 0;
  if ((ret = CreateHttpTransport()) != 0) {
    goto end;
  }

  if ((ret = SetupHttpTransport(url, "", "", resp, HttpMethod::HTTP_GET)) !=
      0) {
    goto end;
  }

  if ((ret = PerformHttpTransport(statusCode)) != 0) {
    goto end;
  }
end:
  CleanupHttpTransport();
  return ret;
}

int iot::HttpClient::HttpPost(const std::string &url,
                              const std::string &postContent, std::string &resp,
                              long *statusCode) {
  int ret = 0;
  if ((ret = CreateHttpTransport()) != 0) {
    goto end;
  }

  if ((ret = SetupHttpTransport(url, postContent, "", resp,
                                HttpMethod::HTTP_POST)) != 0) {
    goto end;
  }

  if ((ret = PerformHttpTransport(statusCode)) != 0) {
    goto end;
  }
end:
  CleanupHttpTransport();
  return ret;
}

int iot::HttpClient::HttpPut(const std::string &url,
                             const std::string &putContent, std::string &resp,
                             long *statusCode) {
  int ret = 0;
  if ((ret = CreateHttpTransport()) != 0) {
    goto end;
  }

  if ((ret = SetupHttpTransport(url, putContent, "", resp,
                                HttpMethod::HTTP_PUT)) != 0) {
    goto end;
  }

  if ((ret = PerformHttpTransport(statusCode)) != 0) {
    goto end;
  }
end:
  CleanupHttpTransport();
  return ret;
}

int iot::HttpClient::HttpPatch(const std::string &url,
                               const std::string &patchContent,
                               std::string &resp, long *statusCode) {
  int ret = 0;
  if ((ret = CreateHttpTransport()) != 0) {
    goto end;
  }

  if ((ret = SetupHttpTransport(url, patchContent, "", resp,
                                HttpMethod::HTTP_PATCH)) != 0) {
    goto end;
  }

  if ((ret = PerformHttpTransport(statusCode)) != 0) {
    goto end;
  }
end:
  CleanupHttpTransport();
  return ret;
}

int iot::HttpClient::HttpsGet(const std::string &url,
                              const std::string &cafilePath, std::string &resp,
                              long *statusCode) {
  int ret = 0;
  if ((ret = CreateHttpTransport()) != 0) {
    goto end;
  }

  if ((ret = SetupHttpTransport(url, "", cafilePath, resp,
                                HttpMethod::HTTP_GET)) != 0) {
    goto end;
  }

  if ((ret = PerformHttpTransport(statusCode)) != 0) {
    goto end;
  }
end:
  CleanupHttpTransport();
  return ret;
}

int iot::HttpClient::HttpsPost(const std::string &url,
                               const std::string &postContent,
                               const std::string &cafilePath, std::string &resp,
                               long *statusCode) {
  int ret = 0;
  if ((ret = CreateHttpTransport()) != 0) {
    goto end;
  }

  if ((ret = SetupHttpTransport(url, postContent, cafilePath, resp,
                                HttpMethod::HTTP_POST)) != 0) {
    goto end;
  }

  if ((ret = PerformHttpTransport(statusCode)) != 0) {
    goto end;
  }
end:
  CleanupHttpTransport();
  return ret;
}

int iot::HttpClient::HttpsPut(const std::string &url,
                              const std::string &putContent,
                              const std::string &cafilePath, std::string &resp,
                              long *statusCode) {
  int ret = 0;
  if ((ret = CreateHttpTransport()) != 0) {
    goto end;
  }

  if ((ret = SetupHttpTransport(url, putContent, cafilePath, resp,
                                HttpMethod::HTTP_PUT)) != 0) {
    goto end;
  }

  if ((ret = PerformHttpTransport(statusCode)) != 0) {
    goto end;
  }
end:
  CleanupHttpTransport();
  return ret;
}

int iot::HttpClient::HttpsPatch(const std::string &url,
                                const std::string &patchContent,
                                const std::string &cafilePath,
                                std::string &resp, long *statusCode) {
  int ret = 0;
  if ((ret = CreateHttpTransport()) != 0) {
    goto end;
  }

  if ((ret = SetupHttpTransport(url, patchContent, cafilePath, resp,
                                HttpMethod::HTTP_PATCH)) != 0) {
    goto end;
  }

  if ((ret = PerformHttpTransport(statusCode)) != 0) {
    goto end;
  }
end:
  CleanupHttpTransport();
  return ret;
}
int iot::HttpClient::HttpDownload(const std::string &url,
                                  const std::string &localPath) {
  if (CreateHttpTransport() != 0) {
    return static_cast<int>(ErrorCode::kInitError);
  }
  FILE *fp = fopen(localPath.c_str(), "ab+");
  if (fp == NULL) {
    return static_cast<int>(ErrorCode::kFileOPError);
  }

  CURLcode code;
#define curl_easy_setopt_safe(curl, opt, val)                  \
  if ((code = curl_easy_setopt(curl, opt, val)) != CURLE_OK) { \
    if (fp) {                                                  \
      fclose(fp);                                              \
    }                                                          \
    return static_cast<int>(iot::ErrorCode::kFailed);          \
  }
  curl_easy_setopt_safe(curl_, CURLOPT_URL, url.c_str());
  curl_easy_setopt_safe(curl_, CURLOPT_READFUNCTION, nullptr);
  curl_easy_setopt_safe(curl_, CURLOPT_WRITEFUNCTION, OnWriteFile);
  curl_easy_setopt_safe(curl_, CURLOPT_WRITEDATA, fp);

  curl_easy_setopt_safe(curl_, CURLOPT_NOPROGRESS, 0);
  curl_easy_setopt_safe(curl_, CURLOPT_PROGRESSFUNCTION, process_cb_);
  curl_easy_setopt_safe(curl_, CURLOPT_PROGRESSDATA, this);

  // request options
  curl_easy_setopt_safe(curl_, CURLOPT_DNS_CACHE_TIMEOUT, 1);
  curl_easy_setopt_safe(curl_, CURLOPT_CONNECTTIMEOUT, 3);
  curl_easy_setopt_safe(curl_, CURLOPT_LOW_SPEED_LIMIT, 1);
  curl_easy_setopt_safe(curl_, CURLOPT_LOW_SPEED_TIME, 20);
#if LIBCURL_VERSION_NUM >= 0x072000
  /*
    New libcurls will prefer the new callback and instead use that one even
    if both callbacks are set.
  */
  curl_easy_setopt_safe(curl_, CURLOPT_XFERINFOFUNCTION, XferinfoCallback);
  curl_easy_setopt_safe(curl_, CURLOPT_XFERINFODATA, this);
#endif
#undef curl_easy_setopt_safe
  long statusCode = 0;
  int ret = 0;
  ret = PerformHttpTransport(&statusCode);
  if (fp) {
    fclose(fp);
  }
  return ret;
}

int iot::HttpClient::ProcessCallback(void *pParam, double dltotal, double dlnow,
                                     double ultotal, double ulone) {
  HttpClient *pthis = (HttpClient *)pParam;
  if (pthis->process_cb_) {
    pthis->process_cb_->Process(pthis->process_cb_param_, dltotal, dlnow);
  }
  return 0;
}
int iot::HttpClient::XferinfoCallback(void *pParam, curl_off_t dltotal,
                                      curl_off_t dlnow, curl_off_t ultotal,
                                      curl_off_t ulnow) {
  HttpClient *pthis = (HttpClient *)pParam;
  if (pthis->process_cb_) {
    pthis->process_cb_->Process(pthis->process_cb_param_,
                                static_cast<double>(dltotal),
                                static_cast<double>(dlnow));
  }
  return 0;
}

int iot::HttpClient::CreateHttpTransport() {
  if (curl_ != NULL) {
    curl_easy_cleanup(curl_);
    curl_ = NULL;
  }
  curl_ = curl_easy_init();
  if (!curl_) {
    return CURLE_FAILED_INIT;
  }
  return static_cast<int>(ErrorCode::kSuccess);
}
int iot::HttpClient::PerformHttpTransport(long *statusCode) {
  int ret = 0;
  int retry = 0;
  while (retry++ < HTTP_RETRY_TIMES) {
    ret = curl_easy_perform(curl_);

    if (CURLE_OK == ret &&
        CURLE_OK ==
            curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, statusCode) &&
        *statusCode >= 200 && *statusCode < 300) {
      std::cout << "done." << *statusCode << std::endl;
      ret = 0;
      break;
    } else if (ret == CURLE_COULDNT_RESOLVE_HOST ||
               ret == CURLE_COULDNT_CONNECT ||
               ret == CURLE_OPERATION_TIMEDOUT) {
      std::cout << "failed. ret:" << ret << std::endl;
      ret = static_cast<int>(ErrorCode::kNetFault);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
      std::cout << "failed. ret:" << ret << *statusCode << std::endl;
      ret = static_cast<int>(ErrorCode::kFailed);
      break;
    }
  }
  return ret;
}
int iot::HttpClient::CleanupHttpTransport() {
  if (curl_ != NULL) {
    curl_easy_cleanup(curl_);
    curl_ = NULL;
  }
  if (curl_headers_) {
    curl_slist_free_all(curl_headers_); /* free the header list again */
    curl_headers_ = NULL;
  }
  return static_cast<int>(ErrorCode::kSuccess);
}

int iot::HttpClient::SetupHttpTransport(const std::string &url,
                                        const std::string &requestData,
                                        const std::string &cafilePath,
                                        std::string &resp, HttpMethod method) {
  CURLcode code;
#define curl_easy_setopt_safe(curl, opt, val)                  \
  if ((code = curl_easy_setopt(curl, opt, val)) != CURLE_OK) { \
    return static_cast<int>(iot::ErrorCode::kInitError);       \
  }
  curl_easy_setopt_safe(curl_, CURLOPT_URL, url.c_str());
  curl_easy_setopt_safe(curl_, CURLOPT_READFUNCTION, nullptr);
  curl_easy_setopt_safe(curl_, CURLOPT_WRITEFUNCTION, OnWriteData);
  curl_easy_setopt_safe(curl_, CURLOPT_WRITEDATA, &resp);
  curl_easy_setopt_safe(curl_, CURLOPT_FILETIME, 1);
  curl_easy_setopt_safe(curl_, CURLOPT_NOSIGNAL, 1);
  curl_easy_setopt_safe(curl_, CURLOPT_NOPROGRESS, 1);

  curl_easy_setopt_safe(curl_, CURLOPT_TCP_NODELAY, 1);
  curl_easy_setopt_safe(curl_, CURLOPT_NETRC, CURL_NETRC_IGNORED);
  curl_easy_setopt_safe(curl_, CURLOPT_VERBOSE, 0);

  // transport options
  curl_easy_setopt_safe(curl_, CURLOPT_SSL_VERIFYPEER, 0);
  if (cafilePath.length() != 0) {
    curl_easy_setopt_safe(curl_, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt_safe(curl_, CURLOPT_SSL_VERIFYHOST, 2L);
    curl_easy_setopt_safe(curl_, CURLOPT_CAPATH, cafilePath.c_str());
  } else {
    curl_easy_setopt_safe(curl_, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt_safe(curl_, CURLOPT_SSL_VERIFYHOST, 0L);
  }

  // request options
  curl_easy_setopt_safe(curl_, CURLOPT_DNS_CACHE_TIMEOUT, 1);
  curl_easy_setopt_safe(curl_, CURLOPT_CONNECTTIMEOUT, 3);
  curl_easy_setopt_safe(curl_, CURLOPT_LOW_SPEED_LIMIT, 1);
  curl_easy_setopt_safe(curl_, CURLOPT_LOW_SPEED_TIME, 20);

  curl_easy_setopt_safe(curl_, CURLOPT_HTTPHEADER, curl_headers_);

  if (proxy_host_.length() != 0) {
    // proxy
    curl_easy_setopt_safe(curl_, CURLOPT_PROXYTYPE, CURLPROXY_HTTP);
    curl_easy_setopt_safe(curl_, CURLOPT_PROXY, proxy_host_.c_str());
    // authorize
    if (proxy_auth_.length() != 0) {
      curl_easy_setopt_safe(curl_, CURLOPT_PROXYAUTH, CURLAUTH_BASIC);
      curl_easy_setopt_safe(curl_, CURLOPT_PROXYUSERPWD, proxy_auth_.c_str());
    }
  }
  switch (method) {
    case HttpMethod::HTTP_HEAD:
      curl_easy_setopt_safe(curl_, CURLOPT_NOBODY, 1);
      break;
    case HttpMethod::HTTP_PUT:
      curl_easy_setopt_safe(curl_, CURLOPT_CUSTOMREQUEST, "PUT");
      curl_easy_setopt_safe(curl_, CURLOPT_POSTFIELDS, requestData.c_str());
      break;
    case HttpMethod::HTTP_POST:
      curl_easy_setopt_safe(curl_, CURLOPT_POST, 1);
      curl_easy_setopt_safe(curl_, CURLOPT_POSTFIELDS, requestData.c_str());
      break;
    case HttpMethod::HTTP_DELETE:
      curl_easy_setopt_safe(curl_, CURLOPT_CUSTOMREQUEST, "DELETE");
      break;
    case HttpMethod::HTTP_PATCH:
      curl_easy_setopt_safe(curl_, CURLOPT_CUSTOMREQUEST, "PATCH");
      curl_easy_setopt_safe(curl_, CURLOPT_POSTFIELDS, requestData.c_str());
      break;
    default:  // HTTP_GET
      break;
  }
#undef curl_easy_setopt_safe
  return static_cast<int>(ErrorCode::kSuccess);
}
