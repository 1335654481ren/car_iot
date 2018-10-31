//
// Created by guoxing xu on 18-03-8.
// Copyright (c) 2018 Horizon Robotics. All rights reserved.
//

#ifndef OTACLI_INCLUDE_OTACLI_HTTP_CLIENT_H_
#define OTACLI_INCLUDE_OTACLI_HTTP_CLIENT_H_

#include <memory>
#include <string>
#include <vector>

#include <curl/curl.h>

#include "ota_typedef.h"

namespace iot {
enum class HttpMethod {
  HTTP_GET = 0,
  HTTP_POST,
  HTTP_PUT,
  HTTP_DELETE,
  HTTP_HEAD,
  HTTP_PATCH,
};

//每次请求创建一个实例
class HttpClient {
 public:
  HttpClient();
  /*
  * @brief 设置http头
  * @param url,请求的URL
  * @param response, 请求的结果
  */
  void set_curl_headers(const std::vector<std::string> curl_headers);
  void set_curl_proxy_host(const std::string &proxy_host);
  void set_curl_proxy_auth(const std::string &proxy_auth);
  void set_process_cb(DownloadCallback *cb);
  void set_process_cb_param(void *p);
  /*
  * @brief HTTP Get请求
  * @param url,请求的URL
  * @param response, 请求的结果
  */
  int HttpGet(const std::string &url, std::string &resp, long *statusCode);

  /*
  * @brief HTTP Post请求
  * @param url,请求的URL
  * @param postCcontent,POST的内容，POST的内容，UTF-8编码
  * @param response，服务器返回的内容
  * @param statusCode, http statu code.
  */
  int HttpPost(const std::string &url, const std::string &postContent,
               std::string &resp,
               long *statusCode);  // NOLINT

  /*
  * @brief HTTP Put请求
  * @param url,请求的URL
  * @param putContent,PUT的内容，UTF-8编码
  * @param response，服务器返回的内容
  * @param statusCode, Http status code.
  */
  int HttpPut(const std::string &url, const std::string &putContent,
              std::string &resp, long *statusCode);
  /*
  * @brief HTTP Patch请求
  * @param url,请求的URL
  * @param patchContent,PUT的内容，UTF-8编码
  * @param response，服务器返回的内容
  * @param statusCode, Http status code.
  */
  int HttpPatch(const std::string &url, const std::string &patchContent,
                std::string &resp, long *statusCode);

  /*
  * @brief HTTPS Post请求
  * @param url,请求的URL
  * @param postContent,POST的内容
  * @param cafilePath,ca证书的绝对路径
  * @param resp,服务器返回的内容,utf-8格式
  * @param statusCode, Http status code.
  */
  int HttpsPost(const std::string &url, const std::string &postContent,
                const std::string &cafilePath, std::string &resp,
                long *statusCode);

  /*
  * @brief HTTPS Get请求
  * @param url,请求的URL
  * @param cafilePath,ca证书的绝对路径
  * @param resp，服务器返回的内容
  * @param statusCode, Http status code.
  */
  int HttpsGet(const std::string &url, const std::string &cafilePath,
               std::string &resp, long *statusCode);

  /*
  * @brief HTTPS Put请求
  * @param url,请求的URL
  * @param putContent,POST的内容
  * @param cafilePath,ca证书的绝对路径
  * @param resp，服务器返回的内容
  * @param statusCode, HTTP返回码
  */
  int HttpsPut(const std::string &url, const std::string &putContent,
               const std::string &cafilePath, std::string &resp,
               long *statusCode);
  /*
   * @brief HTTPS Patch请求
   * @param url,请求的URL
   * @param patchContent,PUT的内容，UTF-8编码
   * @param response，服务器返回的内容
   * @param statusCode, HTTP返回码
   */
  int HttpsPatch(const std::string &url, const std::string &patchContent,
                 const std::string &cafilePath, std::string &resp,
                 long *statusCode);
  /*
  * @brief HTTP Download请求
  * @param url,请求的URL
  * @param localPath, 下载后保存地址
  */
  int HttpDownload(const std::string &url, const std::string &localPath);

  ~HttpClient(){};

 private:
  /* for libcurl older than 7.32.0 (CURLOPT_PROGRESSFUNCTION) */
  static int ProcessCallback(void *pParam, double dltotal, double dlnow,
                             double ultotal, double ulone);

  /* this is how the CURLOPT_XFERINFOFUNCTION callback works */
  static int XferinfoCallback(void *pParam, curl_off_t dltotal,
                              curl_off_t dlnow, curl_off_t ultotal,
                              curl_off_t ulnow);

  int CreateHttpTransport();
  int SetupHttpTransport(const std::string &url, const std::string &requestData,
                         const std::string &cafilePath, std::string &resp,
                         HttpMethod method);
  int PerformHttpTransport(long *statusCode);
  int CleanupHttpTransport();

 private:
  CURL *curl_;
  curl_slist *curl_headers_;
  std::string proxy_host_;
  std::string proxy_auth_;
  void *process_cb_param_;
  DownloadCallback *process_cb_;
};
}  // namespace mercurius
#endif  // OTACLI_INCLUDE_OTACLI_HTTP_CLIENT_H_
