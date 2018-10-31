//
// Created by Tingcheng Wu on 18-01-15.
// Copyright (c) 2018 Horizon Robotics. All rights reserved.
//
#ifndef EASYHTTP_CLIENT_H_
#define EASYHTTP_CLIENT_H_

#include <string>
#include <memory>

#define HTTP_RETRY_TIMES 3

struct HrBuffer {
  char *buffer;
  size_t buf_len;
};

class EasyHttpClient {
 public:
  static EasyHttpClient *Instance();
  /*
   * @brief HTTP Get请求
   * @param url,请求的URL
   * @param response, 请求的结果
   */
  int Get(const std::string &url, HrBuffer *resp);
  /*
   * @brief HTTP Post请求
   * @param url,请求的URL
   * @param post_content,POST的内容
   * @param response，服务器返回的内容
   */
  int Post(const std::string &url,
           const std::string &post_content,
           HrBuffer *resp,
           long *);  // NOLINT

  int Init();
  ~EasyHttpClient();
};
#endif  // MERCURIUS_INCLUDE_MERCURIUS_HTTP_CLIENT_H_