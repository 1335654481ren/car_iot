//
// Created by Tingcheng Wu on 18-01-15.
// Copyright (c) 2018 Horizon Robotics. All rights reserved.
//
//
#include <string.h>
#include <stdio.h>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <thread>
#include "curl/curl.h"
#include "otacli/easyhttp_client.h"

extern "C" size_t EasyOnWriteData(void *buffer, size_t size, size_t nmemb,void *resp_buf) 
{
  HrBuffer *usr_buf = (HrBuffer *)resp_buf;
  auto copy_len =
      size * nmemb < usr_buf->buf_len ? size * nmemb : usr_buf->buf_len;
  strncpy(usr_buf->buffer, reinterpret_cast<char *>(buffer), copy_len);
  usr_buf->buffer[copy_len] = 0;

  return nmemb;
}

int EasyHttpClient::Get(const std::string &url, HrBuffer *resp) {
  int ret;
  long http_code;  // NOLINT
  int retry = 0;
  curl_slist *headers = nullptr;

  auto curl = curl_easy_init();
  if (!curl) return CURLE_FAILED_INIT;
  headers = curl_slist_append(headers, "Host: ihome.ngrok.xiaomiqiu.cn");
  headers = curl_slist_append(headers, "Connection: keep-alive");
  headers = curl_slist_append(headers, "Authorization: Basic dXNlcjoxMjM0NQ==");
  headers = curl_slist_append(headers, "Accept: application/json");
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers = curl_slist_append(headers, "X-HOBOT-Signature: application/json");

  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
  curl_easy_setopt(curl, CURLOPT_VERBOSE, 0L);  // 0 disable messages
  curl_easy_setopt(curl, CURLOPT_READFUNCTION, nullptr);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, EasyOnWriteData);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, resp);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3);

  while (retry++ < HTTP_RETRY_TIMES) {
    ret = curl_easy_perform(curl);

    if (CURLE_OK == ret &&
        CURLE_OK == curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE,
              &http_code) &&
        200 == http_code) {
      std::cout << "curl get url:" << url << " done." << std::endl;
      ret = 0;
      goto out;
    } else {
      std::cout << "curl get url:" << url << " failed." << std::endl;
      ret = -1;
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }

out:
  curl_easy_cleanup(curl);

  return ret;
}

EasyHttpClient *EasyHttpClient::Instance() {
  static EasyHttpClient hc;
  return &hc;
}

int EasyHttpClient::Init() {
  return curl_global_init(CURL_GLOBAL_DEFAULT);
}

EasyHttpClient::~EasyHttpClient() { curl_global_cleanup(); }

int EasyHttpClient::Post(const std::string &url,
        const std::string &post_content, HrBuffer *resp,
        long *http_code) {  // NOLINT
  if (resp == nullptr || http_code == nullptr) {
    std::cout << "buffer or http code will be stored in nullptr" << std::endl;
    return -1;
  }
  std::cout << post_content << std::endl;
  int ret = 0;
  curl_slist *headers = nullptr;
  int retry = 0;

  auto curl = curl_easy_init();
  if (!curl) return CURLE_FAILED_INIT;
  headers = curl_slist_append(headers, "Host: ihome.ngrok.xiaomiqiu.cn");
  headers = curl_slist_append(headers, "Connection: keep-alive");
  headers = curl_slist_append(headers, "Authorization: Basic dXNlcjoxMjM0NQ==");
  headers = curl_slist_append(headers, "Accept: application/json");
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers = curl_slist_append(headers, "X-HOBOT-Signature: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
  curl_easy_setopt(curl, CURLOPT_POST, 1);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_content.c_str());
  curl_easy_setopt(curl, CURLOPT_READFUNCTION, nullptr);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, EasyOnWriteData);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, resp);
  curl_easy_setopt(curl, CURLOPT_VERBOSE,0);  // Disable verbose debug output.
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 8);

  while (retry++ < HTTP_RETRY_TIMES) {
    ret = curl_easy_perform(curl);

    std::cout << "curl perform result: " << ret << std::endl;
    if (CURLE_OK == ret &&
        CURLE_OK == curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE,
              http_code)) {
      std::cout << "curl post url:" << url
           << ", content:" << post_content << " succeed." << std::endl;
      ret = 0;
      goto out;
    } else {
      std::cout << "curl error" << ret << std::endl;
    }

    std::cout << "curl post url:" << url << ",  content:" << post_content
         << " failed." << std::endl;
    ret = -1;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

out:
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  return ret;
}