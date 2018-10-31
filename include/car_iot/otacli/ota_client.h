//
// Created by guoxing xu on 18-03-09.
// Copyright (c) 2018 Horizon Robotics. All rights reserved.
// OTA Client API

#ifndef OTACLI_INCLUDE_OTACLI_OTA_CLIENT_H_
#define OTACLI_INCLUDE_OTACLI_OTA_CLIENT_H_
#include <string>
#include "ota_typedef.h"

namespace iot {

class OtaClient {
 public:
  OtaClient();
  /*
   * @brief 设置全局的配置，以及做全局的初始化工作
   * @param downLoadDir, 下载临时文件存放路径
   * @param authInfoStoreDir, 认证信息存放路径(公私钥等)
   * @param baseUrl,云端URL地址
   */
  int Init(const std::string &downLoadDir, const std::string &authInfoStoreDir,
           const std::string &baseUrl);

  ~OtaClient();
  /*
  * @brief Auth Device
  * @param id_data, id of device
  * @param tenantToken, tenant token
  * @param newTenantToken, 重新分配的tenantToken,需要覆盖掉老的tenantToken
  */
  int AuthDevice(const std::string &idData, const std::string &tenantToken,
                 std::string &newTenantToken);
  /*
  * @brief 上报设备信息
  * @param requestJson, 信息为业务定义的json字符串
  */
  int ReportDeviceAttrs(const std::string &requestJson);
  /*
  * @brief 上报设备状态
  * @param statusJson, 信息为限定的json字符串
  * status: 状态字符串("installing", "downloading", "rebooting", "success",
  * "failure", "already-installed")
  * @example {"status":"failure"}
  */
  int ReportDeployStatus(const std::string &statusJson);
  /*
  * @brief 上报设备log信息
  * @param logsJson, 信息为限定格式的json字符串
  * @exmaple {"messages": [{"level": "debug", "message":
  *           "debug", "timestamp": "2012-12-23T22:08:41+00:00"}]}
  */
  int ReportDeployLogs(const std::string &logsJson);
  /*
   * @brief 查询升级包地址
   * @param artifactName, 当前已安装的更新包版本名
   * @param deviceType, 设备类型
   * @param updatorPkgUrl,返回的下载地址
   */
  int QueryUpdatorPackage(const std::string &artifactName,
                          const std::string &deviceType);

  /*
  * @brief 下载升级包
  * @param updatorPkgUrl, 地址
  * @param localPath, 本地存放路径
  * @param cb,进度回掉函数
  * @param pParm, 传递给cb的参数
  */
  int DownLoadUpdatorPackage(const std::string &localPath, DownloadCallback *cb,
                             void *pParam);

  /*
  * @brief 查询设备全部属性信息
  * @param responseJson 返回属性的json字符串
  * @example
  * [
  *  {
  *     "name": "config",
  *     "value": "{\"abc\":\"123\"}"
  *  },
  *  {
  *     "name": "regist_id",
  *     "value": "123456789"
  *  },
  *  {
  *    "name": "time",
  *    "value": "12345678"
  *  }
  * ]
  */
  int QueryDeviceFullAttrs(std::string &responseJson);
  /*
  * @brief 查询设备全部属性信息
  * @param responseJson 返回属性的json字符串
  * @exmple
  * [
  *  {
  *   "name": "config",
  *   "value": "{\"abc\":\"123\"}"
  *  }
  * ]
  */
  int QueryDeviceAttrsByAttrName(const std::string &key,
                                 std::string &responseJson);
  /*
  * query version
  */
  int QueryServerVersion();
 private:
  int AuthDevice(const std::string &idData, const std::string &pubKey,
                 const std::string &priKey, const std::string &tenantToken,
                 std::string &authToken, std::string &newTenantToken);

  bool VerifyJsonString(const std::string &jsonString, std::string &errs);

  int GenRsaKeyPair(const std::string &encKey, int keysize);
  int GetRsaKeyPair(const std::string &endKey, std::string &priKey,
                    std::string &pubKey);
  int GetAuthToken();

 private:
  const std::string kBaseUrl = "https://42.62.85.21/api/devices/v1";
  const std::string kVersion = "/version";
  const std::string kAuth = "/authentication/auth_requests";
  const std::string kAttrs = "/inventory/device/attributes";
  const std::string kDeployBaseUrl = "/deployments/device/deployments/";
  const std::string kStatus = "/status";
  const std::string kLogs = "/log";
  const std::string kUpdator = "next";
  const std::string kCaPath = "";
  const std::string kPriKeyPath = "pri.key";
  const std::string kPubKeyPath = "pub.key";
  const std::string kDownloadPath = "/tmp/";
  const std::string kStorePath = "./";
  std::string base_url_;
  std::string auth_token_;
  std::string device_id_;
  std::string download_url_;
  std::string authfile_path_;
  std::string download_path_;
  bool initd_;
};

};      // namespace iot
#endif  // OTACLI_INCLUDE_OTACLI_OTA_CLIENT_H_
