//
// Created by guoxing xu on 2018-03-15.
// Copyright (c) 2018 Horizon Robotics. All rights reserved.
// base APIs.
//
#ifndef OTACLI_INCLUDE_OTACLI_HOBOT_UTILS_H_
#define OTACLI_INCLUDE_OTACLI_HOBOT_UTILS_H_

#include <fstream>
#include <string>

namespace iot {
class HobotUtils {
 public:
  inline static bool FileExists(const std::string &fname) {
    std::ifstream test_stream(fname);
    return test_stream.good();
  }
  static bool DirExists(const std::string &dir);
  static int SignData(const std::string &privateKey, const std::string &encKey,
                      const std::string &rquestData, const std::string &hashAlg,
                      std::string &responseData);

  static int Base64Encode(const unsigned char *input, int length,
                          std::string &output);

  static int GenRsaKeyPair(const std::string &priKeyPath,
                           const std::string &pubKeyPath,
                           const std::string &encKey, unsigned int nbits);

  static int LoadKeyFromFile(const std::string &filePath, std::string &keyStr);

  static int ExtraTarDotGzFile(const std::string &srcFile,
                               const std::string &destDir);
  static int MoveFile(const std::string &srcFile, const std::string &dstFile);
  static int DeleteFile(const std::string &fileName);
  static int DeleteFloder(const std::string &Floder);
  static int GetFileDir(const std::string &filename, std::string &fileDir);
  static int ComputeSHA256(const std::string &fileName, std::string &hash);
  static int GetPureFileName(const std::string &filename,
                             std::string &pureFileName);
};
}  // namespace iot

#endif  // OTACLI_INCLUDE_OTACLI_HOBOT_UTILS_H_
