//
// Created by guoxing xu on 18-03-09.
// Copyright (c) 2018 Horizon Robotics. All rights reserved.
// basic APIs.
//
#include "otacli/hobot_utils.h"
#include <iostream>
#include <string>
#include <fcntl.h>
#include <libtar.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <zlib.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <stdint.h>
#include <sys/param.h>
#include <sys/types.h>

#include "openssl/bio.h"
#include "openssl/err.h"
#include "openssl/evp.h"
#include "openssl/md5.h"
#include "openssl/pem.h"
#include "openssl/rsa.h"
#include "otacli/ota_typedef.h"

const static int kMaxSignLen = 1024;
const static int kMaxErrLen = 1024;
bool iot::HobotUtils::DirExists(const std::string &dir) {
  DIR *d = opendir(dir.c_str());
  if (d == NULL) {
    return false;
  }
  closedir(d);
  return true;
}

int iot::HobotUtils::SignData(const std::string &privateKey,
                              const std::string &encKey,
                              const std::string &rquestData,
                              const std::string &hashAlg,
                              std::string &responseData) {
  int mdType = 0;
  if (hashAlg.compare("SHA1") == 0) {
    mdType = NID_sha1;
  } else {
    mdType = NID_sha256;
  }

  RSA *rsa = NULL;
  BIO *keyBio = NULL;

  // load key
  keyBio = BIO_new_mem_buf(privateKey.c_str(), -1);
  if (keyBio == NULL) {
    return static_cast<int>(ErrorCode::kMemError);
  }

  rsa = PEM_read_bio_RSAPrivateKey(keyBio, &rsa, NULL,
                                   const_cast<char *>(encKey.c_str()));
  if (rsa == NULL) {
    BIO_free(keyBio);
    keyBio = NULL;
    return static_cast<int>(ErrorCode::kFailed);
  }

  unsigned char sigret[kMaxSignLen];
  unsigned int siglen = 0;
  unsigned char md[SHA256_DIGEST_LENGTH];
  unsigned int mdLen = 0;
  if (mdType == NID_sha256) {
    SHA256((const unsigned char *)rquestData.c_str(), rquestData.length(), md);
    mdLen = SHA256_DIGEST_LENGTH;
  } else {
    SHA1((const unsigned char *)rquestData.c_str(), rquestData.length(), md);
    mdLen = SHA_DIGEST_LENGTH;
  }
  // sign
  int ret = RSA_sign(mdType, md, mdLen, sigret, &siglen, rsa);
  if (ret != 1) {
    BIO_free(keyBio);
    keyBio = NULL;
    RSA_free(rsa);
    rsa = NULL;
    char szErrMsg[kMaxErrLen] = {0};
    unsigned long ulErr = ERR_get_error();
    ERR_error_string_n(ulErr, szErrMsg, kMaxErrLen);
    std::cout << "RSA_sign failed " << ulErr << ":" << szErrMsg << "\n";
    return static_cast<int>(ErrorCode::kFailed);
  }

  BIO_free(keyBio);
  keyBio = NULL;
  RSA_free(rsa);
  rsa = NULL;
  // base64
  return iot::HobotUtils::Base64Encode(sigret, siglen, responseData);
}

int iot::HobotUtils::Base64Encode(const unsigned char *input, int length,
                                  std::string &output) {
  BIO *bmem = NULL;
  BIO *b64 = NULL;
  BUF_MEM *bptr = NULL;
  b64 = BIO_new(BIO_f_base64());
  if (b64 == NULL) {
    return static_cast<int>(ErrorCode::kFailed);
  }

  BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);

  bmem = BIO_new(BIO_s_mem());
  if (bmem == NULL) {
    return static_cast<int>(ErrorCode::kMemError);
  }
  b64 = BIO_push(b64, bmem);
  BIO_write(b64, input, length);
  (void)BIO_flush(b64);
  BIO_get_mem_ptr(b64, &bptr);
  if (bptr == NULL) {
    std::cout << "Base64Encode Failed\n";
    BIO_free_all(b64);
    return static_cast<int>(ErrorCode::kFailed);
  }
  output.assign(bptr->data, bptr->length);
  output.push_back('\0');
  BIO_free_all(b64);
  return static_cast<int>(ErrorCode::kSuccess);
}

int iot::HobotUtils::GenRsaKeyPair(const std::string &priKeyPath,
                                   const std::string &pubKeyPath,
                                   const std::string &encKey,
                                   unsigned int nbits) {
  int ret = 0;
  BIO *out = NULL;
  RSA *r = NULL;
  if (nbits != 1024 && nbits != 2048 && nbits != 4096) {
    std::cout << "keysize not support. keysize = " << nbits << std::endl;
    return static_cast<int>(ErrorCode::kParamError);
  }
  unsigned long e = RSA_F4;
  BIGNUM *bne = BN_new();
  if (bne == NULL || BN_set_word(bne, e) != 1) {
    std::cout << "Gen E failed.\n";
    return static_cast<int>(ErrorCode::kMemError);
  }

  r = RSA_new();
  if (r == NULL) {
    std::cout << "New Memory failed." << std::endl;
    BN_free(bne);
    return static_cast<int>(ErrorCode::kFailed);
  }
  if (RSA_generate_key_ex(r, nbits, bne, NULL) != 1) {
    RSA_free(r);
    BN_free(bne);
    std::cout << "Gen keyPair failed." << std::endl;
    return static_cast<int>(ErrorCode::kFailed);
  }

  out = BIO_new_file(priKeyPath.c_str(), "w");
  if (out == NULL) {
    std::cout << "New Memory failed." << priKeyPath << std::endl;
    RSA_free(r);
    BN_free(bne);
    return static_cast<int>(ErrorCode::kFileOPError);
  }

  if (PEM_write_bio_RSAPrivateKey(out, r, EVP_aes_128_cbc(), NULL, 0, NULL,
                                  const_cast<char *>(encKey.c_str())) != 1) {
    std::cout << "Write PriKey Failed." << std::endl;
    RSA_free(r);
    BN_free(bne);
    BIO_free(out);
    return static_cast<int>(ErrorCode::kFileOPError);
  }
  BIO_free(out);
  out = NULL;
  out = BIO_new_file(pubKeyPath.c_str(), "w");
  EVP_PKEY *pubk = EVP_PKEY_new();
  if (pubk == NULL) {
    std::cout << "New Memory Failed." << std::endl;
    RSA_free(r);
    BN_free(bne);
    BIO_free(out);
    return static_cast<int>(ErrorCode::kMemError);
  }
  EVP_PKEY_assign_RSA(pubk, r);
  if (out != NULL && PEM_write_bio_PUBKEY(out, pubk) == 1) {
    ret = static_cast<int>(ErrorCode::kSuccess);
  } else {
    ret = static_cast<int>(ErrorCode::kFailed);
  }
  EVP_PKEY_free(pubk);
  BN_free(bne);
  BIO_free(out);
  std::cout << "NEW Gen KeyPair" << std::endl;
  return ret;
}

int iot::HobotUtils::LoadKeyFromFile(const std::string &filePath,
                                     std::string &keyStr) {
  std::ifstream infile(filePath);
  if (infile.is_open()) {
    std::string tmpstr((std::istreambuf_iterator<char>(infile)),
                       std::istreambuf_iterator<char>());
    keyStr.assign(tmpstr);
    infile.close();
    return static_cast<int>(ErrorCode::kSuccess);
  }
  return static_cast<int>(ErrorCode::kFailed);
}
#ifdef LIBTARGZ
int gzopen_frontend(char *pathname, int oflags, int mode) {
  const char *gzoflags;
  gzFile gzf;
  int fd;
  switch (oflags & O_ACCMODE) {
    case O_WRONLY:
      gzoflags = "wb";
      break;
    case O_RDONLY:
      gzoflags = "rb";
      break;
    default:
    case O_RDWR:
      errno = EINVAL;
      return -1;
  }
  fd = open(pathname, oflags, mode);
  if (fd == -1) return -1;
  if ((oflags & O_CREAT) && fchmod(fd, mode)) {
    close(fd);
    return -1;
  }
  gzf = gzdopen(fd, gzoflags);
  if (!gzf) {
    errno = ENOMEM;
    return -1;
  }
  /* This is a bad thing to do on big-endian lp64 systems, where the
     size and placement of integers is different than pointers.
     However, to fix the problem 4 wrapper functions would be needed and
     an extra bit of data associating GZF with the wrapper functions.  */
  return (intptr_t)(gzf);
}
#endif
int iot::HobotUtils::ExtraTarDotGzFile(const std::string &srcFile,
                                       const std::string &destDir) {
#ifdef LIBTARGZ
  tartype_t gztype = {(openfunc_t)gzopen_frontend, (closefunc_t)gzclose,
                      (readfunc_t)gzread, (writefunc_t)gzwrite};
  TAR *t = NULL;
  if (tar_open(&t, srcFile.c_str(), &gztype, O_RDONLY, 0, TAR_GNU) == -1) {
    std::cout << "tar_open() Failed:" << strerror(errno) << std::endl;
    return static_cast<int>(ErrorCode::kFileOPError);
  }
  if (tar_extract_all(t, const_cast<char *>(destDir.c_str())) != 0) {
    std::cout << "tar_extract_all() failed:" << strerror(errno) << std::endl;
    tar_close(t);
    return static_cast<int>(ErrorCode::kFileOPError);
  }
  if (tar_close(t) != 0) {
    std::cout << "tar_close():" << strerror(errno) << std::endl;
    return static_cast<int>(ErrorCode::kFileOPError);
  }
#endif
  return static_cast<int>(ErrorCode::kSuccess);
}

int iot::HobotUtils::MoveFile(const std::string &srcFile,
                              const std::string &dstFile) {
  if (FileExists(srcFile) == false) {
    return static_cast<int>(ErrorCode::kFileOPError);
  }
  if (rename(srcFile.c_str(), dstFile.c_str()) != 0) {
    return static_cast<int>(ErrorCode::kFileOPError);
  }
  return 0;
}
int iot::HobotUtils::DeleteFile(const std::string &fileName) {
  if (remove(fileName.c_str()) != 0) {
    return static_cast<int>(ErrorCode::kFileOPError);
  }
  return static_cast<int>(ErrorCode::kSuccess);
}
int iot::HobotUtils::DeleteFloder(const std::string &floder) {
  DIR *d = opendir(floder.c_str());
  int ret = -1;
  if (d) {
    struct dirent *p;
    ret = 0;
    while (!ret && (p = readdir(d))) {
      char buf[256];
      if (!strcmp(p->d_name, ".") || !strcmp(p->d_name, "..")) {
        continue;
      }
      struct stat statbuf;
      snprintf(buf, sizeof(buf), "%s/%s", floder.c_str(), p->d_name);
      if (!stat(buf, &statbuf)) {
        if (S_ISDIR(statbuf.st_mode)) {
          ret = DeleteFloder(buf);
        } else {
          ret = remove(buf);
        }
      }
    }
    closedir(d);
  }
  if (!ret) {
    ret = rmdir(floder.c_str());
  }
  return ret;
}
int iot::HobotUtils::GetFileDir(const std::string &filename,
                                std::string &fileDir) {
  if (FileExists(filename) == false) {
    return static_cast<int>(ErrorCode::kFileOPError);
  }
  fileDir.assign(filename);
  for (auto rit = filename.crbegin(); rit != filename.crend(); ++rit) {
    if (*rit != '/') {
      fileDir.pop_back();
    } else {
      break;
    }
  }
  return 0;
}

int iot::HobotUtils::ComputeSHA256(const std::string &fileName,
                                   std::string &hash) {
  std::ifstream infile(fileName);
  if (infile.is_open()) {
    std::string tmpstr((std::istreambuf_iterator<char>(infile)),
                       std::istreambuf_iterator<char>());
    infile.close();
    unsigned char md[SHA256_DIGEST_LENGTH];
    SHA256((const unsigned char *)tmpstr.c_str(), tmpstr.length(), md);
    char buf[SHA256_DIGEST_LENGTH * 2 + 1];
    //int bufLen = SHA256_DIGEST_LENGTH *2 + 1;
    memset(buf, 0, SHA256_DIGEST_LENGTH * 2 + 1);
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
      sprintf(buf,"%s%02x", buf, md[i]);
    }
    buf[SHA256_DIGEST_LENGTH * 2] = '\0';
    hash.assign(buf);
    infile.close();
    return static_cast<int>(ErrorCode::kSuccess);
  }
  return static_cast<int>(ErrorCode::kFailed);
}

int iot::HobotUtils::GetPureFileName(const std::string &filename,
                                     std::string &pureFileName) {
  if (filename.back() == '/') {
    return static_cast<int>(ErrorCode::kFailed);
  }
  std::size_t found = filename.find_last_of("/");
  if (found == std::string::npos) {
    pureFileName = filename;
  } else {
    pureFileName = filename.substr(found + 1);
  }
  return 0;
}
