#pragma once

// Host replacement for the one mbedTLS primitive the hub uses (AES-128-ECB
// encrypt of a single block), backed by the platform crypto library so the 2W
// challenge response is computed for real rather than faked.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef __APPLE__
#include <CommonCrypto/CommonCryptor.h>
#else
#include <openssl/evp.h>
#endif

#define MBEDTLS_AES_ENCRYPT 1

struct mbedtls_aes_context {
  uint8_t key[16];
};

inline void mbedtls_aes_init(mbedtls_aes_context *ctx) { memset(ctx, 0, sizeof(*ctx)); }
inline void mbedtls_aes_free(mbedtls_aes_context *ctx) { memset(ctx, 0, sizeof(*ctx)); }
inline int mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const uint8_t *key, unsigned int keybits) {
  if (keybits != 128)
    return -1;
  memcpy(ctx->key, key, 16);
  return 0;
}
inline int mbedtls_aes_crypt_ecb(mbedtls_aes_context *ctx, int mode, const uint8_t input[16], uint8_t output[16]) {
  (void) mode;
#ifdef __APPLE__
  size_t moved = 0;
  const CCCryptorStatus status = CCCrypt(kCCEncrypt, kCCAlgorithmAES, kCCOptionECBMode, ctx->key, kCCKeySizeAES128,
                                         nullptr, input, 16, output, 16, &moved);
  if (status != kCCSuccess || moved != 16) {
    fprintf(stderr, "AES stub failed: status=%d moved=%zu\n", status, moved);
    exit(2);
  }
#else
  EVP_CIPHER_CTX *evp = EVP_CIPHER_CTX_new();
  int moved = 0;
  int final_moved = 0;
  if (evp == nullptr || EVP_EncryptInit_ex(evp, EVP_aes_128_ecb(), nullptr, ctx->key, nullptr) != 1 ||
      EVP_CIPHER_CTX_set_padding(evp, 0) != 1 || EVP_EncryptUpdate(evp, output, &moved, input, 16) != 1 ||
      EVP_EncryptFinal_ex(evp, output + moved, &final_moved) != 1 || moved + final_moved != 16) {
    fprintf(stderr, "AES stub failed\n");
    EVP_CIPHER_CTX_free(evp);
    exit(2);
  }
  EVP_CIPHER_CTX_free(evp);
#endif
  return 0;
}
