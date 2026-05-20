#ifndef OPENSSL_MD5_H
#define OPENSSL_MD5_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MD5_DIGEST_LENGTH 16

typedef struct MD5state_st {
    unsigned int A, B, C, D;
    unsigned int Nl, Nh;
    unsigned int data[16];
    unsigned int num;
} MD5_CTX;

int MD5_Init(MD5_CTX *c);
int MD5_Update(MD5_CTX *c, const void *data, size_t len);
int MD5_Final(unsigned char *md, MD5_CTX *c);

#ifdef __cplusplus
}
#endif

#endif
