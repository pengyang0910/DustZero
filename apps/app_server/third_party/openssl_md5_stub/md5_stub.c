/*
 * Minimal MD5 implementation compatible with OpenSSL API.
 * Based on the standard RFC 1321 algorithm.
 */

#include "openssl/md5.h"
#include <string.h>

#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z)))

#define ROTLEFT(a, b) (((a) << (b)) | ((a) >> (32 - (b))))

#define FF(a, b, c, d, m, s, t) \
    { (a) += F((b), (c), (d)) + (m) + (t); \
      (a) = ROTLEFT((a), (s)); \
      (a) += (b); }

#define GG(a, b, c, d, m, s, t) \
    { (a) += G((b), (c), (d)) + (m) + (t); \
      (a) = ROTLEFT((a), (s)); \
      (a) += (b); }

#define HH(a, b, c, d, m, s, t) \
    { (a) += H((b), (c), (d)) + (m) + (t); \
      (a) = ROTLEFT((a), (s)); \
      (a) += (b); }

#define II(a, b, c, d, m, s, t) \
    { (a) += I((b), (c), (d)) + (m) + (t); \
      (a) = ROTLEFT((a), (s)); \
      (a) += (b); }

static void md5_transform(unsigned int state[4], const unsigned char block[64])
{
    unsigned int a = state[0], b = state[1], c = state[2], d = state[3];
    unsigned int m[16];
    int i, j;

    for (i = 0, j = 0; i < 16; ++i, j += 4)
        m[i] = (unsigned int)(block[j]) |
               ((unsigned int)(block[j + 1]) << 8) |
               ((unsigned int)(block[j + 2]) << 16) |
               ((unsigned int)(block[j + 3]) << 24);

    /* Round 1 */
    FF(a, b, c, d, m[0],  7, 0xd76aa478);
    FF(d, a, b, c, m[1], 12, 0xe8c7b756);
    FF(c, d, a, b, m[2], 17, 0x242070db);
    FF(b, c, d, a, m[3], 22, 0xc1bdceee);
    FF(a, b, c, d, m[4],  7, 0xf57c0faf);
    FF(d, a, b, c, m[5], 12, 0x4787c62a);
    FF(c, d, a, b, m[6], 17, 0xa8304613);
    FF(b, c, d, a, m[7], 22, 0xfd469501);
    FF(a, b, c, d, m[8],  7, 0x698098d8);
    FF(d, a, b, c, m[9], 12, 0x8b44f7af);
    FF(c, d, a, b, m[10], 17, 0xffff5bb1);
    FF(b, c, d, a, m[11], 22, 0x895cd7be);
    FF(a, b, c, d, m[12],  7, 0x6b901122);
    FF(d, a, b, c, m[13], 12, 0xfd987193);
    FF(c, d, a, b, m[14], 17, 0xa679438e);
    FF(b, c, d, a, m[15], 22, 0x49b40821);

    /* Round 2 */
    GG(a, b, c, d, m[1],  5, 0xf61e2562);
    GG(d, a, b, c, m[6],  9, 0xc040b340);
    GG(c, d, a, b, m[11], 14, 0x265e5a51);
    GG(b, c, d, a, m[0], 20, 0xe9b6c7aa);
    GG(a, b, c, d, m[5],  5, 0xd62f105d);
    GG(d, a, b, c, m[10],  9, 0x02441453);
    GG(c, d, a, b, m[15], 14, 0xd8a1e681);
    GG(b, c, d, a, m[4], 20, 0xe7d3fbc8);
    GG(a, b, c, d, m[9],  5, 0x21e1cde6);
    GG(d, a, b, c, m[14],  9, 0xc33707d6);
    GG(c, d, a, b, m[3], 14, 0xf4d50d87);
    GG(b, c, d, a, m[8], 20, 0x455a14ed);
    GG(a, b, c, d, m[13],  5, 0xa9e3e905);
    GG(d, a, b, c, m[2],  9, 0xfcefa3f8);
    GG(c, d, a, b, m[7], 14, 0x676f02d9);
    GG(b, c, d, a, m[12], 20, 0x8d2a4c8a);

    /* Round 3 */
    HH(a, b, c, d, m[5],  4, 0xfffa3942);
    HH(d, a, b, c, m[8], 11, 0x8771f681);
    HH(c, d, a, b, m[11], 16, 0x6d9d6122);
    HH(b, c, d, a, m[14], 23, 0xfde5380c);
    HH(a, b, c, d, m[1],  4, 0xa4beea44);
    HH(d, a, b, c, m[4], 11, 0x4bdecfa9);
    HH(c, d, a, b, m[7], 16, 0xf6bb4b60);
    HH(b, c, d, a, m[10], 23, 0xbebfbc70);
    HH(a, b, c, d, m[13],  4, 0x289b7ec6);
    HH(d, a, b, c, m[0], 11, 0xeaa127fa);
    HH(c, d, a, b, m[3], 16, 0xd4ef3085);
    HH(b, c, d, a, m[6], 23, 0x04881d05);
    HH(a, b, c, d, m[9],  4, 0xd9d4d039);
    HH(d, a, b, c, m[12], 11, 0xe6db99e5);
    HH(c, d, a, b, m[15], 16, 0x1fa27cf8);
    HH(b, c, d, a, m[2], 23, 0xc4ac5665);

    /* Round 4 */
    II(a, b, c, d, m[0],  6, 0xf4292244);
    II(d, a, b, c, m[7], 10, 0x432aff97);
    II(c, d, a, b, m[14], 15, 0xab9423a7);
    II(b, c, d, a, m[5], 21, 0xfc93a039);
    II(a, b, c, d, m[12],  6, 0x655b59c3);
    II(d, a, b, c, m[3], 10, 0x8f0ccc92);
    II(c, d, a, b, m[10], 15, 0xffeff47d);
    II(b, c, d, a, m[1], 21, 0x85845dd1);
    II(a, b, c, d, m[8],  6, 0x6fa87e4f);
    II(d, a, b, c, m[15], 10, 0xfe2ce6e0);
    II(c, d, a, b, m[6], 15, 0xa3014314);
    II(b, c, d, a, m[13], 21, 0x4e0811a1);
    II(a, b, c, d, m[4],  6, 0xf7537e82);
    II(d, a, b, c, m[11], 10, 0xbd3af235);
    II(c, d, a, b, m[2], 15, 0x2ad7d2bb);
    II(b, c, d, a, m[9], 21, 0xeb86d391);

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
}

static void md5_encode(unsigned char *output, const unsigned int *input, unsigned int len)
{
    unsigned int i, j;
    for (i = 0, j = 0; j < len; i++, j += 4) {
        output[j]     = (unsigned char)(input[i] & 0xff);
        output[j + 1] = (unsigned char)((input[i] >> 8) & 0xff);
        output[j + 2] = (unsigned char)((input[i] >> 16) & 0xff);
        output[j + 3] = (unsigned char)((input[i] >> 24) & 0xff);
    }
}

int MD5_Init(MD5_CTX *ctx)
{
    if (ctx == NULL)
        return 0;
    ctx->A = 0x67452301;
    ctx->B = 0xefcdab89;
    ctx->C = 0x98badcfe;
    ctx->D = 0x10325476;
    ctx->Nl = 0;
    ctx->Nh = 0;
    ctx->num = 0;
    return 1;
}

int MD5_Update(MD5_CTX *ctx, const void *data, size_t len)
{
    const unsigned char *input = data;
    unsigned int i, index, partLen;

    if (ctx == NULL || (data == NULL && len != 0))
        return 0;

    index = (unsigned int)((ctx->Nl >> 3) & 0x3f);
    partLen = 64 - index;

    ctx->Nl += (unsigned int)(len << 3);
    if (ctx->Nl < (len << 3))
        ctx->Nh++;
    ctx->Nh += (unsigned int)(len >> 29);

    if (len >= partLen) {
        if (index) {
            memcpy((unsigned char *)ctx->data + index, input, partLen);
            md5_transform(&ctx->A, (unsigned char *)ctx->data);
            i = partLen;
        } else {
            i = 0;
        }
        for (; i + 63 < len; i += 64)
            md5_transform(&ctx->A, input + i);
        index = 0;
    } else {
        i = 0;
    }

    if (len - i)
        memcpy((unsigned char *)ctx->data + index, input + i, len - i);

    return 1;
}

int MD5_Final(unsigned char *md, MD5_CTX *ctx)
{
    unsigned char bits[8];
    unsigned int index, padLen;
    static const unsigned char padding[64] = {
        0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    if (ctx == NULL || md == NULL)
        return 0;

    md5_encode(bits, &ctx->Nl, 8);

    index = (unsigned int)((ctx->Nl >> 3) & 0x3f);
    padLen = (index < 56) ? (56 - index) : (120 - index);
    MD5_Update(ctx, padding, padLen);
    MD5_Update(ctx, bits, 8);
    md5_encode(md, &ctx->A, 16);

    memset(ctx, 0, sizeof(*ctx));
    return 1;
}
