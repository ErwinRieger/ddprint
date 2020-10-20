/*
 * uzlib  -  tiny deflate/inflate library (deflate, gzip, zlib)
 *
 * Copyright (c) 2003 by Joergen Ibsen / Jibz
 * All Rights Reserved
 * http://www.ibsensoftware.com/
 *
 * Copyright (c) 2014-2018 by Paul Sokolovsky
 *
 * This software is provided 'as-is', without any express
 * or implied warranty.  In no event will the authors be
 * held liable for any damages arising from the use of
 * this software.
 *
 * Permission is granted to anyone to use this software
 * for any purpose, including commercial applications,
 * and to alter it and redistribute it freely, subject to
 * the following restrictions:
 *
 * 1. The origin of this software must not be
 *    misrepresented; you must not claim that you
 *    wrote the original software. If you use this
 *    software in a product, an acknowledgment in
 *    the product documentation would be appreciated
 *    but is not required.
 *
 * 2. Altered source versions must be plainly marked
 *    as such, and must not be misrepresented as
 *    being the original software.
 *
 * 3. This notice may not be removed or altered from
 *    any source distribution.
 */

#ifndef UZLIB_H_INCLUDED
#define UZLIB_H_INCLUDED

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* calling convention */
#ifndef TINFCC
 #ifdef __WATCOMC__
  #define TINFCC __cdecl
 #else
  #define TINFCC
 #endif
#endif

/* ok status, more data produced */
#define TINF_OK             0
/* end of compressed stream reached */
#define TINF_DONE           1
#define TINF_DATA_ERROR    (-3)
#define TINF_CHKSUM_ERROR  (-4)
#define TINF_DICT_ERROR    (-5)

/* checksum types */
#define TINF_CHKSUM_NONE  0
#define TINF_CHKSUM_ADLER 1
#define TINF_CHKSUM_CRC   2

/* helper macros */
#define TINF_ARRAY_SIZE(arr) (sizeof(arr) / sizeof(*(arr)))

/* data structures */

typedef struct {
   unsigned short table[16];  /* table of code length counts */
   unsigned short trans[288]; /* code -> symbol translation table */
} TINF_TREE;

struct uzlib_uncomp {
    uint8_t tag;
    uint8_t bitcount;

    // bool eof;

    int8_t btype;
    // int bfinal;
    uint8_t curlen;
    uint8_t lzOff;

    TINF_TREE ltree; /* dynamic length/symbol tree */
    TINF_TREE dtree; /* dynamic distance tree */
};

unsigned char TINFCC uzlib_get_byte(uzlib_uncomp *d);

/* Decompression API */

void TINFCC uzlib_init(void);
void TINFCC uzlib_uncompress_init(uzlib_uncomp *d, void *dict, unsigned int dictLen);
int  TINFCC uzlib_uncompress(uzlib_uncomp *d);
int  TINFCC uzlib_uncompress_chksum(uzlib_uncomp *d);

int TINFCC uzlib_zlib_parse_header(uzlib_uncomp *d);
int TINFCC uzlib_gzip_parse_header(uzlib_uncomp *d);

/* Compression API */

typedef const uint8_t *uzlib_hash_entry_t;

void TINFCC uzlib_compress(struct uzlib_comp *c, const uint8_t *src, unsigned slen);

/* Checksum API */

/* prev_sum is previous value for incremental computation, 1 initially */
uint32_t TINFCC uzlib_adler32(const void *data, unsigned int length, uint32_t prev_sum);
/* crc is previous value for incremental computation, 0xffffffff initially */
uint32_t TINFCC uzlib_crc32(const void *data, unsigned int length, uint32_t crc);

#endif /* UZLIB_H_INCLUDED */
