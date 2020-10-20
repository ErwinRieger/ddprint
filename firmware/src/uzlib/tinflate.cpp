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

#include <stdio.h>
#include <assert.h>
#include "uzlib.h"

#define UZLIB_DUMP_ARRAY(heading, arr, size) \
    { \
        printf("%s", heading); \
        for (int i = 0; i < size; ++i) { \
            printf(" %d", (arr)[i]); \
        } \
        printf("\n"); \
    }

uint32_t tinf_get_le_uint32(uzlib_uncomp *d);
uint32_t tinf_get_be_uint32(uzlib_uncomp *d);

extern int tinf_getbit(uzlib_uncomp *d);

/* --------------------------------------------------- *
 * -- uninitialized global data (static structures) -- *
 * --------------------------------------------------- */

unsigned char length_bits[30] = {
   0, 0, 0, 0, 0, 0, 0, 0,
   1, 1, 1, 1, 2, 2, 2, 2,
   3, 3, 3, 3, 4, 4, 4, 4,
   5, 5, 5, 5
};
unsigned short length_base[30] = {
   3, 4, 5, 6, 7, 8, 9, 10,
   11, 13, 15, 17, 19, 23, 27, 31,
   35, 43, 51, 59, 67, 83, 99, 115,
   131, 163, 195, 227, 258 };

unsigned char dist_bits[30] = {
   0, 0, 0, 0, 1, 1, 2, 2,
   3, 3, 4, 4, 5, 5, 6, 6,
   7, 7, 8, 8, 9, 9, 10, 10,
   11, 11, 12, 12, 13, 13
};
unsigned short dist_base[30] = {
   1, 2, 3, 4, 5, 7, 9, 13,
   17, 25, 33, 49, 65, 97, 129, 193,
   257, 385, 513, 769, 1025, 1537, 2049, 3073,
   4097, 6145, 8193, 12289, 16385, 24577
};

/* special ordering of code length codes */
const unsigned char clcidx[] = {
   16, 17, 18, 0, 8, 7, 9, 6,
   10, 5, 11, 4, 12, 3, 13, 2,
   14, 1, 15
};

/* ----------------------- *
 * -- utility functions -- *
 * ----------------------- */

/* build the fixed huffman trees */
void tinf_build_fixed_trees(TINF_TREE *lt, TINF_TREE *dt)
{
   int i;

   /* build fixed length tree */
   for (i = 0; i < 7; ++i) lt->table[i] = 0;

   lt->table[7] = 24;
   lt->table[8] = 152;
   lt->table[9] = 112;

   for (i = 0; i < 24; ++i) lt->trans[i] = 256 + i;
   for (i = 0; i < 144; ++i) lt->trans[24 + i] = i;
   for (i = 0; i < 8; ++i) lt->trans[24 + 144 + i] = 280 + i;
   for (i = 0; i < 112; ++i) lt->trans[24 + 144 + 8 + i] = 144 + i;

   /* build fixed distance tree */
   for (i = 0; i < 5; ++i) dt->table[i] = 0;

   dt->table[5] = 32;

   for (i = 0; i < 32; ++i) dt->trans[i] = i;
}

/* given an array of code lengths, build a tree */
static void tinf_build_tree(TINF_TREE *t, const unsigned char *lengths, unsigned int num)
{
   unsigned short offs[16];
   unsigned int i, sum;

   /* clear code length count table */
   for (i = 0; i < 16; ++i) t->table[i] = 0;

   /* scan symbol lengths, and sum code length counts */
   for (i = 0; i < num; ++i) t->table[lengths[i]]++;

   #if UZLIB_CONF_DEBUG_LOG >= 2
   UZLIB_DUMP_ARRAY("codelen counts:", t->table, TINF_ARRAY_SIZE(t->table));
   #endif

   /* In the lengths array, 0 means unused code. So, t->table[0] now contains
      number of unused codes. But table's purpose is to contain # of codes of
      particular length, and there're 0 codes of length 0. */
   t->table[0] = 0;

   /* compute offset table for distribution sort */
   for (sum = 0, i = 0; i < 16; ++i)
   {
      offs[i] = sum;
      sum += t->table[i];
   }

   #if UZLIB_CONF_DEBUG_LOG >= 2
   UZLIB_DUMP_ARRAY("codelen offsets:", offs, TINF_ARRAY_SIZE(offs));
   #endif

   /* create code->symbol translation table (symbols sorted by code) */
   for (i = 0; i < num; ++i)
   {
      if (lengths[i]) t->trans[offs[lengths[i]]++] = i;
   }
}

/* ---------------------- *
 * -- decode functions -- *
 * ---------------------- */

/* read a num bit value from a stream and add base */
unsigned int tinf_read_bits(uzlib_uncomp *d, int num, int base)
{
   unsigned int val = 0;

   /* read num bits */
   if (num)
   {
      unsigned int limit = 1 << (num);
      unsigned int mask;

      for (mask = 1; mask < limit; mask *= 2)
         if (tinf_getbit(d)) val += mask;
   }

   return val + base;
}

/* given a data stream and a tree, decode a symbol */
int tinf_decode_symbol(uzlib_uncomp *d, TINF_TREE *t)
{
   int sum = 0, cur = 0, len = 0;

   /* get more bits while code value is above sum */
   do {

      cur = 2*cur + tinf_getbit(d);

      if (++len == TINF_ARRAY_SIZE(t->table)) {
         return TINF_DATA_ERROR;
      }

      sum += t->table[len];
      cur -= t->table[len];

   } while (cur >= 0);

   sum += cur;
   #if UZLIB_CONF_PARANOID_CHECKS
   if (sum < 0 || sum >= TINF_ARRAY_SIZE(t->trans)) {
      return TINF_DATA_ERROR;
   }
   #endif

   return t->trans[sum];
}

/* given a data stream, decode dynamic trees from it */
int tinf_decode_trees(uzlib_uncomp *d, TINF_TREE *lt, TINF_TREE *dt)
{
   /* code lengths for 288 literal/len symbols and 32 dist symbols */
   unsigned char lengths[288+32];
   unsigned int hlit, hdist, hclen, hlimit;
   unsigned int i, num, length;

   /* get 5 bits HLIT (257-286) */
   hlit = tinf_read_bits(d, 5, 257);

   /* get 5 bits HDIST (1-32) */
   hdist = tinf_read_bits(d, 5, 1);

   /* get 4 bits HCLEN (4-19) */
   hclen = tinf_read_bits(d, 4, 4);

   for (i = 0; i < 19; ++i) lengths[i] = 0;

   /* read code lengths for code length alphabet */
   for (i = 0; i < hclen; ++i)
   {
      /* get 3 bits code length (0-7) */
      unsigned int clen = tinf_read_bits(d, 3, 0);

      lengths[clcidx[i]] = clen;
   }

   /* build code length tree, temporarily use length tree */
   tinf_build_tree(lt, lengths, 19);

   /* decode code lengths for the dynamic trees */
   hlimit = hlit + hdist;
   for (num = 0; num < hlimit; )
   {
      int sym = tinf_decode_symbol(d, lt);
      unsigned char fill_value = 0;
      int lbits, lbase = 3;

      /* error decoding */
      if (sym < 0) return sym;

      switch (sym)
      {
      case 16:
         /* copy previous code length 3-6 times (read 2 bits) */
         if (num == 0) return TINF_DATA_ERROR;
         fill_value = lengths[num - 1];
         lbits = 2;
         break;
      case 17:
         /* repeat code length 0 for 3-10 times (read 3 bits) */
         lbits = 3;
         break;
      case 18:
         /* repeat code length 0 for 11-138 times (read 7 bits) */
         lbits = 7;
         lbase = 11;
         break;
      default:
         /* values 0-15 represent the actual code lengths */
         lengths[num++] = sym;
         /* continue the for loop */
         continue;
      }

      /* special code length 16-18 are handled here */
      length = tinf_read_bits(d, lbits, lbase);
      if (num + length > hlimit) return TINF_DATA_ERROR;
      for (; length; --length)
      {
         lengths[num++] = fill_value;
      }
   }

   #if UZLIB_CONF_DEBUG_LOG >= 2
   printf("lit code lengths (%d):", hlit);
   UZLIB_DUMP_ARRAY("", lengths, hlit);
   printf("dist code lengths (%d):", hdist);
   UZLIB_DUMP_ARRAY("", lengths + hlit, hdist);
   #endif

   #if UZLIB_CONF_PARANOID_CHECKS
   /* Check that there's "end of block" symbol */
   if (lengths[256] == 0) {
      return TINF_DATA_ERROR;
   }
   #endif

   /* build dynamic trees */
   tinf_build_tree(lt, lengths, hlit);
   tinf_build_tree(dt, lengths + hlit, hdist);

   return TINF_OK;
}

