/*
* This file is part of ddprint - a 3D printer firmware.
* 
* Copyright 2020 erwin.rieger@ibrieger.de
* 
* ddprint is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* ddprint is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "uz.h"
#include "uzlib/uzlib.h"
#include "serialport.h"
#include "swapdev.h"
#include "mdebug.h"

extern unsigned char length_bits[30];
extern unsigned char length_base[30];
extern unsigned char dist_bits[30];
extern unsigned char dist_bases[30];

extern unsigned short dist_base[30];

extern int tinf_getbit(TINF_DATA *d);
extern unsigned int tinf_read_bits(TINF_DATA *d, int num, int base);
extern void tinf_build_fixed_trees(TINF_TREE *lt, TINF_TREE *dt);
extern int tinf_decode_trees(TINF_DATA *d, TINF_TREE *lt, TINF_TREE *dt);
extern int tinf_decode_symbol(TINF_DATA *d, TINF_TREE *t);

/* inflate next output bytes from compressed stream */
bool UnZipper::Run() {

    int sym;

    PT_BEGIN();

    // PT_WAIT_UNTIL(busyWriting);
    // PT_WAIT_WHILE(writeBlock(writeBlockNumber, writeBuffer));

    // Skip 2 header bytes, read by uzlib_zlib_parse_header() else
    serialPort.readNoCheckCobs();
    serialPort.readNoCheckCobs();

    d.eof = 0;
    d.bitcount = 0;
    d.bfinal = 0;
    d.btype = -1;
    d.dict_size = 0;
    d.dict_ring = NULL;
    d.dict_idx = 0;
    d.curlen = 0;

    d.source = 0;
    d.source_limit = 0;
    d.source_read_cb = NULL;

    // d.dest_start = d.dest = out;
    // d.dest_limit = d.dest + outsize;

    while (1) {

        sym = 0;

        /* start a new block */
        if (d.btype == -1) {
            int old_btype;

            old_btype = d.btype;
            /* read final block flag */
            d.bfinal = tinf_getbit(&d);
            /* read block type (2 bits) */
            d.btype = tinf_read_bits(&d, 2, 0);

            #if UZLIB_CONF_DEBUG_LOG >= 1
            printf("Started new block: type=%d final=%d\n", d.btype, d.bfinal);
            #endif

            if (d.btype == 1 && old_btype != 1) {
                /* build fixed huffman trees */
                tinf_build_fixed_trees(&d.ltree, &d.dtree);
            } else if (d.btype == 2) {
                /* decode trees from stream */
                massert( tinf_decode_trees(&d, &d.ltree, &d.dtree) == TINF_OK );
            }
        }

        /* process current block */
        if (d.btype == 0) {

            /* decompress uncompressed block */
            // res = tinf_inflate_uncompressed_block(&d);

            //////////////////////////////////////////////////////////////
            /* inflate next byte from uncompressed block of data */
            // static int tinf_inflate_uncompressed_block(TINF_DATA *d)
            // {
                if (d.curlen == 0) {
                    unsigned int length, invlength;

                    /* get length */
                    length = uzlib_get_byte(&d);
                    length += 256 * uzlib_get_byte(&d);
                    /* get one's complement of length */
                    invlength = uzlib_get_byte(&d);
                    invlength += 256 * uzlib_get_byte(&d);
                    /* check length */
                    if (length != (~invlength & 0x0000ffff)) return TINF_DATA_ERROR;

                    /* increment length to properly return TINF_DONE below, without
                    producing data at the same time */
                    d.curlen = length + 1;

                    /* make sure we start next block on a byte boundary */
                    d.bitcount = 0;
                }

                if (--d.curlen == 0) {
                    // return TINF_DONE;
                    break;
                }

                // unsigned char c = uzlib_get_byte(d);
                // TINF_PUT(d, c);
                // return TINF_OK;

                sym = uzlib_get_byte(&d);
                TINF_PUT_TAIL(d, sym);
            // }
            //////////////////////////////////////////////////////////////
        }
        else if ((d.btype == 1) || (d.btype == 2)) {

            /* decompress block with fixed/dynamic huffman trees */
            /* trees were decoded previously, so it's the same routine for both */
            // res = tinf_inflate_block_data(&d, &d.ltree, &d.dtree);

            //////////////////////////////////////////////////////////////

            if (d.curlen == 0) {

                sym = tinf_decode_symbol(&d, &d.ltree);
                if (sym < 256) {
                    /* literal byte */

                    // TINF_PUT(&d, sym);

                    // swapDev.addByte(sym);
                    // PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    TINF_PUT_TAIL(d, sym);
                    // continue;
                }
                else if (sym == 256) {
                    /* end of block */
                    break;
                }
                else {

                    /* substring from sliding dictionary */
                    sym -= 257;

                    /* possibly get more bits from length code */
                    d.curlen = tinf_read_bits(&d, length_bits[sym], length_base[sym]);

                    int dist = tinf_decode_symbol(&d, &d.dtree);

                    /* possibly get more bits from distance code */
                    unsigned int offs = tinf_read_bits(&d, dist_bits[dist], dist_base[dist]);

                    /* calculate and validate actual LZ offset to use */
                    if (d.dict_ring) {
                        /* Note: unlike full-dest-in-memory case below, we don't
                        try to catch offset which points to not yet filled
                        part of the dictionary here. Doing so would require
                        keeping another variable to track "filled in" size
                        of the dictionary. Appearance of such an offset cannot
                        lead to accessing memory outside of the dictionary
                        buffer, and clients which don't want to leak unrelated
                        information, should explicitly initialize dictionary
                        buffer passed to uzlib. */

                        d.lzOff = d.dict_idx - offs;
                        if (d.lzOff < 0) {
                            d.lzOff += d.dict_size;
                        }
                    } else {
                        /* catch trying to point before the start of dest buffer */
                        d.lzOff = -offs;
                    }

                    // A
                    /* copy next byte from dict substring */
                    if (d.dict_ring) {

                        // TINF_PUT(&d, d.dict_ring[d.lzOff]);

                        sym = d.dict_ring[d.lzOff];

                        // swapDev.addByte(sym);
                        // PT_WAIT_WHILE( swapDev.isBusyWriting() );

                        TINF_PUT_TAIL(d, sym);

                        if ((unsigned)++d.lzOff == d.dict_size) {
                            d.lzOff = 0;
                        }
                    } else {
                        d.dest[0] = d.dest[d.lzOff];
                        d.dest++;
                        d.curlen--;
                        continue; // dont add byte
                    }
                    d.curlen--;
                    /* */
                }
            }
            else {
                // A
                /* copy next byte from dict substring */
                if (d.dict_ring) {

                    // TINF_PUT(&d, d.dict_ring[d.lzOff]);

                    sym = d.dict_ring[d.lzOff];

                    // swapDev.addByte(sym);
                    // PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    TINF_PUT_TAIL(d, sym);

                    if ((unsigned)++d.lzOff == d.dict_size) {
                        d.lzOff = 0;
                    }
                } else {
                    d.dest[0] = d.dest[d.lzOff];
                    d.dest++;
                    d.curlen--;
                    continue; // dont add byte
                }
                d.curlen--;
                /* */
            }
            //////////////////////////////////////////////////////////////
        }
        else {
            // Unknown btype
            massert(0);
        }

        swapDev.addByte(sym);
        PT_WAIT_WHILE( swapDev.isBusyWriting() );
    }

    PT_RESTART();
    PT_END();
}

UnZipper unZipper;




