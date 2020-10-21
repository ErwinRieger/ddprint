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

#include "serialport.h"
#include "swapdev.h"

// #define UZ_MODULE_TEST 1

#if defined(UZ_MODULE_TEST)
class SerialPort {

    public:

        int inputpos;
        unsigned char inputbuffer[256];

        SerialPort() {
            init();
        }

        void init() {
            inputpos = 0;
        }

        unsigned char readNoCheckCobs() {
            assert(inputpos < 256);
            return inputbuffer[inputpos++];
        }
};

class SwapDev {

    public:

        uint8_t outputpos;
        unsigned char outputbuffer[256];

        void init() { 
            outputpos = 233; // test overflow
        }
        void addByte(unsigned char c) {
            // printf("addbyte %d: %c\n", outputpos, c);
            outputbuffer[outputpos++] = c;
        }
        void addBackRefByte(uint8_t offs) {
            uint8_t c = outputbuffer[outputpos - offs];
            printf("addbackref %d - %d = %d %c\n", outputpos, offs, (outputpos - offs), c);
            outputbuffer[outputpos++] = c;
        }
        bool isBusyWriting() {
            static bool wait = false;
            // printf("isbuxy...\n");
            wait = !wait;
            return wait;
        }
};

SerialPort serialPort;
SwapDev swapDev;
#endif

/* get one bit from source stream */
int tinf_getbit(uzlib_uncomp *d)
{
   unsigned int bit;

   /* check if tag is empty */
   if (!d->bitcount--)
   {
      /* load next tag */
      d->tag = serialPort.readNoCheckCobs();
      d->bitcount = 7;
   }

   /* shift bit out of tag */
   bit = d->tag & 0x01;
   d->tag >>= 1;

   return bit;
}

/* Inflate next output bytes from compressed stream */
bool UnZipper::Run() {

    int sym;
    int dist;
    int res;

    PT_BEGIN();

    d.bitcount = 0;
    d.curlen = 0;
    d.btype = -1;

    while (true) {

        /* start a new block */
        if (d.btype == -1) {
            int old_btype;

            old_btype = d.btype;
            /* read final block flag */
            /* d.bfinal = */ tinf_getbit(&d);
            /* read block type (2 bits) */
            d.btype = tinf_read_bits(&d, 2, 0);

            #if UZLIB_CONF_DEBUG_LOG >= 1
            printf("Started new block: type=%d final=%d\n", d.btype);
            #endif
// xxx build tables ony once?
            if (d.btype == 1 && old_btype != 1) {
                /* build fixed huffman trees */
                tinf_build_fixed_trees(&d.ltree, &d.dtree);
            } else if (d.btype == 2) {
                /* decode trees from stream */
                // xxxx trees contained in d...
                massert(tinf_decode_trees(&d, &d.ltree, &d.dtree) == TINF_OK);
            }
        }

        // printf("V: btype: %d, res: %d, curlen: %d, pos: %d\n", d.btype, res, d.curlen, serialPort.inputpos);

        /* process current block */
        if ((d.btype == 1) || (d.btype == 2)) {

            /* decompress block with fixed/dynamic huffman trees */
            /* trees were decoded previously, so it's the same routine for both */
            // res = tinf_inflate_block_data(d, &d.ltree, &d.dtree);
            // int tinf_inflate_block_data(uzlib_uncomp *d, TINF_TREE *lt, TINF_TREE *dt)
            // res = my_tinf_inflate_block_data(&d, &d.ltree, &d.dtree);
            if (d.curlen == 0) {
                sym = tinf_decode_symbol(&d, &d.ltree);
                //printf("huff sym: %02x\n", sym);

                /* end of block */
                if (sym == 256) {
                    res = TINF_DONE;
                }
                else if (sym < 256) {
                    /* literal byte */
                    swapDev.addByte(sym);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );
                    res = TINF_OK;
                }
                else {

                    /* substring from sliding dictionary */
                    sym -= 257;
                    #if defined(HEAVYDEBUG)
                        massert(sym < 29);
                    #endif

                    /* possibly get more bits from length code */
                    d.curlen = tinf_read_bits(&d, length_bits[sym], length_base[sym]);

                    dist = tinf_decode_symbol(&d, &d.dtree);
                    #if defined(HEAVYDEBUG)
                        massert(dist < 30);
                    #endif

                    /* possibly get more bits from distance code */
                    /* calculate and validate actual LZ offset to use */
                    d.lzOff = tinf_read_bits(&d, dist_bits[dist], dist_base[dist]);;

                    /* copy next byte from dict substring */
                    swapDev.addBackRefByte(d.lzOff);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    d.curlen--;
                    res = TINF_OK;
                }
            }
            else {
                /* copy next byte from dict substring */
                swapDev.addBackRefByte(d.lzOff);
                PT_WAIT_WHILE( swapDev.isBusyWriting() );
                d.curlen--;
                res = TINF_OK;
            }
        }
        else {
            UZUNSUPPORTED_ERR;
        }

        // printf("N: res: %d, curlen: %d, pos: %d %d\n", res, d.curlen, serialPort.inputpos);
        //
        if (res == TINF_DONE) {
            // assert(d.bfinal);
            break;
        }
        else {
            #if defined(HEAVYDEBUG)
                massert(res == TINF_OK);
            #endif
        }
    }

    // PT_RESTART();
    PT_END();
}

UnZipper unZipper;


#if defined(UZ_MODULE_TEST)
#include <string.h>

int main(int argc, char** argv) {

    uint8_t orig[256];

    for (int filenr=0; filenr<102; filenr++) {

        char fn[64];

        sprintf(fn, "/tmp/ddprint_block_%d.zip", filenr);

        FILE *f = fopen(fn, "r");
        assert(f);

        int inputsize;
        inputsize = fread(serialPort.inputbuffer, 1, 256, f);
        printf("\n\ntesting %s, %d bytes read...\n", fn, inputsize);
        assert(inputsize > 0);

        serialPort.init();
        swapDev.init();

        unZipper.Restart();
        while (unZipper.Run());

        printf("block done after %d of %d chars...\n", serialPort.inputpos, inputsize);

        assert((inputsize == serialPort.inputpos) or 
                ((inputsize == serialPort.inputpos+1) and (serialPort.inputbuffer[serialPort.inputpos] == 0)));

        int len = (uint8_t)(swapDev.outputpos-233);

        printf("decompressed %u bytes\n", len);

        sprintf(fn, "/tmp/ddprint_block_%d", filenr);
        f = fopen(fn, "r");
        assert(f);

        assert(fread(orig, 1, 256, f) == len);
        printf("len OK\n");

        assert(memcmp(orig, swapDev.outputbuffer, len));
        printf("compared %d bytes - file %d OK\n", len, filenr);
    }

    return 0;
}

#endif
























