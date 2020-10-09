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

#include "uzlib.h"
#include "mdebug.h"

unsigned int zlibUncompress( unsigned char *in, unsigned int insize, unsigned char *out, unsigned int outsize ) {

    struct uzlib_uncomp d;

    d.eof = 0;
    d.bitcount = 0;
    d.bfinal = 0;
    d.btype = -1;
    d.dict_size = 0;
    d.dict_ring = NULL;
    d.dict_idx = 0;
    d.curlen = 0;

    d.source = in + 2; // skip 2 header bytes, read by uzlib_zlib_parse_header() else
    d.source_limit = in + insize;
    d.source_read_cb = NULL;

    d.dest_start = d.dest = out;

    d.dest_limit = d.dest + outsize;

    int res = uzlib_uncompress(&d);

    if (res != TINF_DONE) {
        // printf("Error during decompression: %d\n", res);
        // exit(-res);
        massert(0);
    }

    return d.dest - out;
}


