/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2015 erwin.rieger@ibrieger.de
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

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <assert.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

#include "MarlinSerial.h"
#include "ddserial.h"
#include "Configuration.h"

extern unsigned long long getTimestamp();
extern unsigned long long timestamp;

static uint8_t rx_buffer_error = 0;
// Filedescriptor for serial pseudo tty  
int ptty;
bool randomSerialError = false;

MarlinSerial::MarlinSerial() {

    printf("new MarlinSerial\n");

    rxBuffer.head = rxBuffer.tail = rxerror = 0;

    ptty = open("/dev/ptmx", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (ptty == -1) {
        printf("error opening /dev/ptmx\n");
        return;
    }

    grantpt(ptty);
    unlockpt(ptty);

    const char* pts_name = ptsname(ptty);
    assert(pts_name);

    printf("ptsname: %s (/tmp/ttyUSB10)\n", pts_name);

    char buf[1024];
    sprintf(buf, "ln -sf %s /tmp/ttyUSB10", pts_name);
    system(buf);

    int tmp;
    assert((tmp = open(pts_name, O_RDWR | O_NOCTTY)) >= 0);
    struct termios slave_orig_term_settings;

    assert( tcgetattr(tmp, &slave_orig_term_settings) >= 0); 

    // Disable echo on the slave side of the PTY
    slave_orig_term_settings.c_lflag &= ~ECHO;

    assert(tcsetattr (tmp, TCSANOW, &slave_orig_term_settings) >= 0); 
    close(tmp);
};

void charClock() {

    static bool connected = false;
    static int nerr = 0;
    char buf[1024];

    float delta = getTimestamp() - timestamp;

    // 10 bits brauchen 10/Baudrate sekunden 
    // Berechne anzahl zeichen die seit dem letzten read empfangen
    // worden sein könten:
    int maxread = (delta*BAUDRATE/10000000);
    if (maxread == 0)
        maxread ++;

#if 0
    if (randomSerialError) {

        unsigned long long delta = getTimestamp() - timestamp;

        // 10 bits brauchen 87uS bei 115200 baud
        // berechne anzahl zeichen die seit dem letzten read empfangen
        // worden sein könten:
        maxread = delta / 90;
        if (maxread == 0)
            maxread ++;
    }
    else {
        maxread = 3 - fifoFill;
    }
#endif

    // printf("delta: %f us, maxread: %d\n", delta, maxread);

    struct pollfd pfd = { .fd = ptty, .events = POLLHUP };
    poll(&pfd, 1, 0 /* or other small timeout */);

    if (!(pfd.revents & POLLHUP))
    {

        if (! connected) {

            /* There is now a reader on the slave side */
            printf("client connected\n");

            // simulate reset, echo greeting message
            // SERIAL_ECHO_START;
            // SERIAL_ECHOLN("MARLINSIM");
            // SERIAL_ECHO_START;
            // SERIAL_ECHOLN("SD card ok");
            connected = true;
            nerr = 0;
        }
    }
    else {
        if (connected) {
            printf("client dis-connected\n");
            connected = false;
        }
    }

    ssize_t nread = read(ptty, buf, maxread);

    if (nread > 0) {

        for (int i=0; i<nread; i++) {

            int rxChar = buf[i];

            // printf("serial read: '0x%x'\n", rxChar);

            /*
            if (fifoFill == 3) {
                printf("serial overflow maxread: %d, nread: %d\n", maxread, nread);
                // set error bits in 80% of the cases, else let the
                // other error detecting mechanisms come into play
                if (drand48() > 0.20)
                    rx_buffer_error = 0x8; // SET DOR0A bit, data overrun
                break;
            }
            fifo[fifoFill++] = rxChar;
            */
            if (MSerial._available() == (RX_BUFFER_SIZE-1)) {
                printf("serial overflow maxread: %d, nread: %d\n", maxread, nread);
                assert(0);
            }
            MSerial.store_char(rxChar);
        }
    }
}

void MarlinSerial::begin(long) { };

#if 0
void MarlinSerial::serWrite(uint8_t c) {
    // write(ptty, &c, 1);
    txBuffer.pushChar(c);
}
#endif









