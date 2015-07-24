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


#include <libgen.h>
#include <sys/timeb.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <dirent.h>
#include <Arduino.h>
#include <avr/wdt.h>

#include "ddprint.h"
#include "MarlinSerial.h"
#include "swapdev.h"
#include "stepper.h"

unsigned long long getTimestamp() {

    struct timeval tv;
    gettimeofday(&tv, NULL); 

    return tv.tv_sec * 1000000 + tv.tv_usec;
}

unsigned long long timestamp;
static unsigned long long lastTempIRQ;
static unsigned long long lastHeater;

// xxx baudrate 125000 -> geht nicht mit python serial?
// xxx check file close after print
// xxx check stepper range/buildvoluem
// xxx fileprint scheint nicht zu funktionieren, heitzt auf, und ist dann sofortd fertig...
//
// sicherheitsfunktionen im temp-modul wieder aktivierenk, setWatch();
//
// XXX aus dem ext. maint. menü kommt man nicht mehr raus falls 'save eeprom' benutzt wurde...
// XXX frequenz steppertakte prüfen, um bug mit feedrate zu detektieren
//
static time_t watchdogCalled = time(NULL);
static bool wdEnabled = true;

extern void loop();
extern bool randomSerialError;
extern void charClock();
static bool reset_button_press = false;

void *isrThread(void *data);
// static void checkPrintFinished();

int printStarted = 0;

pthread_t tids;

// unsigned char block_buffer_head = 0;
// unsigned char block_buffer_tail = 0;

bool headHeaterOn = false;
int heaterADC = 250;
bool bedHeaterOn = false;
int bedADC = 250;

FILE *viewFile = NULL;

#if ! defined(PROFILING)

void nonblock(int state);
int kbhit();

//
// Write simulated gcode, view with "yagv sim.gcode".
//
void openOutputModel() {

    if (viewFile != NULL)
        fclose(viewFile);

    assert((viewFile = fopen("sim.gcode", "w+")) != NULL);

#if 0
    char *prefix = "\n\
        G1 X-5 Y5 Z5\n\
        G1 X200 Y5 Z5\n\
        G1 X200 Y5 Z-200\n\
        G1 X200 Y5 Z5\n\
        G1 X200 Y-200 Z5\n\
        G1 X200 Y-200 Z-200\n\
        G1 X200 Y-200 Z5\n\
        G1 X-5 Y-200 Z5\n\
        G1 X-5 Y-200 Z-200\n\
        G1 X-5 Y-200 Z5\n\
        G1 X-5 Y5 Z5\n\
        \n\
        G1 X-5 Y5 Z-200\n\
        G1 X200 Y5 Z-200\n\
        G1 X200 Y-200 Z-200\n\
        G1 X-5 Y-200 Z-200\n\
        G1 X-5 Y5 Z-200\n\
        G1 X-5 Y5 Z5\n\
        \n";
#endif

// #define InitialY 230
// #define InitialZ 230
// G1 X-0.1 Y-0.1 Z0
//
    char *prefix = "\n\
        G1 X0 Y230 Z230\n\
        \n";
        fwrite(prefix, 1, strlen(prefix), viewFile);
}
#endif

main(int argc, char** argv) {

    printf("usage:\n");
    printf("%s -p<filename>: print file (sdread mode)\n", argv[0]);
    printf("%s -l: run normal loop(), process serial commands)\n", argv[0]);
    printf("%s -w: disable watchdog (for gdb)\n", argv[0]);

    char ch;
    char * filename = NULL;
    uint8_t mode = 0;


///////////////////////////////////////////////////////////

	while ((ch = getopt(argc, argv, "rlwp:")) != -1)
	switch(ch) {
	case 'r':
		randomSerialError = true;
		break;
	case 'p':
        mode = 'p';
		filename = optarg;
		break;
	case 'l':
        mode = 'l';
		break;
	case 'w':
        // Disable watchdog for run in debugger
        wdEnabled = false;
		break;
	}

    assert(mode);

///////////////////////////////////////////////////////////


        srand(time(NULL));

        int i=0;

#if ! defined(PROFILING)
        openOutputModel();
        nonblock(1);
#endif

        assert(pthread_create(&tids, NULL, isrThread, (void *) NULL) == 0);

        setup();

        bool runMainloop = true;

        while (runMainloop) {

#if ! defined(PROFILING)
                if (kbhit()) {
                    //
                    // Keys:
                    //  j/k          -> down/up
                    //  n            -> next selection (main screens)
                    //  Blank/Return -> click
                    //  q            -> exit 
                    //
                    char c=fgetc(stdin);
                    printf("pressed: '%c'\n", c);
                    if (c=='q') {
                        fclose(viewFile);
                        break;
                    }
#if 0
                    else if (c=='k')
                        lcd_lib_encoder_pos -= 4;
                    else if (c=='j')
                        lcd_lib_encoder_pos += 4;
                    else if (c=='n')
                        lcd_lib_encoder_pos += 8;
#endif
                    else if (c=='f')
                        // flush output model
                        fflush(viewFile);
                    else if (c=='r')
                        // reopen output model
                        openOutputModel();
#if 0
                    else if (c==' ' || c=='\n')
                        lcd_lib_button_pressed = 1;
#endif
                }

#endif
#if 0
                // xxx debug
                if (i == 100) {

                    // Queue the G28 Task
                    G28Task *homeTask = new G28Task(0.0);
                    queueTask(homeTask);

                    // enquecommand_P(PSTR("G1 Z40"));
                    CommandTask *raiseTask = new CommandTask();
                    raiseTask->strcpy_PGM(PSTR("G1 Z40"));
                    queueTask(raiseTask);
                }
#endif

                timestamp = getTimestamp();

                loop();

#if 0
                if (reset_button_press) {
                    lcd_lib_button_pressed = 0;
                    reset_button_press = false;
                }
#endif

                if (mode == 'p') {
                    assert(0);
#if 0
                    if (i == 5)
                        card.openFile(filename, 1);
#endif
                    if (i == 10) {
                        printStarted = 1;
                        assert(0); // card.startFileprint();
                    }

                    assert(0);
#if 0
                    if (i>100 && !card.sdprinting) {
                        printf("main(): printing done, exiting...\n");
                    }
#endif
                }

                if (mode == 'p' && i < 20) {
                    usleep(100000);
                }
                else {
                    unsigned long delta = getTimestamp() - timestamp;
                    if (delta < 150)
                        usleep(150 - delta);

                    double r = drand48();
                    if (r < 0.001 && r > 0.0000001) {
                        // printf("r: %.5f\n", r);
                        usleep(250000 * r);
                    }
                    charClock();
                }

                if (printStarted==2) {

                    printf("Print finished...\n");
                    assert(0); // assert(!card.sdprinting);
                    assert(0); // assert (! card.isFileOpen());

                    // if (is_command_queued()) {
                        // printf("queue not empty... %d\n", buflen);
                    // }
                    // else {
                        runMainloop = false;
                        if (mode == 'l') {
                            printf("sleep 10...");
                            sleep(10);
                        }
                        if (errorFlags)
                            printf("\n\nSerial errors encountered: 0x%x\n\n", errorFlags);
                    // }
                }

                // checkPrintFinished();


                i++;
        }

#if ! defined(PROFILING)
    nonblock(0);
#endif

    write(1, NULL, 0);
}

//////////////////////// nonblockin keyboard input /////////////////////////////
#if ! defined(PROFILING)
int kbhit()
{

    return 0;

    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(int state)
{
    struct termios ttystate;
 
    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);
 
    if (state==1)
    {
        //turn off canonical mode
        ttystate.c_lflag &= ~ICANON;
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state==0)
    {
        //turn on canonical mode
        ttystate.c_lflag |= ICANON;
    }
    //set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
 
}
//////////////////////// nonblockin keyboard input /////////////////////////////
#endif


// int strcmp_P(const char *s1, const char *s2) {
    // return strncmp(s1, s2, min(strlen(s1), strlen(s2)));
// }

ssize_t xwrite(int fd, const void *buf, size_t count) {
    return write(fd, buf, count);
}

char *xfgets(char *s, int size, FILE *stream){
    return fgets(s, size, stream);
}


extern long position[4];   //rescaled from extern when axis_steps_per_unit are changed by gcode
extern  long count_position[NUM_AXIS];
extern signed char count_direction[NUM_AXIS];

float lastPosX = 0;
float lastPosY = Y_MAX_POS;
float lastPosZ = Z_MAX_POS;

extern int ISRTIMER1_COMPA_vect();
extern int ISRTIMER1_COMPB_vect();

void *isrThread(void * data) {

   printf("isrThread called\n");
    unsigned char out_bits;        // The next stepping-bits to be output

   while (true) {

            // Check watchdog:
            time_t wdtTS = time(NULL);
            int wdtTime = wdtTS - watchdogCalled;
            if (wdtTime > WDTO_4S && wdEnabled) {
                printf("Watchdog timeout: %d!!!\n", wdtTime);
                assert(0);
            }

            // Temperature timer interrupt
            unsigned long long ts = getTimestamp();
            unsigned long tempdelta = ts - lastTempIRQ;
            if (tempdelta > 1000) {

                // Test, if someone wants to read the adc:
                if (ADCSRA & (1<<ADSC)) {

                    // Heater emulation
                    unsigned long heatdelta = ts - lastHeater;
                    if (ADMUX ==  ((1 << REFS0) | (TEMP_0_PIN & 0x07))) {
                        if (heatdelta > 100000) {
                            if (headHeaterOn)
                                heaterADC += 2;
                            else 
                                if (heaterADC > 240)
                                    heaterADC -= 1;
                            lastHeater = ts;
                        }
                        ADC = heaterADC;
                    }
            
                    if (ADMUX == ((1 << REFS0) | (TEMP_BED_PIN & 0x07))) {
                        if (heatdelta > 100000) {
                            if (bedHeaterOn)
                                bedADC += 2;
                            else 
                                if (bedADC > 240)
                                    bedADC -= 1;
                            lastHeater = ts;
                        }
                        ADC = bedADC;
                    }

                    ADCSRA = ADCSRA & ~(1 << ADSC);
                }

                // ISRTIMER0_COMPB_vect();
                lastTempIRQ = ts;
            }

            unsigned long moveStart = getTimestamp();
             
            // Stepper interrupt
            if (TIMSK1 & (1<<OCIE1A))
                ISRTIMER1_COMPA_vect();

            if (TIMSK1 & (1<<OCIE1B))
                ISRTIMER1_COMPB_vect();


#if ! defined(PROFILING)
            // Write gcode simulation output
            if ((fabs(lastPosX - ssx.pos) > 1) || (fabs(lastPosY -ssy.pos) > 1) || (fabs(lastPosZ - ssz.pos) >= 0.1)) {

                char buf[64];

                sprintf(buf, "G1 X%.1f Y%.1f Z%.1f\n", ssx.pos, ssy.pos, ssz.pos);

                fwrite(buf, 1, strlen(buf), viewFile);

                lastPosX = ssx.pos;
                lastPosY = ssy.pos;
                lastPosZ = ssz.pos;
            }
#endif

            unsigned long delta = getTimestamp() - moveStart;

            if (delta < 100) 
                usleep(100 - delta);
    }
}

float pgm_read_float_near(const float *p) {
    float f = *p;
    // printf("pgm_read_float_near(0x%x) called: %f\n", (uint)p, f);
    return f;
}

char pgm_read_byte_near(const signed char *p) {
    char c = *p;
    // printf("pgm_read_byte_near(0x%x) called: '%c'\n", (uint)p, c);
    return c;
}

unsigned short pgm_read_word_near(const uint8_t *p) {
    unsigned short c = *(unsigned short*)p;
    // printf("pgm_read_word_near(0x%x) called: '%c'\n", (uint)p, c);
    return c;
}

uint16_t pgm_read_word(const void* ptr)
{
        return *(const uint16_t*)ptr;
}


uint8_t READ(int pin) { 
        switch(pin) {
            case SAFETY_TRIGGERED_PIN:
                return 0;
            case X_STOP_PIN:
                // X
                if (ssx.pos <= -10) {
                    // printf("Reached X-enstop at pos %.1f\n", ssx.pos);
                    return 0;
                }
                return 1;
            case Y_STOP_PIN:
                // Y
                if (ssy.pos >= Y_MAX_LENGTH + 10) {
                    // printf("Reached Y-enstop at pos %.1f\n", ssy.pos);
                    return 0;
                }
                return 1;
            case Z_STOP_PIN:
                // Z
                if (ssz.pos >= Z_MAX_LENGTH + 10) {
                    // printf("Reached Z-enstop at pos %.1f\n", ssz.pos);
                    return 0;
                }
                return 1;
        }
        assert(0);
}

void WRITE(int pin, uint8_t v) { 

        switch(pin) {
            case X_ENABLE_PIN:
                // printf("writing %d to pin X_ENABLE_PIN\n", v);
                ssx.enable(v == X_ENABLE_ON);
                return;
            case Y_ENABLE_PIN:
                // printf("writing %d to pin Y_ENABLE_PIN\n", v);
                ssy.enable(v == Y_ENABLE_ON);
                return;
            case Z_ENABLE_PIN:
                // printf("writing %d to pin Z_ENABLE_PIN\n", v);
                ssz.enable(v == Z_ENABLE_ON);
                return;

            case X_DIR_PIN:
                // printf("writing %d to pin X_DIR_PIN\n", v);
                ssx.setDir(v);
                return;
            case Y_DIR_PIN:
                // printf("writing %d to pin Y_DIR_PIN\n", v);
                ssy.setDir(v);
                return;
            case Z_DIR_PIN:
                // printf("writing %d to pin Z_DIR_PIN\n", v);
                ssz.setDir(v);
                return;
            case E0_DIR_PIN:
                // printf("writing %d to pin E0_DIR_PIN\n", v);
                // sse.setDir(v);
                return;

            case X_STEP_PIN:
                // printf("writing %d to pin X_STEP_PIN\n", v);
                ssx.setPin(v);
                return;
            case Y_STEP_PIN:
                // printf("writing %d to pin Y_STEP_PIN\n", v);
                ssy.setPin(v);
                return;
            case Z_STEP_PIN:
                // printf("writing %d to pin Z_STEP_PIN\n", v);
                ssz.setPin(v);
                return;

            case X_STOP_PIN:
                // printf("writing %d to pin X_STOP_PIN\n", v);
                return;
            case Y_STOP_PIN:
                // printf("writing %d to pin Y_STOP_PIN\n", v);
                return;
            case Z_STOP_PIN:
                // printf("writing %d to pin Z_STOP_PIN\n", v);
                return;

            case E0_ENABLE_PIN:
                // printf("writing %d to pin E0_ENABLE_PIN\n", v);
                return;
            case E1_ENABLE_PIN:
                // printf("writing %d to pin E1_ENABLE_PIN\n", v);
                return;

            case E0_STEP_PIN:
                return;
            case E1_STEP_PIN:
                return;

            case PS_ON_PIN:
                printf("writing %d to pin PS_ON_PIN\n", v);
                return;
            case HEATER_0_PIN:
                // extruder heater
                headHeaterOn = v;
                return;
            case HEATER_BED_PIN:
                // bed heater
                bedHeaterOn = v;
                return;
        }
        assert(0);
}

int32_t max(int32_t a ,int32_t b) {

    if (a > b)
        return a;
    return b;
}

int32_t min(int32_t a ,int32_t b) {
    if (a < b)
        return a;
    return b;
}


ulong millis()
{
    struct timeb timebuffer;
    ftime(&timebuffer);
    return (timebuffer.time * 1000) + timebuffer.millitm;
}


extern "C" {
    void * __brkval = 0;
    unsigned int __bss_end = 0;
}

bool lcd_lib_button_down = false;
// unsigned long lastSerialCommandTime = 0;
// char fnAutoPrint[13] = { '\0' };


int constrain(int v, int l, int u) {
  if (v < l) v=l;
  if (v > u) v=u;
  return v;
}

float constrain(float v, float l, float u) {
  if (v < l) v=l;
  if (v > u) v=u;
  return v;
}

bool tone(int, int)
{ assert(0); }
bool delay(int)
{ assert(0); }
bool noTone(int)
{ assert(0); }
void prepare_arc_move(bool b)
{ assert(0); }

void wdt_enable(uint8_t /* wdt_time */) { watchdogCalled = time(NULL); }

void SET_OUTPUT(int pin) { printf("Setting pin %d to output mode\n", pin); }
void SET_INPUT(int pin) { printf("Setting pin %d to input mode\n", pin); }
void pinMode(int pin, uint8_t m) { 
    switch(pin) {
        case MOTOR_CURRENT_PWM_XY_PIN:
            printf("Setting MOTOR_CURRENT_PWM_XY_PIN to mode %d\n", m);
            break;
        case MOTOR_CURRENT_PWM_Z_PIN:
            printf("Setting MOTOR_CURRENT_PWM_Z_PIN to mode %d\n", m);
            break;
        case MOTOR_CURRENT_PWM_E_PIN:
            printf("Setting MOTOR_CURRENT_PWM_E_PIN to mode %d\n", m);
            break;
        default:
            assert(0);
    }
}

void digitalWrite(int pin, uint8_t m)
{ assert(0); }

void analogWrite(int pin, uint8_t m) {
    switch(pin) {
        case FAN_PIN:
            // printf("Setting FAN_PIN to value %d\n", m);
            break;
        case MOTOR_CURRENT_PWM_XY_PIN:
            break;
        case MOTOR_CURRENT_PWM_Z_PIN:
            break;
        case MOTOR_CURRENT_PWM_E_PIN:
            break;
        case LED_PIN:
            break;
        case HEATER_0_PIN:
            if (m > 50) 
                headHeaterOn = true;
            else
                headHeaterOn = false;
            break;
        default:
            assert(0);
    }
}


int TIMSK1 = 0;
// int TIMSK3 = 0;
int OCR1A = 0;
int OCR1B = 0;
int OCR3A = 0;
int OCR0A = 0;
int TCNT1 = 1;
int TCNT3 = 1;
int TCCR1B = 1;
int TCCR1A = 1;
int TCCR3B = 1;
int TCCR3A = 1;
int TCCR5B = 1;
int ADCSRA = 0;
int DIDR0 = 1;
int DIDR2 = 1;
int OCR0B = 1;
int TIMSK0 = 1;
// Temp 0
int ADC = 250;
int ADCSRB = 0;
int ADMUX = 0;

// extern uint8_t __eeprom__storage[4096];
static uint8_t * __eeprom__storage = NULL;
static int eepromFd = -1;
static bool eepromRead = false;

void eepromROOpen() {
    if (__eeprom__storage == NULL)
        __eeprom__storage = ( uint8_t *) malloc(4096);

    if (! eepromRead) {
        assert((eepromFd = open("simulator/eeprom.dump", O_RDONLY)) >= 0);
        assert(read(eepromFd,__eeprom__storage,4096) == 4096);
        close(eepromFd);
        eepromRead = true;
    }
}

void eepromFlush() {

    assert((eepromFd = open("simulator/eeprom.dump", O_WRONLY)) >= 0);
    // assert(lseek(eepromFd, 0, SEEK_SET) == 0);
    assert(write(eepromFd,__eeprom__storage,4096) == 4096);
    close(eepromFd);
}

uint8_t eeprom_read_byte (const uint8_t *__p)
{  
    eepromROOpen();
    return __eeprom__storage[long(__p)];
}

uint16_t eeprom_read_word (const uint16_t *__p)
{
    eepromROOpen();
    return *(uint16_t*)&__eeprom__storage[long(__p)];
}

#define LIFETIME_EEPROM_OFFSET 0x700
uint32_t eeprom_read_dword (const uint32_t *__p)
{
    eepromROOpen();
    return *(uint32_t*)&__eeprom__storage[long(__p)];
}

void eeprom_read_block (void *__dst, const void *__src, size_t __n)
{
    eepromROOpen();
    memcpy(__dst, &__eeprom__storage[long(__src)], __n);
}

void eeprom_write_dword (uint32_t *__p, uint32_t __value)
{
    eepromROOpen();
    *(uint32_t*)&__eeprom__storage[long(__p)] = __value;
    eepromFlush();
}

void eeprom_write_word (uint16_t *__p, uint16_t __value)
{
    eepromROOpen();
    *(uint16_t*)&__eeprom__storage[long(__p)] = __value;
    eepromFlush();
}

void eeprom_write_byte (uint8_t *__p, uint8_t __value)
{
    eepromROOpen();
    __eeprom__storage[long(__p)] = __value;
    eepromFlush();
}

void eeprom_write_block(const void * __src, void * __dst, size_t __n) {
    eepromROOpen();
    memcpy(__eeprom__storage+(long)__dst, __src, __n);
    eepromFlush();
}

// Display memory, 10 lines a 20 characters:
// 128x64 
#define PixelWidth 128
#define PixelHeight 64
#define CharWidth 6
// #define CharHeight 8
#define CharHeight 7
// In characters:
#define DisplayWidth (PixelWidth / CharWidth)
#define DisplayHeight (PixelHeight / CharHeight)

#define CharXFromPixel(x) (x/CharWidth)
#define CharYFromPixel(y) (y/CharHeight)

#define DpBufferLen (DisplayHeight*(DisplayWidth+1))

static char dpBuffer[DisplayHeight][DisplayWidth+1];
static char dpBuffer2[DisplayHeight][DisplayWidth+1];

void lcd_lib_init() {
    memset(dpBuffer, ' ', DisplayHeight*(DisplayWidth+1));
    for (int y=0; y<DisplayHeight; y++)
        dpBuffer[y][DisplayWidth] = '\0';
    assert(0); // lcd_lib_encoder_pos = ENCODER_NO_SELECTION;
}

bool lcd_lib_update_ready()
{
    return true;
}

void lcd_lib_buttons_update()
{
}

const char *strSet(int len, char c) {


    static char s[DisplayWidth + 1];
    int i;
    for (i=0; i<len; i++) {
        assert(i < DisplayWidth);
        s[i] = c;
    }
    s[i] = 0;

    return s;
}

static unsigned long long lastLcdUpdate = getTimestamp();

void lcd_lib_update_screen()
{

    unsigned long long ts = getTimestamp();
    unsigned long delta = ts - lastLcdUpdate;
    if (delta < 250000)
        return;
    lastLcdUpdate = ts;

    if (memcmp(dpBuffer, dpBuffer2, DpBufferLen) ) {

        printf("    %s\n", strSet(DisplayWidth, '-'));
        for (int l=0; l<DisplayHeight; l++) {
            printf("    |"); printf(dpBuffer[l]); printf("|\n");
        }
        printf("    %s\n", strSet(DisplayWidth, '-'));
        memcpy(dpBuffer2, dpBuffer, DpBufferLen);
    }
}

void lcd_lib_draw_stringP(uint8_t x, uint8_t y, const char* pstr)
{
    assert(x>=0 && y>=0);
    assert(x<128 && y<64);

    int cX = x / CharWidth;
    int cY = y / CharHeight;

    assert(cY < DisplayHeight);

    uint8_t c;
    int i=0;

    char * line = dpBuffer[cY];

    while (c = pstr[i]) {
        assert(cX+i < DisplayWidth);
        line[cX+i] = c;
        i++;
    }
}

void lcd_lib_draw_string_centerP(uint8_t y, const char* pstr)
{
    lcd_lib_draw_stringP(64 - strlen_P(pstr) * 3, y, pstr);
}

const char *handleLineBreak(const char *pstr) {
    // Crude line break handling:
    char *pos = strchr((char*)pstr, '|');
    if (pos) {
        static char s[DisplayWidth+1];
        strcpy(s, pstr);
        s[pos-pstr] = 0;
        return s;
    }
    return pstr;
}

void lcd_lib_clear_string_center_atP(uint8_t x, uint8_t y, const char* pstr)
{
    // printf("lcd_lib_clear_string_center_atP(): '%s'\n", pstr);
    pstr = handleLineBreak(pstr);
    int xofs = (strlen(pstr)*CharWidth) / 2;
    lcd_lib_draw_stringP(x-xofs, y, strSet(strlen(pstr), '_'));
    lcd_lib_draw_stringP(x-xofs, y, pstr);
}
void lcd_lib_draw_string_center_atP(uint8_t x, uint8_t y, const char* pstr)
{
    // printf("lcd_lib_draw_string_center_atP(): '%s'\n", pstr);
    pstr = handleLineBreak(pstr);
    int xofs = (strlen(pstr)*CharWidth) / 2;
    lcd_lib_draw_stringP(x-xofs, y, pstr);
}
void lcd_lib_clear_string_centerP(uint8_t y, const char* pstr)
{
    // printf("lcd_lib_clear_string_centerP(): '%s'\n", pstr);
    // lcd_lib_draw_stringP(0, y, ".-.-.-.-");
    lcd_lib_clear_string_center_atP(PixelWidth/2, y, pstr);
}
void lcd_lib_clear_stringP(uint8_t x, uint8_t y, const char* pstr)
{
    // printf("lcd_lib_clear_stringP(): '%s'\n", pstr);
    assert(0); // lcd_lib_clear_string(x, y, pstr);
}

void lcd_lib_draw_string(uint8_t x, uint8_t y, const char* str)
{
    // printf("lcd_lib_draw_string(): '%s'\n", str);
    lcd_lib_draw_stringP(x, y, str);
}

void lcd_lib_set(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    assert(x1>x0);
    assert(y1>y0);

    int cX0 = CharXFromPixel(x0);
    int cX1 = CharXFromPixel(x1);
    int cY0 = CharYFromPixel(y0);
    int cY1 = CharYFromPixel(y1);

    int len = cX1 - cX0;// + 1;

    const char *s = strSet(len, '*');

    for (int y=cY0; y<cY1; y++) 
        lcd_lib_draw_stringP(x0, y*CharHeight, s);
}

void lcd_lib_clear_string(uint8_t x, uint8_t y, const char* str)
{
    lcd_lib_draw_stringP(x, y, strSet(strlen(str), '_'));
    lcd_lib_draw_stringP(x, y, str);
}

void lcd_lib_draw_shade(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{

    assert(0);
}
void run_electronics_test()
{
    assert(0);
}
static void mc_arc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1,
  uint8_t axis_linear, float feed_rate, float radius, uint8_t isclockwise, uint8_t extruder)
{
    assert(0);
}

void lcd_lib_led_color(uint8_t r, uint8_t g, uint8_t b)
{
}
void lcd_lib_draw_gfx(uint8_t x, uint8_t y, const uint8_t* gfx)
{
}
void lcd_lib_clear()
{
    memset(dpBuffer, ' ', DisplayHeight*(DisplayWidth+1));
    for (int y=0; y<DisplayHeight; y++)
        dpBuffer[y][DisplayWidth] = '\0';
}

void lcd_lib_clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{

    if (x1 > 128) {
        printf("XXX lcd_lib_clear: x1 %d out of bounds\n", x1);
        x1 = 128;
    }
    int cX0 = CharXFromPixel(x0);
    int cX1 = CharXFromPixel(x1);
    int cY0 = CharYFromPixel(y0);
    int cY1 = CharYFromPixel(y1);

    int len = cX1 - cX0;// + 1;

    const char *s = strSet(len, '_');

    len = cY1 - cY0;// + 1;
    for (int y=cY0; y<cY0+len; y++) 
        lcd_lib_draw_stringP(cX0, y, s);

}
void lcd_lib_beep()
{
}
void lcd_lib_draw_hline(uint8_t x, uint8_t y0, uint8_t y1)
{
}
void lcd_lib_draw_box(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
}
void lcd_lib_draw_vline(uint8_t x, uint8_t y0, uint8_t y1)
{
    int cY0 = CharYFromPixel(y0);
    int cY1 = CharYFromPixel(y1);

    for (int y=cY0; y<cY1; y++)
        lcd_lib_draw_stringP(x, y*CharWidth, "|");
}

char* float_to_string(float f, char* temp_buffer, const char* p_postfix)
{
    int32_t i = f * 100.0 + 0.5;
    char* c = temp_buffer;
    if (i < 0)
    {
        *c++ = '-';
        i = -i;
    }
    if (i >= 10000)
        *c++ = ((i/10000)%10)+'0';
    if (i >= 1000)
        *c++ = ((i/1000)%10)+'0';
    *c++ = ((i/100)%10)+'0';
    *c++ = '.';
    if (i >= 10)
        *c++ = ((i/10)%10)+'0';
    *c++ = ((i)%10)+'0';
    *c = '\0';
    if (p_postfix)
    {
        strcpy_P(c, p_postfix);
        c += strlen_P(p_postfix);
    }
    return c;
}

char* int_to_string(int i, char* temp_buffer, const char* p_postfix)
{
    char* c = temp_buffer;
    if (i < 0)
    {
        *c++ = '-';
        i = -i;
    }
    if (i >= 10000)
        *c++ = ((i/10000)%10)+'0';
    if (i >= 1000)
        *c++ = ((i/1000)%10)+'0';
    if (i >= 100)
        *c++ = ((i/100)%10)+'0';
    if (i >= 10)
        *c++ = ((i/10)%10)+'0';
    *c++ = ((i)%10)+'0';
    *c = '\0';
    if (p_postfix)
    {
        strcpy_P(c, p_postfix);
        c += strlen_P(p_postfix);
    }
    return c;
}

void lcd_lib_draw_string_center(uint8_t y, const char* str)
{
    lcd_lib_draw_string(64 - strlen(str) * 3, y, str);
}

void lcd_lib_buttons_update_interrupt()
{
    reset_button_press = true;
}

char* int_to_time_string(unsigned long i, char* temp_buffer)
{
    char* c = temp_buffer;
    uint8_t hours = i / 60 / 60;
    uint8_t mins = (i / 60) % 60;
    uint8_t secs = i % 60;

    if (hours > 0)
    {
        if (hours > 99)
            *c++ = '0' + hours / 100;
        if (hours > 9)
            *c++ = '0' + (hours / 10) % 10;
        *c++ = '0' + hours % 10;
        if (hours > 1)
        {
            strcpy_P(c, PSTR(" hours"));
            return c + 6;
        }
        strcpy_P(c, PSTR(" hour"));
        return c + 5;
    }
    if (mins > 0)
    {
        if (mins > 9)
            *c++ = '0' + (mins / 10) % 10;
        *c++ = '0' + mins % 10;
        strcpy_P(c, PSTR(" min"));
        return c + 4;
    }
    if (secs > 9)
        *c++ = '0' + secs / 10;
    *c++ = '0' + secs % 10;
    strcpy_P(c, PSTR(" sec"));
    return c + 4;
    /*
    if (hours > 99)
        *c++ = '0' + hours / 100;
    *c++ = '0' + (hours / 10) % 10;
    *c++ = '0' + hours % 10;
    *c++ = ':';
    *c++ = '0' + mins / 10;
    *c++ = '0' + mins % 10;
    *c++ = ':';
    *c++ = '0' + secs / 10;
    *c++ = '0' + secs % 10;
    *c = '\0';
    return c;
    */
}

static int8_t lcd_lib_encoder_pos_interrupt = 0;
bool lcd_lib_button_pressed = 0; // ENCODER_NO_SELECTION;
int16_t lcd_lib_encoder_pos;
 
void watchdog_reset() { watchdogCalled = time(NULL); }

