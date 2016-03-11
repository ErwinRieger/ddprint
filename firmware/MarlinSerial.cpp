
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

#include <util/crc16.h>

#include "ddprint.h"
#include "ddserial.h"
#include "ddcommands.h"
#include "MarlinSerial.h"

//things to write to serial from Programmemory. saves 400 to 2k of RAM.
void serialprintPGM(const char *str)
{
    uint8_t len = strlen_P(str);
    txBuffer.sendResponseStart(RespGenericString, len);

    for (uint8_t i=0; i<len; i++) {

        uint8_t ch=pgm_read_byte(str+i);
        txBuffer.pushCharChecksum(ch);
    }
    txBuffer.sendResponseEnd();
}

uint8_t MarlinSerial::peekN(uint8_t index) {

    return rx_buffer.buffer[(rx_buffer.tail+index) % RX_BUFFER_SIZE];
}

void MarlinSerial::peekChecksum(uint16_t *checksum, uint8_t count) {

    for (uint8_t i=0; i<count; i++)
        *checksum = _crc_xmodem_update(*checksum, peekN(i));
}

uint8_t MarlinSerial::serReadNoCheck(void)
{
    unsigned char c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    return c;
}

float MarlinSerial::serReadFloat()
{
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        float f = *(float*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail = (rx_buffer.tail + 4) % RX_BUFFER_SIZE; // xxx rx_buffer.tail += 4
        return f;
    }

    uint32_t i = serReadNoCheck() + (((uint32_t)serReadNoCheck())<<8) + (((uint32_t)serReadNoCheck())<<16) + (((uint32_t)serReadNoCheck())<<24);
    return *(float*)&i;;
}

int32_t MarlinSerial::serReadInt32()
{
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        int32_t i = *(int32_t*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail = (rx_buffer.tail + 4) % RX_BUFFER_SIZE;  // xxx rx_buffer.tail += 4 
        return i;
    }

    return MSerial.serReadNoCheck() + (((int32_t)MSerial.serReadNoCheck())<<8) + (((int32_t)MSerial.serReadNoCheck())<<16) + (((int32_t)MSerial.serReadNoCheck())<<24);
}

uint32_t MarlinSerial::serReadUInt32()
{
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        uint32_t i = *(uint32_t*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail = (rx_buffer.tail + 4) % RX_BUFFER_SIZE;  // xxx rx_buffer.tail += 4 
        return i;
    }

    return MSerial.serReadNoCheck() + (((uint32_t)MSerial.serReadNoCheck())<<8) + (((uint32_t)MSerial.serReadNoCheck())<<16) + (((uint32_t)MSerial.serReadNoCheck())<<24);
}

void MarlinSerial::flush()
{
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  rx_buffer.head = rx_buffer.tail;
}

void MarlinSerial::serWrite(const char *str)
{
      // while (*str)
        // serWrite(*str++);
        uint8_t len = strlen(str);
        txBuffer.sendResponseStart(RespGenericString, len+1);
        txBuffer.sendResponseValue((char*)str, len);
        txBuffer.sendResponseEnd();
}

void MarlinSerial::serWrite(const uint8_t *buffer, size_t size)
{
      // while (size--)
        // serWrite(*buffer++);
    txBuffer.sendResponseStart(RespGenericString, size);
    txBuffer.sendResponseValue((uint8_t*)buffer, size);
    txBuffer.sendResponseEnd();
}


void MarlinSerial::print(char c, int base)
{
  print((long) c, base);
}

void MarlinSerial::print(unsigned char b, int base)
{
  print((unsigned long) b, base);
}

void MarlinSerial::print(int n, int base)
{
  print((long) n, base);
}

void MarlinSerial::print(unsigned int n, int base)
{
  print((unsigned long) n, base);
}

void MarlinSerial::print(long n, int base)
{
#if defined(DDSim)
    printf("xxx MarlinSerial::print(long n, int base)\n");
    assert(0);
#endif
#if 0
  if (base == 0) {
    serWrite(n);
  } else if (base == 10) {
    if (n < 0) {
      print('-');
      n = -n;
    }
    printNumber(n, 10);
  } else {
    printNumber(n, base);
  }
#endif
}

void MarlinSerial::print(unsigned long n, int base)
{
#if defined(DDSim)
    printf("xxx  MarlinSerial::print(unsigned long n, int base)\n");
    assert(0);
#endif
#if 0
  if (base == 0) serWrite(n);
  else printNumber(n, base);
#endif
}

void MarlinSerial::print(double n, int digits)
{
  printFloat(n, digits);
}

void MarlinSerial::println(void)
{
  print('\r');
  print('\n');
}

#if 0
void MarlinSerial::println(const String &s)
{
  print(s);
  println();
}
#endif

void MarlinSerial::println(const char c[])
{
  print(c);
  println();
}

void MarlinSerial::println(char c, int base)
{
  print(c, base);
  println();
}

void MarlinSerial::println(unsigned char b, int base)
{
  print(b, base);
  println();
}

void MarlinSerial::println(int n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned int n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(long n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned long n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(double n, int digits)
{
  print(n, digits);
  println();
}

// Private Methods /////////////////////////////////////////////////////////////

void MarlinSerial::printNumber(unsigned long n, uint8_t base)
{
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long i = 0;

  if (n == 0) {
    print('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}

void MarlinSerial::printFloat(double number, uint8_t digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint;
  }
}
// Preinstantiate Objects //////////////////////////////////////////////////////


MarlinSerial MSerial;

