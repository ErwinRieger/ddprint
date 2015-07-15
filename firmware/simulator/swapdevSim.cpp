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


#include "ddprint.h"
#include "MarlinSerial.h"
#include "swapdev.h"


static FILE* swapFile = NULL;
static bool chipSelect = true; // high
static int spiCommand = -1; // no command
// static int busyWait = 5;
static int commandBytes = 0;
static uint32_t writePos = 0;


#define CMD24Byte (CMD24 | 0x40)

/** SPI receive a byte */
uint8_t spiRec() {

    // SPDR = 0XFF;
    // while (!(SPSR & (1 << SPIF))) { /* Intentionally left empty */ }
    // return SPDR;

    assert(chipSelect == false);

    if (spiCommand == -1) {

        // if (busyWait--)
            // return 0;

        // busyWait = 5;
        return 0xff;
    }

    if ((spiCommand == CMD24Byte) && (commandBytes == 0)) {
        // printf("cmd24 off\n");
        spiCommand = -1;
        return 0;
    }

    if (spiCommand == CMD13) {
        spiCommand = -1;
        return 0;
    }

    assert(0);
}

void spiSend(uint8_t b) {

    // Data or command byte
    if ((spiCommand == -1) && (b == CMD24Byte)) {

        // printf("cmd24 on\n");
        // Wait for 5 bytes, len and crc
        spiCommand = b;
        writePos = 0;
        commandBytes = 5;
        return;
    }

    if (spiCommand == CMD24Byte) {
        if (commandBytes > 1) {
            writePos += ((uint32_t)b) << ((commandBytes-2)*8);
        }
        commandBytes--;
        // printf("commbytes: %d\n",  commandBytes);
        return;
    }

    assert(0);
}

uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {

    assert (spiCommand == -1);

    if (cmd == CMD13) {
        spiCommand = CMD13;
        return 0; // OK
    }

    assert(0);
}

bool Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin) {

    assert(swapFile == NULL);

    errorCode_ = 0;
    type_ = 3;

    //
    // XXX not possible to open a file with O_CREAT | O_APPEND | O_RDWR | O_TRUNC with fopen.
    // Best match is mode "a+": O_RDWR|O_CREAT|O_APPEND.
    // So simulate the missing O_TRUNC flag by an additional fopen/fclose call:
    //
    // swapFile = fopen("swapfile", "w");
    // fclose(swapFile);

    // assert((swapFile = fopen("swapfile", "a+")) != NULL);
    assert((swapFile = fopen("swapfile", "w+")) != NULL);

    return true;
}

bool Sd2Card::writeData(uint8_t token, const uint8_t* src) {

    // printf("Write pos: 0x%x\n", writePos);

    if (fseek(swapFile, writePos << 9, SEEK_SET) != 0 ) {
        assert(0);
    }

    int written = fwrite(src, 1, 512, swapFile);
    
    fflush(swapFile);
    return (written == 512);
}

bool Sd2Card::readBlock(uint32_t blockNumber, uint8_t* dst) {

    if (fseek(swapFile, blockNumber << 9, SEEK_SET) != 0 ) {
        assert(0);
    }

    int16_t c = fread(dst, 1, 512, swapFile);
    return c >= 0;
}

//------------------------------------------------------------------------------
void Sd2Card::chipSelectHigh() {
  chipSelect = true;
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow() {
  chipSelect = false;
}
//------------------------------------------------------------------------------
uint32_t Sd2Card::cardSize() {
  return 4194304; // 2Gb in 512byte blocks
}
//------------------------------------------------------------------------------
bool Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock) {
    return true;
}
//------------------------------------------------------------------------------
bool Sd2Card::eraseSingleBlockEnable() {
    return true;
}
//------------------------------------------------------------------------------
//
//
//
//
//
//
//
//
//
//
//
//
//
#if 0
static bool writeError = false;
static int errorcode = 0;


CardReader::CardReader()
{
   printf("new CardReader\n");

   sdReadPos  = 0;
   sdprinting = false;
   pause = false;
   cardOK = true;
   // logging = false;
   // autostart_atmillis=0;
   // workDirDepth = 0;
   // memset(workDirParents, 0, sizeof(workDirParents));

   // autostart_stilltocheck=true; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.
   // lastnr=0;
  //power to SD reader
  // #if SDPOWER > -1
    // SET_OUTPUT(SDPOWER);
    // WRITE(SDPOWER,HIGH);
  // #endif //SDPOWER

    // autostart_atmillis=millis()+5000;

    opencount = 0;
    sdInserted = true;
    saving = false;

    file = NULL;

    filename[0] = '\0';
}

void CardReader::initsd()
{

  if (cardOK) {
    SERIAL_ECHOLNPGM("WARNING: initsd() already done");
    return;
  }

  cardOK = false;
#if 0
  if(root.isOpen())
    root.close();
#endif

  if (! sdInserted) {
    SERIAL_ECHOLNPGM("SD init fail, no card");
    return;
  }

#if 0
#ifdef SDSLOW
  if (!card.init(SPI_HALF_SPEED,SDSS))
#else
  if (!card.init(SPI_FULL_SPEED,SDSS))
#endif
  {
    //if (!card.init(SPI_HALF_SPEED,SDSS))
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
  }
  else if (!volume.init(&card))
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_VOL_INIT_FAIL);
  }
  else if (!root.openRoot(&volume))
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
  }
  else
  {
#endif

    cardOK = true;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
#if 0
  }
  workDir=root;
  curDir=&root;
  /*
  if(!workDir.openRoot(&volume))
  {
    SERIAL_ECHOLNPGM(MSG_SD_WORKDIR_FAIL);
  }
  */
#endif
}

bool CardReader::isFileOpen() { return file != NULL; }

void CardReader::openFile(const char* name, bool read)
{

  if (opencount == 0 && isFileOpen()) {
    SERIAL_ERRORLNPGM("ERROR: openFile(): file already opened!");
    fclose(file);
  }

  // sdprinting = false;
  // pause = false;

  // SdFile myDir;
  // curDir=&root;
  const char *fname=name;

  assert(strlen(name) < 13);
  strcpy(filename, name);

  // char *dirname_start,*dirname_end;
  if(name[0]=='/')
  {
    assert(0);

    #if 0
    dirname_start=strchr(name,'/')+1;
    while(dirname_start>(char*)1)
    {
      dirname_end=strchr(dirname_start,'/');
      //SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
      //SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
      if(dirname_end>0 && dirname_end>dirname_start)
      {
        char subdirname[13];
        strncpy(subdirname, dirname_start, dirname_end-dirname_start);
        subdirname[dirname_end-dirname_start]=0;
        SERIAL_ECHOLN(subdirname);
        if(!myDir.open(curDir,subdirname,O_READ))
        {
          SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
          SERIAL_PROTOCOL(subdirname);
          SERIAL_PROTOCOLLNPGM(".");
          return;
        }
        else
        {
          //SERIAL_ECHOLN("dive ok");
        }

        curDir=&myDir;
        dirname_start=dirname_end+1;
      }
      else // the reminder after all /fsa/fdsa/ is the filename
      {
        fname=dirname_start;
        //SERIAL_ECHOLN("remaider");
        //SERIAL_ECHOLN(fname);
        break;
      }

    }
    #endif
  }
  else //relative path
  {
    // curDir=&workDir;
  }

  if(read)
  {
    // Don't open for reading if already opened for RW
    if (opencount || (file = fopen(fname, "r")) != NULL)
    {
      SERIAL_PROTOCOLPGM(MSG_SD_FILE_OPENED);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLPGM(MSG_SD_SIZE);
      SERIAL_PROTOCOLLN(getFileSize());
      sdReadPos  = 0;

      SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);

      opencount++;
      // xxx debug
      char buffer[64];
      sprintf(buffer, "openfile(%s), opencount:  %d", name, opencount);
      SERIAL_ECHOLN(buffer);
    }
    else
    {
      SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLLNPGM(".");
    }
  }
  else
  { //write
    // Mod ERRI: open in Read/Write mode
    // if (!file.open(curDir, fname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
    // if ((file = open(fname, O_CREAT | O_APPEND | O_RDWR | O_TRUNC, 0666)) <= 0)

    //
    // XXX not possible to open a file with O_CREAT | O_APPEND | O_RDWR | O_TRUNC with fopen.
    // Best match is mode "a+": O_RDWR|O_CREAT|O_APPEND.
    // So simulate the missing O_TRUNC flag by an additional fopen/fclose call:
    //
    FILE * f = fopen(fname, "w");
    fclose(f);

    // if ((file = fopen(fname, "w+")) == NULL)
    if ((file = fopen(fname, "a+")) == NULL)
    {
      SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLLNPGM(".");
    }
    else
    {
      saving = true;
      SERIAL_PROTOCOLPGM(MSG_SD_WRITE_TO_FILE);
      SERIAL_PROTOCOLLN(name);

      sdReadPos  = 0;
      opencount++;
      // xxx debug
      char buffer[64];
      sprintf(buffer, "openfile(%s), opencount:  %d", name, opencount);
      SERIAL_ECHOLN(buffer);
    }
  }
}

void CardReader::closefile()
{

  // sync();

  // saving = false;
  // logging = false;

  if (opencount < 1) {
    SERIAL_ERRORLNPGM("CardReader::closefile(): file not open.");
    return;
  }

  opencount--;

  // xxx debug
  char buffer[64];
  // file.getFilename(fn), 
  sprintf(buffer, "closefile(%s), opencount:  %d", filename, opencount);
  SERIAL_ECHOLN(buffer);

  filename[0] = '\0';

  if (! opencount) {
    assert(fclose(file) == 0);
    file = NULL;
  }
}

bool CardReader::write_string(char* buffer, uint16_t len) {

    int written = fwrite(buffer, 1, len, file);

    if (written != len) {
        assert(0);
    }

    return writeError;
}

#if 0
bool CardReader::write_string(char* buffer)
{
    return write_string(buffer, strlen(buffer));
}
#endif

uint32_t  CardReader::getFileSize() { 
        
        if (isFileOpen()) {
            // xxx sync?
            // struct stat st;
            // assert( fstat(file, &st) == 0);
            // return st.st_size;
            int prev=ftell(file);
            fseek(file, 0L, SEEK_END);
            int sz=ftell(file);
            fseek(file,prev,SEEK_SET);
            return sz;
        }

        // Aufruf mit geschlossenem file ist wohr mit der SdFile implementierung
        // möglich, desswegen hier folgender workaround:
        return sdReadPos;
    }

bool CardReader::isOk() { return true; }

int16_t CardReader::read(void *buffer, uint16_t len) {

    if (fseek(file, sdReadPos, SEEK_SET) != 0 )
        MSerial.print("fgets: can't seek\n");

    int16_t c = fread(buffer, 1, len, file);
    if (c != -1)
      sdReadPos += c;
    return c;
}
#endif

