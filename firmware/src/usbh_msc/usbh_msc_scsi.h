//--------------------------------------------------------------
// File     : usbh_msc_scsi.h
//--------------------------------------------------------------

#ifndef __USBH_MSC_SCSI_H__
#define __USBH_MSC_SCSI_H__

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "usbh_stdreq.h"

typedef enum {
  USBH_MSC_OK = 0,
  USBH_MSC_FAIL = 1,
  USBH_MSC_PHASE_ERROR = 2,
  USBH_MSC_BUSY = 3
}USBH_MSC_Status_TypeDef;

typedef enum {
  CMD_UNINITIALIZED_STATE =0,
  CMD_SEND_STATE,
  CMD_WAIT_STATUS,
  CMD_STORAGE_RESET,
  CMD_SEND_RESET,
  CMD_CLEAR_FEATURE_IN,
  CMD_CLEAR_FEATURE_OUT,
  CMD_TEST_CLEAR_FEATURE_OUT,
} CMD_STATES_TypeDef;  

typedef struct __MassStorageParameter
{
  uint32_t MSCapacity;
  uint32_t MSSenseKey; 
  uint16_t MSPageLength;
  uint8_t MSBulkOutEp;
  uint8_t MSBulkInEp;
  uint8_t MSWriteProtect;
} MassStorageParameter_TypeDef;


#define OPCODE_TEST_UNIT_READY            0X00
#define OPCODE_INQUIRY                    0x12
#define OPCODE_READ_CAPACITY10            0x25
#define OPCODE_MODE_SENSE6                0x1A
#define OPCODE_READ10                     0x28
#define OPCODE_WRITE10                    0x2A
#define OPCODE_REQUEST_SENSE              0x03

#define DESC_REQUEST_SENSE                0X00
#define ALLOCATION_LENGTH_REQUEST_SENSE   14 
#define XFER_LEN_READ_CAPACITY10           8
#define XFER_LEN_MODE_SENSE6              63

#define MASK_MODE_SENSE_WRITE_PROTECT     0x80
#define MODE_SENSE_PAGE_CONTROL_FIELD     0x00
#define MODE_SENSE_PAGE_CODE              0x3F
#define DISK_WRITE_PROTECTED              0x01

USBH_Status USBH_MSC_TestUnitReady(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);
USBH_Status USBH_MSC_ReadCapacity10(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);

extern USBH_Status USBH_MSC_Write10(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost,
                         uint8_t *,
                         uint32_t ,
                         uint32_t);
extern USBH_Status USBH_MSC_Read10(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost,
                        uint8_t *,
                        uint32_t ,
                        uint32_t );
extern USBH_Status USBH_MSC_BlockReset(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);

void USBH_MSC_StateMachine(USB_OTG_CORE_HANDLE *pdev);

#endif  //__USBH_MSC_SCSI_H__



