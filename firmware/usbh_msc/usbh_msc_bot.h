//--------------------------------------------------------------
// File     : usbh_msc_bot.h
//--------------------------------------------------------------


#ifndef __USBH_MSC_BOT_H__
#define __USBH_MSC_BOT_H__

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "usbh_stdreq.h"


#define USBH_MSC_MPS_SIZE                 0x40

#define CBW_LENGTH                        10
#define CBW_CB_LENGTH_16                  16
#define USBH_MSC_BOT_CBW_PACKET_LENGTH_31 31  // this should be sizeof(struct __CBW) = 15+16

typedef union _USBH_CBW_Block
{
  struct __CBW
  {
    uint32_t CBWSignature;
    uint32_t CBWTag;
    uint32_t CBWTransferLength;
    uint8_t CBWFlags;
    uint8_t CBWLUN; 
    uint8_t CBWLength;              // this field specifies the length (in bytes) of the
                                    // command CBWCB (-array).
    uint8_t CBWCB[CBW_CB_LENGTH_16];// CBWCB: the command block to be executed by the device.
  }field;
  uint8_t CBWArray[USBH_MSC_BOT_CBW_PACKET_LENGTH_31];
}HostCBWPkt_TypeDef;

#define USBH_MSC_CSW_LENGTH_13            13  

typedef union _USBH_CSW_Block
{
  struct __CSW
  {
    uint32_t CSWSignature;
    uint32_t CSWTag;
    uint32_t CSWDataResidue;
    uint8_t  CSWStatus;
  }field;
  uint8_t CSWArray[USBH_MSC_CSW_LENGTH_13];
}HostCSWPkt_TypeDef;

#define DATA_LEN_INQUIRY                    36U

typedef enum {
  USBH_MSC_BOT_INIT_STATE = 0,                
  USBH_MSC_BOT_READ_INQUIRY,
  USBH_MSC_TEST_UNIT_READY,          
  USBH_MSC_READ_CAPACITY10,
  USBH_MSC_DEFAULT_APPLI_STATE,  // == IDLE
  USBH_MSC_UNRECOVERED_STATE
} MSCStateType;

typedef enum {
     USBH_BOTSTATE_SENT_CBW,
     USBH_BOTSTATE_BOT_DATAIN_STATE,
     USBH_BOTSTATE_BOT_DATAIN_WAIT_STATE,
     USBH_BOTSTATE_BOT_DATAOUT_STATE,
     USBH_BOTSTATE_BOT_DATAOUT_WAIT_STATE,
     USBH_BOTSTATE_RECEIVE_CSW_STATE,
     USBH_BOTSTATE_DECODE_CSW,
     USBH_BOTSTATE_BOT_ERROR_IN,
     USBH_BOTSTATE_BOT_ERROR_OUT,
} BOTStateType;


typedef struct _BOTXfer
{
    MSCStateType MSCState;
    CMD_STATES_TypeDef CmdStateMachine;
    BOTStateType BOTState;
    BOTStateType isthis_needed_BOTStateBkp;
    uint8_t* pRxTxBuff;
    uint16_t DataLength;
    USBH_MSC_Status_TypeDef isthis_needed_BOTXferStatus;
    // Buffer for small (one packet) requests
    uint8_t packetBuffer[USBH_MSC_MPS_SIZE];
} USBH_BOTXfer_TypeDef;



#define USBH_MSC_BOT_CBW_SIGNATURE        0x43425355
#define USBH_MSC_BOT_CBW_TAG              0x20304050             
#define USBH_MSC_BOT_CSW_SIGNATURE        0x53425355           
// #define USBH_MSC_CSW_DATA_LENGTH          0x000D

/* CSW Status Definitions */
#define USBH_MSC_CSW_CMD_PASSED           0x00
#define USBH_MSC_CSW_CMD_FAILED           0x01
#define USBH_MSC_CSW_PHASE_ERROR          0x02

#define USBH_MSC_SEND_CSW_DISABLE         0
#define USBH_MSC_SEND_CSW_ENABLE          1

#define USBH_MSC_DIR_IN                   0
#define USBH_MSC_DIR_OUT                  1
#define USBH_MSC_BOTH_DIR                 2

//#define USBH_MSC_PAGE_LENGTH                 0x40
#define USBH_MSC_PAGE_LENGTH              512


#define CBW_LENGTH_TEST_UNIT_READY         6

#define USB_REQ_BOT_RESET                0xFF
#define USB_REQ_GET_MAX_LUN              0xFE

#define MAX_BULK_STALL_COUNT_LIMIT       0x04   /* If STALL is seen on Bulk 
                                         Endpoint continously, this means 
                                         that device and Host has phase error
                                         Hence a Reset is needed */


extern USBH_BOTXfer_TypeDef USBH_MSC_BOTXferParam;
 
USBH_Status USBH_MSC_HandleBOTXfer(USB_OTG_CORE_HANDLE *pdev,
                            USBH_HOST *phost);
uint8_t USBH_MSC_DecodeCSW(USB_OTG_CORE_HANDLE *pdev,
                           USBH_HOST *phost);
void USBH_MSC_Init(USB_OTG_CORE_HANDLE *pdev);
USBH_Status USBH_MSC_BOT_Abort(USB_OTG_CORE_HANDLE *pdev, 
                               USBH_HOST *phost,
                               uint8_t direction);
 

#endif  //__USBH_MSC_BOT_H__



