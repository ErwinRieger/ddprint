
#include <stdint.h>
#include <string.h> 

#include <libmaple/gpio.h>

extern "C" {
#include <usb_hcd_int.h>
// usbF4/STM32_USB_OTG_Driver/inc/usb_bsp.h
#include <usb_bsp.h>
}

#include "usb_hcd.h"
#include "usbh_usr.h"
#include "dd_usbh_msc.h"
#include "mdebug.h"

//xxx
extern void USB_OTG_BSP_mDelay (const uint32_t msec);
uint32_t dd_HCD_SubmitRequest (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num);
uint32_t dd_HCD_IsDeviceConnected    (USB_OTG_CORE_HANDLE *pdev); 
uint32_t dd_HCD_GetXferCnt (USB_OTG_CORE_HANDLE *pdev, uint8_t ch_num);
uint32_t dd_HCD_Init(USB_OTG_CORE_HANDLE *pdev , USB_OTG_CORE_ID_TypeDef coreID);
URB_STATE dd_HCD_GetURB_State (USB_OTG_CORE_HANDLE *pdev , uint8_t ch_num);
uint32_t dd_HCD_GetCurrentFrame (USB_OTG_CORE_HANDLE *pdev) ;
uint32_t dd_HCD_GetCurrentSpeed (USB_OTG_CORE_HANDLE *pdev);
uint32_t dd_HCD_Init(USB_OTG_CORE_HANDLE *pdev , USB_OTG_CORE_ID_TypeDef coreID);
uint32_t dd_HCD_ResetPort(USB_OTG_CORE_HANDLE *pdev);

//
// Todo:
// * Code assumes sector size of 512 (USBH_MSC_PAGE_LENGTH), add a check for this
//   (USBH_MSC_Param.MSPageLength).
//
//

//--------------------------------------------------------------
//
USB_OTG_CORE_HANDLE     USB_OTG_Core_Host; // pdev, USB_OTG_dev
USBH_HOST               USB_Host;

static USBH_BOTXfer_TypeDef usbh_msc; 

static HostCBWPkt_TypeDef USBH_MSC_CBWData;
static HostCSWPkt_TypeDef USBH_MSC_CSWData;

static MSC_Machine_TypeDef         MSC_Machine;
static MassStorageParameter_TypeDef USBH_MSC_Param; 

static SCSI_StdInquiryDataTypeDef inquiry;

// Sense data.
typedef struct
{
  uint8_t error;   
  uint8_t key;   
  uint8_t asc;   
  uint8_t ascq;  
} SCSI_SenseTypeDef;

static SCSI_SenseTypeDef sensKey;

// debug
struct DebugCalls {
    int unitReadyCalls;
    int reqSenseCalls;
    USBH_Status USBH_MSC_HandleBOTXferStatus;
    int stallErrors;
};

static struct DebugCalls debugCalls;

//--------------------------------------------------------------
//
bool usbhMscInitialized() {
    return usbh_msc.MSCState == USBH_MSC_DEFAULT_APPLI_STATE;
}

//--------------------------------------------------------------
//
uint32_t usbhMscSizeInBlocks() {
    return USBH_MSC_Param.MSCapacity;
}
 

//--------------------------------------------------------------
//
// Generic usb error handler
//
// Todo: use some error code to better describe error reason.
//
void usbMSCHostError() {

    massert(0);
}

//--------------------------------------------------------------
//
USBH_Status USB_disk_write(
        USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost,
        uint8_t *buff, uint32_t sector) {

    USBH_Status status;
    
    while (true) {
        status = USBH_MSC_Write10(pdev, phost, buff, sector, 512);
        if (status != USBH_BUSY)
            break;
    }
    return status;
}

//--------------------------------------------------------------
//
USBH_Status USB_disk_read(
        USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost,
        uint8_t *buff, uint32_t sector) {

    USBH_Status status;

    while (true) {
        status = USBH_MSC_Read10(pdev, phost, buff, sector, 512);
        if (status != USBH_BUSY)
            break;
    }

  return status;
}

//--------------------------------------------------------------
//
USBH_Status dd_USBH_BulkSendData ( USB_OTG_CORE_HANDLE *pdev, 
                                uint8_t *buff, 
                                uint16_t length,
                                uint8_t hc_num)
{ 
  pdev->host.hc[hc_num].ep_is_in = 0;
  pdev->host.hc[hc_num].xfer_buff = buff;
  pdev->host.hc[hc_num].xfer_len = length;  

  /* Set the Data Toggle bit as per the Flag */
  if ( pdev->host.hc[hc_num].toggle_out == 0)
  { /* Put the PID 0 */
      pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;    
  }
  else
  { /* Put the PID 1 */
      pdev->host.hc[hc_num].data_pid = HC_PID_DATA1 ;
  }

  dd_HCD_SubmitRequest (pdev , hc_num);   
  return USBH_OK;
}

//--------------------------------------------------------------
//
/**
  * @brief  Returns the current toggle of a pipe.
  * @param  phost: Host handle
  * @param  pipe: Pipe index
  * @retval toggle (0/1)
  */
uint8_t USBH_LL_GetToggle(USB_OTG_CORE_HANDLE *pdev, uint8_t pipe)
{
  uint8_t toggle = 0;

  if(pdev->host.hc[pipe].ep_is_in)
  {
      toggle = pdev->host.hc[pipe].toggle_in;
  }
  else
  {
      toggle = pdev->host.hc[pipe].toggle_out;
  }

  return toggle; 
}

//--------------------------------------------------------------
//
/**
  * @brief  Sets toggle for a pipe.
  * @param  phost: Host handle
  * @param  pipe: Pipe index   
  * @param  toggle: toggle (0/1)
  * @retval USBH Status
  */
void USBH_LL_SetToggle(USB_OTG_CORE_HANDLE *pdev, uint8_t pipe, uint8_t toggle)
{

    if(pdev->host.hc[pipe].ep_is_in)
    {
      pdev->host.hc[pipe].toggle_in = toggle;
    }
    else
    {
      pdev->host.hc[pipe].toggle_out = toggle;
    }
}

//--------------------------------------------------------------
//
USBH_Status dd_USBH_MSC_HandleBOTXfer (USB_OTG_CORE_HANDLE *pdev ,USBH_HOST *phost)
{
  uint8_t xferDirection;
  USBH_Status status = USBH_BUSY;
  
  URB_STATE URB_Status = URB_IDLE;
  
  switch (usbh_msc.BOTState)
  {
    case USBH_BOTSTATE_SENT_CBW:

      URB_Status = dd_HCD_GetURB_State(pdev, MSC_Machine.hc_num_out);
      
      if(URB_Status == URB_DONE)
      { 

        /* If the CBW Pkt is sent successful, then change the state */
        xferDirection = (USBH_MSC_CBWData.field.CBWFlags & USB_REQ_DIR_MASK);
        
        if ( USBH_MSC_CBWData.field.CBWTransferLength != 0 )
        {
         
          /* If there is Data Transfer Stage */
          if (xferDirection == USB_D2H)
          {
            /* Data Direction is IN */
            usbh_msc.BOTState = USBH_BOTSTATE_BOT_DATAIN_STATE;
          }
          else
          {
            /* Data Direction is OUT */
            usbh_msc.BOTState = USBH_BOTSTATE_BOT_DATAOUT_STATE;
          } 

          // firstBlock = true; 
        }
        
        else
        {/* If there is NO Data Transfer Stage */
          usbh_msc.BOTState = USBH_BOTSTATE_RECEIVE_CSW_STATE;
        }
      }   
      else if (URB_Status == URB_NOTREADY)
      {
            massert(0);
      }     
      else if(URB_Status >= URB_ERROR) // error or stall
      {
        massert(0);
      }
      break;

    case USBH_BOTSTATE_BOT_DATAIN_STATE:
      
        USBH_BulkReceiveData (
                pdev, usbh_msc.pRxTxBuff, 
                USBH_MSC_MPS_SIZE, MSC_Machine.hc_num_in);

        usbh_msc.BOTState = USBH_BOTSTATE_BOT_DATAIN_WAIT_STATE;
        break;   
      
    case USBH_BOTSTATE_BOT_DATAIN_WAIT_STATE:
      
      URB_Status = dd_HCD_GetURB_State(pdev , MSC_Machine.hc_num_in);

      /* BOT DATA IN stage */
      if (URB_Status == URB_DONE)
      {
        if (USBH_MSC_CBWData.field.CBWTransferLength > USBH_MSC_MPS_SIZE) {
            // More data left, back to datain_state for next packet
            usbh_msc.BOTState = USBH_BOTSTATE_BOT_DATAIN_STATE;
            usbh_msc.pRxTxBuff += USBH_MSC_MPS_SIZE;
            USBH_MSC_CBWData.field.CBWTransferLength -= USBH_MSC_MPS_SIZE;
        }
        else {
            // USBH_MSC_CBWData.field.CBWTransferLength = 0;
            // Data transferred, switch to next state
            usbh_msc.BOTState = USBH_BOTSTATE_RECEIVE_CSW_STATE;
        }
      }
      else if(URB_Status == URB_NOTREADY)
      {
        massert(0);
      }     
      // else if(URB_Status == URB_STALL)
      else if (URB_Status >= URB_ERROR) // error or stall
      {
        massert(0);
        /* This is Data Stage STALL Condition */
        // error_direction = USBH_MSC_DIR_IN;
        usbh_msc.BOTState  = USBH_BOTSTATE_BOT_ERROR_IN;
        
        /* Refer to USB Mass-Storage Class : BOT (www.usb.org) 
        6.7.2 Host expects to receive data from the device
        3. On a STALL condition receiving data, then:
        The host shall accept the data received.
        The host shall clear the Bulk-In pipe.
        4. The host shall attempt to receive a CSW.
        */
      }     
      break;   
      
    case USBH_BOTSTATE_BOT_DATAOUT_STATE:

        dd_USBH_BulkSendData (pdev,
                           usbh_msc.pRxTxBuff, 
                           USBH_MSC_MPS_SIZE, MSC_Machine.hc_num_out);

        usbh_msc.BOTState = USBH_BOTSTATE_BOT_DATAOUT_WAIT_STATE;
        break;

    case USBH_BOTSTATE_BOT_DATAOUT_WAIT_STATE:

      URB_Status = dd_HCD_GetURB_State(pdev , MSC_Machine.hc_num_out);       

      /* BOT DATA OUT stage */
      if(URB_Status == URB_DONE)
      {
        if (USBH_MSC_CBWData.field.CBWTransferLength > USBH_MSC_MPS_SIZE) {
            // More data left, back to dataout_state for next packet
            usbh_msc.pRxTxBuff += USBH_MSC_MPS_SIZE;
            USBH_MSC_CBWData.field.CBWTransferLength -= USBH_MSC_MPS_SIZE;
            usbh_msc.BOTState = USBH_BOTSTATE_BOT_DATAOUT_STATE;
        }
        else {
            // USBH_MSC_CBWData.field.CBWTransferLength = 0;
            // Data transferred, switch to next state
            usbh_msc.BOTState = USBH_BOTSTATE_RECEIVE_CSW_STATE;
        }
      }
      else if(URB_Status == URB_NOTREADY)
      {
        /* Nack received from device, resend last packet */
        usbh_msc.BOTState = USBH_BOTSTATE_BOT_DATAOUT_STATE;
      }
      else if (URB_Status == URB_STALL)
      {
        massert(0);
        // error_direction = USBH_MSC_DIR_OUT;
        usbh_msc.BOTState  = USBH_BOTSTATE_BOT_ERROR_OUT;
        
        /* Refer to USB Mass-Storage Class : BOT (www.usb.org) 
        6.7.3 Ho - Host expects to send data to the device
        3. On a STALL condition sending data, then:
        " The host shall clear the Bulk-Out pipe.
        4. The host shall attempt to receive a CSW.
        
        The Above statement will do the clear the Bulk-Out pipe.
        The Below statement will help in Getting the CSW.  
        */
      }
      break;

    case USBH_BOTSTATE_RECEIVE_CSW_STATE:

      /* BOT CSW stage */     
        // usbh_msc.pRxTxBuff = USBH_MSC_CSWData.CSWArray;  // 13
        // usbh_msc.DataLength = USBH_MSC_CSW_MAX_LENGTH;   // 63
        // usbh_msc.DataLength = USBH_MSC_CSW_LENGTH_13;   // 13
        
        // for(index = USBH_MSC_CSW_LENGTH_13; index != 0; index--)
        // {
          // USBH_MSC_CSWData.CSWArray[index] = 0;
        // }
        
        // USBH_MSC_CSWData.CSWArray[0] = 0; 
        
        USBH_BulkReceiveData (
                pdev, USBH_MSC_CSWData.CSWArray,
                USBH_MSC_CSW_LENGTH_13, MSC_Machine.hc_num_in);

        usbh_msc.BOTState = USBH_BOTSTATE_DECODE_CSW;    

      break;
      
    case USBH_BOTSTATE_DECODE_CSW:

      URB_Status = dd_HCD_GetURB_State(pdev , MSC_Machine.hc_num_in);

      /* Decode CSW */
      if(URB_Status == URB_DONE)
      {
        // usbh_msc.MSCState = usbh_msc.MSCStateCurrent ;
        
        USBH_MSC_Status_TypeDef decodeState = USBH_MSC_DecodeCSW(pdev , phost);

        // Note: USBH_MSC_BUSY never returned by USBH_MSC_DecodeCSW
        if (decodeState == USBH_MSC_OK) {
            status = USBH_OK;
        }
        else 
        {
            status = USBH_FAIL;
        }
      }
      else if(URB_Status == URB_NOTREADY)
      {
        massert(0);
      }     
      else if(URB_Status == URB_ERROR) // error or stall
      {
          massert(0);
          usbh_msc.BOTState  = USBH_BOTSTATE_BOT_ERROR_IN;
      }
      else if(URB_Status == URB_STALL) {
          usbh_msc.BOTState  = USBH_BOTSTATE_BOT_ERROR_OUT;
          debugCalls.stallErrors++;
      }
      break;

    case USBH_BOTSTATE_BOT_ERROR_IN: 

      status = USBH_MSC_BOT_Abort(pdev, phost, USBH_MSC_DIR_IN);
      if (status == USBH_OK)
      {
        // /* Check if the error was due in Both the directions */
        // if (error_direction == USBH_MSC_BOTH_DIR)
        // {/* If Both directions are Needed, Switch to OUT Direction */
          // usbh_msc.BOTState = USBH_BOTSTATE_BOT_ERROR_OUT;
        // }
        // else
        // {
          // /* Switch Back to the Original State, In many cases this will be 
          // USBH_BOTSTATE_RECEIVE_CSW_STATE state */
        // }
        // status = USBH_FAIL; // restart upper level command
        usbh_msc.BOTState  = USBH_BOTSTATE_RECEIVE_CSW_STATE;
        status = USBH_BUSY;
      }
      // else if (URB_Status == USBH_UNRECOVERED_ERROR)
      // {
        // /* This means that there is a STALL Error limit, Do Reset Recovery */
        // usbh_msc.isthis_needed_BOTXferStatus = USBH_MSC_PHASE_ERROR;
      // }
      else if (status == USBH_UNRECOVERED_ERROR)
      {
          massert(0);
      }
      else {      
          usbMSCHostAssert(status == USBH_BUSY);
      }
      break;
      
    case USBH_BOTSTATE_BOT_ERROR_OUT: 

      status = USBH_MSC_BOT_Abort(pdev, phost, USBH_MSC_DIR_OUT);

      if ( status == USBH_OK)
      { /* Switch Back to the Original State */
        // status = USBH_FAIL; // restart upper level command
        // status = USBH_ERROR_SPEED_UNKNOWN; // xxxx restart

        uint8_t toggle = USBH_LL_GetToggle(pdev, MSC_Machine.hc_num_out);
        USBH_LL_SetToggle(pdev, MSC_Machine.hc_num_out, 1 - toggle);
        USBH_LL_SetToggle(pdev, MSC_Machine.hc_num_in, 0);

        usbh_msc.BOTState  = USBH_BOTSTATE_BOT_ERROR_IN;
        status = USBH_BUSY;
      }
      else {      
          usbMSCHostAssert(status == USBH_BUSY);
      }
      // else if (status == USBH_UNRECOVERED_ERROR)
      // {
        // /* This means that there is a STALL Error limit, Do Reset Recovery */
        // usbh_msc.isthis_needed_BOTXferStatus = USBH_MSC_PHASE_ERROR;
      // }
      break;

    default:      
      usbMSCHostAssert(0);
      break;
  }

  debugCalls.USBH_MSC_HandleBOTXferStatus = status;

  return status;
}

//--------------------------------------------------------------
//
USBH_Status USBH_MSC_Read10(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost,
                        uint8_t *dataBuffer,
                        uint32_t address,
                        uint32_t nbOfbytes)
{
  USBH_Status bot_status;
  uint16_t nbOfPages;
  
    switch(usbh_msc.CmdStateMachine) {

    case CMD_SEND_STATE:
      USBH_MSC_CBWData.field.CBWTransferLength = nbOfbytes;
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
      USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH; // 10
     
      memset(USBH_MSC_CBWData.field.CBWCB, 0x00, CBW_LENGTH);
      USBH_MSC_CBWData.field.CBWCB[0]  = OPCODE_READ10; 

      /*logical block address*/
      USBH_MSC_CBWData.field.CBWCB[2]  = (((uint8_t*)&address)[3]);
      USBH_MSC_CBWData.field.CBWCB[3]  = (((uint8_t*)&address)[2]);
      USBH_MSC_CBWData.field.CBWCB[4]  = (((uint8_t*)&address)[1]);
      USBH_MSC_CBWData.field.CBWCB[5]  = (((uint8_t*)&address)[0]);
      
      /*USBH_MSC_PAGE_LENGTH = 512*/
      nbOfPages = nbOfbytes / USBH_MSC_PAGE_LENGTH;  
      
      /*Tranfer length */
      USBH_MSC_CBWData.field.CBWCB[7]  = (((uint8_t *)&nbOfPages)[1]) ; 
      USBH_MSC_CBWData.field.CBWCB[8]  = (((uint8_t *)&nbOfPages)[0]) ; 
      
      // Handle state machines:
      // Leave upper level states unchanged,
      // Switch our level states to 'wait'.
      usbh_msc.CmdStateMachine = CMD_WAIT_STATUS;
      // Initialize lower level transfer state to 'cbw sent'
      usbh_msc.BOTState = USBH_BOTSTATE_SENT_CBW;

      usbh_msc.pRxTxBuff = dataBuffer;

      dd_USBH_BulkSendData (pdev,
                         USBH_MSC_CBWData.CBWArray,          // 31, entire CBW struct
                         USBH_MSC_BOT_CBW_PACKET_LENGTH_31 , // 31
                         MSC_Machine.hc_num_out);
      
      return USBH_BUSY;
      
    case CMD_WAIT_STATUS:
   
      /* Process the BOT state machine */
      bot_status = dd_USBH_MSC_HandleBOTXfer(pdev, phost);

      if (bot_status != USBH_BUSY) {
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }

      return bot_status;
      
    default:
      return USBH_NOT_SUPPORTED;
    }
}

//--------------------------------------------------------------
//
USBH_Status USBH_MSC_Write10(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost,
                         uint8_t *dataBuffer,
                         uint32_t address,
                         uint32_t nbOfbytes)
{
  USBH_Status bot_status;
  uint16_t nbOfPages;
  
  switch(usbh_msc.CmdStateMachine) {

    case CMD_SEND_STATE:   
      USBH_MSC_CBWData.field.CBWTransferLength = nbOfbytes;
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_OUT;
      USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH; // 10

      memset(USBH_MSC_CBWData.field.CBWCB, 0x00, CBW_LENGTH);
      USBH_MSC_CBWData.field.CBWCB[0]  = OPCODE_WRITE10; 

      /*logical block address*/
      USBH_MSC_CBWData.field.CBWCB[2]  = (((uint8_t*)&address)[3]) ;
      USBH_MSC_CBWData.field.CBWCB[3]  = (((uint8_t*)&address)[2]);
      USBH_MSC_CBWData.field.CBWCB[4]  = (((uint8_t*)&address)[1]);
      USBH_MSC_CBWData.field.CBWCB[5]  = (((uint8_t*)&address)[0]);
      
      /*USBH_MSC_PAGE_LENGTH = 512*/
      nbOfPages = nbOfbytes / USBH_MSC_PAGE_LENGTH; 
      
      /*Tranfer length */
      USBH_MSC_CBWData.field.CBWCB[7]  = (((uint8_t *)&nbOfPages)[1]) ; 
      USBH_MSC_CBWData.field.CBWCB[8]  = (((uint8_t *)&nbOfPages)[0]) ; 
      
      // Handle state machines:
      // Leave upper level states unchanged,
      // Switch our level states to 'wait'.
      usbh_msc.CmdStateMachine = CMD_WAIT_STATUS;
      // Initialize lower level transfer state to 'cbw sent'
      usbh_msc.BOTState = USBH_BOTSTATE_SENT_CBW;

      usbh_msc.pRxTxBuff = dataBuffer;

      dd_USBH_BulkSendData (pdev,
                         USBH_MSC_CBWData.CBWArray,          // 31, entire CBW struct
                         USBH_MSC_BOT_CBW_PACKET_LENGTH_31 , // 31
                         MSC_Machine.hc_num_out);
      return USBH_BUSY;
      
    case CMD_WAIT_STATUS:

      /* Process the BOT state machine */
      bot_status = dd_USBH_MSC_HandleBOTXfer(pdev, phost);

      if (bot_status != USBH_BUSY) {
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }

      return bot_status;
      
    default:
      return USBH_NOT_SUPPORTED;
  }
}

//--------------------------------------------------------------
//
USBH_Status USBH_MSC_BlockReset(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{


  USBH_Status bot_status;

  switch(usbh_msc.CmdStateMachine) {

    case CMD_SEND_STATE:   

      USBH_MSC_CBWData.field.CBWTransferLength = 0; // xxx not needed?
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_OUT;
      USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH; // 10

      // Rest set to 0x00 or 0xff ?
      memset(USBH_MSC_CBWData.field.CBWCB, 0xff, CBW_LENGTH);
      USBH_MSC_CBWData.field.CBWCB[0]  = 0x1D; 
      USBH_MSC_CBWData.field.CBWCB[1]  = 0x04; // special reset block mode

      // Handle state machines:
      // Leave upper level states unchanged,
      // Switch our level states to 'wait'.
      usbh_msc.CmdStateMachine = CMD_WAIT_STATUS;
      // Initialize lower level transfer state to 'cbw sent'
      usbh_msc.BOTState = USBH_BOTSTATE_SENT_CBW;

      dd_USBH_BulkSendData (pdev,
                         USBH_MSC_CBWData.CBWArray,          // 31, entire CBW struct
                         USBH_MSC_BOT_CBW_PACKET_LENGTH_31 , // 31
                         MSC_Machine.hc_num_out);
      return USBH_BUSY;
      
    case CMD_WAIT_STATUS:

      /* Process the BOT state machine */
      bot_status = dd_USBH_MSC_HandleBOTXfer(pdev, phost);

      if (bot_status != USBH_BUSY) {
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }

      return bot_status;
      
    default:
      return USBH_NOT_SUPPORTED;
  }
}

//--------------------------------------------------------------
//

void USB_OTG_BSP_ConfigVBUS(USB_OTG_CORE_HANDLE *pdev) {};
void USB_OTG_BSP_DriveVBUS(USB_OTG_CORE_HANDLE *pdev,uint8_t state) {};

//--------------------------------------------------------------
static uint32_t USB_OTG_USBH_handle_rx_qlvl_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_GRXFSTS_TypeDef       grxsts;
  USB_OTG_GINTMSK_TypeDef       intmsk;
  USB_OTG_HCTSIZn_TypeDef       hctsiz; 
  USB_OTG_HCCHAR_TypeDef        hcchar;
  __IO uint8_t                  channelnum =0;  
  uint32_t                      count;    
  
  /* Disable the Rx Status Queue Level interrupt */
  intmsk.d32 = 0;
  intmsk.b.rxstsqlvl = 1;
  USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);
  
  grxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GRXSTSP);
  channelnum = grxsts.b.chnum;  
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[channelnum]->HCCHAR);
  
  switch (grxsts.b.pktsts)
  {
  case GRXSTS_PKTSTS_IN:
  
    /* Read the data into the host buffer. */
    if ((grxsts.b.bcnt > 0) && (pdev->host.hc[channelnum].xfer_buff != (void  *)0))
    {  
      
      USB_OTG_ReadPacket(pdev, pdev->host.hc[channelnum].xfer_buff, grxsts.b.bcnt);
      /*manage multiple Xfer */
      pdev->host.hc[grxsts.b.chnum].xfer_buff += grxsts.b.bcnt;           
      pdev->host.hc[grxsts.b.chnum].xfer_count  += grxsts.b.bcnt;
     
      count = pdev->host.hc[channelnum].xfer_count;
      pdev->host.XferCnt[channelnum]  = count;
      
      hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[channelnum]->HCTSIZ);
      if(hctsiz.b.pktcnt > 0)
      {
        /* re-activate the channel when more packets are expected */
        hcchar.b.chen = 1;
        hcchar.b.chdis = 0;
        USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[channelnum]->HCCHAR, hcchar.d32);
      }
    }
    break;
    
    case GRXSTS_PKTSTS_IN_XFER_COMP:
    case GRXSTS_PKTSTS_DATA_TOGGLE_ERR:
    case GRXSTS_PKTSTS_CH_HALTED:
  default:
    break;
  }
  
  /* Enable the Rx Status Queue Level interrupt */
  intmsk.b.rxstsqlvl = 1;
  USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, 0, intmsk.d32);
  return 1;
}

//--------------------------------------------------------------
static uint32_t USB_OTG_USBH_handle_port_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_HPRT0_TypeDef  hprt0;
  USB_OTG_HPRT0_TypeDef  hprt0_dup;
  USB_OTG_HCFG_TypeDef   hcfg;    
  uint32_t do_reset = 0;
  uint32_t retval = 0;
 
  // hcfg.d32 = 0;
  hprt0.d32 = 0;
  hprt0_dup.d32 = 0;
  
  hprt0.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);
  hprt0_dup.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);
  
  /* Clear the interrupt bits in GINTSTS */
  hprt0_dup.b.prtena = 0;
  hprt0_dup.b.prtconndet = 0;
  hprt0_dup.b.prtenchng = 0;
  hprt0_dup.b.prtovrcurrchng = 0;
  
  /* Port Connect Detected */
  if (hprt0.b.prtconndet)
  {
    // CONNECT AppEvent
    // pdev->host.port_cb->Connect(pdev);
    hprt0_dup.b.prtconndet = 1;
    do_reset = 1;
    retval |= 1;
  }
  
  /* Port Enable Changed */
  if (hprt0.b.prtenchng)
  {

    hprt0_dup.b.prtenchng = 1;
    if (hprt0.b.prtena == 1)
    {
      pdev->host.ConnSts = 1;
      USB_OTG_BSP_mDelay(50); // from HAL_HCD_Connect_Callback()
     
      //
      // Support FULL_SPEED devices, only.
      //
      usbMSCHostAssert(hprt0.b.prtspd == HPRT0_PRTSPD_FULL_SPEED);

      USB_OTG_WRITE_REG32(&pdev->regs.HREGS->HFIR, 48000 );            

      hcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HCFG);

      if (hcfg.b.fslspclksel != HCFG_48_MHZ)
      {
        USB_OTG_InitFSLSPClkSel(pdev ,HCFG_48_MHZ );
        do_reset = 1;
      }
    }
  }

  if (do_reset)
    USB_OTG_ResetPort(pdev);

  /* Clear Port Interrupts */
  USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0_dup.d32);
  
  return retval;
}

//--------------------------------------------------------------
//
uint32_t USB_OTG_USBH_handle_hc_n_Out_ISR (USB_OTG_CORE_HANDLE *pdev , uint32_t num)
{
  
  USB_OTG_HCINTn_TypeDef     hcint;
  USB_OTG_HCGINTMSK_TypeDef  hcintmsk;
  USB_OTG_HC_REGS *hcreg;
  USB_OTG_HCCHAR_TypeDef     hcchar; 
  
  hcreg = pdev->regs.HC_REGS[num];
  hcint.d32 = USB_OTG_READ_REG32(&hcreg->HCINT);
  // hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCGINTMSK);
  hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCINTMSK);
  hcint.d32 = hcint.d32 & hcintmsk.d32;
  
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCCHAR);
  
  if (hcint.b.ahberr)
  {
    CLEAR_HC_INT(hcreg ,ahberr);
    UNMASK_HOST_INT_CHH (num);
  } 
  else if (hcint.b.ack)
  {
    CLEAR_HC_INT(hcreg , ack);
  }
  
  else if (hcint.b.xfercompl)
  {
    pdev->host.ErrCnt[num] = 0;
    UNMASK_HOST_INT_CHH (num);
    USB_OTG_HC_Halt(pdev, num);
    CLEAR_HC_INT(hcreg , xfercompl);
    pdev->host.HC_Status[num] = HC_XFRC;            
  }
  
  else if (hcint.b.stall)
  {
    CLEAR_HC_INT(hcreg , stall);
    UNMASK_HOST_INT_CHH (num);
    USB_OTG_HC_Halt(pdev, num);
    pdev->host.HC_Status[num] = HC_STALL;      
  }
  
  else if (hcint.b.nak)
  {
    pdev->host.ErrCnt[num] = 0;
    UNMASK_HOST_INT_CHH (num);
    USB_OTG_HC_Halt(pdev, num);
    CLEAR_HC_INT(hcreg , nak);
    pdev->host.HC_Status[num] = HC_NAK;      
  }
  
  else if (hcint.b.xacterr)
  {
    UNMASK_HOST_INT_CHH (num);
    USB_OTG_HC_Halt(pdev, num);
    pdev->host.ErrCnt[num] ++;
    pdev->host.HC_Status[num] = HC_XACTERR;
    CLEAR_HC_INT(hcreg , xacterr);
  }
  else if (hcint.b.nyet)
  {
    pdev->host.ErrCnt[num] = 0;
    UNMASK_HOST_INT_CHH (num);
    USB_OTG_HC_Halt(pdev, num);
    CLEAR_HC_INT(hcreg , nyet);
    pdev->host.HC_Status[num] = HC_NYET;    
  }
  else if (hcint.b.datatglerr)
  {
   
    UNMASK_HOST_INT_CHH (num);
    USB_OTG_HC_Halt(pdev, num);
    CLEAR_HC_INT(hcreg , nak);   
    pdev->host.HC_Status[num] = HC_DATATGLERR;
    
    CLEAR_HC_INT(hcreg , datatglerr);
  }  
  else if (hcint.b.chhltd)
  {
    MASK_HOST_INT_CHH (num);
    
    if(pdev->host.HC_Status[num] == HC_XFRC)
    {
      pdev->host.URB_State[num] = URB_DONE;  
      
      if (hcchar.b.eptype == EP_TYPE_BULK)
      {
        pdev->host.hc[num].toggle_out ^= 1; 
      }
    }
    else if(pdev->host.HC_Status[num] == HC_NAK)
    {
      pdev->host.URB_State[num] = URB_NOTREADY;      
    }    
    else if(pdev->host.HC_Status[num] == HC_NYET)
    {
      if(pdev->host.hc[num].do_ping == 1)
      {
        USB_OTG_HC_DoPing(pdev, num);
      }
      pdev->host.URB_State[num] = URB_NOTREADY;      
    }      
    else if(pdev->host.HC_Status[num] == HC_STALL)
    {
      pdev->host.URB_State[num] = URB_STALL;      
    }  
    else if(pdev->host.HC_Status[num] == HC_XACTERR)
    {
      if (pdev->host.ErrCnt[num] == 3)
      {
        pdev->host.URB_State[num] = URB_ERROR;  
        pdev->host.ErrCnt[num] = 0;
      }
    }
    CLEAR_HC_INT(hcreg , chhltd);    
  }
  

  return 1;
}

//--------------------------------------------------------------
//
uint32_t USB_OTG_USBH_handle_hc_n_In_ISR (USB_OTG_CORE_HANDLE *pdev , uint32_t num)
{
  USB_OTG_HCINTn_TypeDef     hcint;
  USB_OTG_HCGINTMSK_TypeDef  hcintmsk;
  USB_OTG_HCCHAR_TypeDef     hcchar; 
  USB_OTG_HCTSIZn_TypeDef  hctsiz;
  USB_OTG_HC_REGS *hcreg;

  
  hcreg = pdev->regs.HC_REGS[num];
  hcint.d32 = USB_OTG_READ_REG32(&hcreg->HCINT);
  // hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCGINTMSK);
  hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCINTMSK);
  hcint.d32 = hcint.d32 & hcintmsk.d32;
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCCHAR);
  hcintmsk.d32 = 0;
  
  
  if (hcint.b.ahberr)
  {
    CLEAR_HC_INT(hcreg ,ahberr);
    UNMASK_HOST_INT_CHH (num);
  }  
  else if (hcint.b.ack)
  {
    CLEAR_HC_INT(hcreg ,ack);
  }
  
  else if (hcint.b.stall)  
  {
    UNMASK_HOST_INT_CHH (num);
    pdev->host.HC_Status[num] = HC_STALL; 
    CLEAR_HC_INT(hcreg , nak);   /* Clear the NAK Condition */
    CLEAR_HC_INT(hcreg , stall); /* Clear the STALL Condition */
    hcint.b.nak = 0;           /* NOTE: When there is a 'stall', reset also nak, 
                                  else, the pdev->host.HC_Status = HC_STALL
                                  will be overwritten by 'nak' in code below */
    USB_OTG_HC_Halt(pdev, num);    
  }
  else if (hcint.b.datatglerr)
  {

      UNMASK_HOST_INT_CHH (num);
      USB_OTG_HC_Halt(pdev, num);
      CLEAR_HC_INT(hcreg , nak);   
      pdev->host.HC_Status[num] = HC_DATATGLERR; 
    CLEAR_HC_INT(hcreg , datatglerr);
  }    
  
  if (hcint.b.frmovrun)
  {
    UNMASK_HOST_INT_CHH (num);
    USB_OTG_HC_Halt(pdev, num);
    CLEAR_HC_INT(hcreg ,frmovrun);
  }
  
  else if (hcint.b.xfercompl)
  {
    
    if (pdev->cfg.dma_enable == 1)
    {
      hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCTSIZ);
      pdev->host.XferCnt[num] =  pdev->host.hc[num].xfer_len - hctsiz.b.xfersize;
    }
 
    pdev->host.HC_Status[num] = HC_XFRC;     
    pdev->host.ErrCnt [num]= 0;
    CLEAR_HC_INT(hcreg , xfercompl);
    
    if ((hcchar.b.eptype == EP_TYPE_CTRL)||
        (hcchar.b.eptype == EP_TYPE_BULK))
    {
      UNMASK_HOST_INT_CHH (num);
      USB_OTG_HC_Halt(pdev, num);
      CLEAR_HC_INT(hcreg , nak); 
      pdev->host.hc[num].toggle_in ^= 1;

    }
    else if(hcchar.b.eptype == EP_TYPE_INTR)
    {
      hcchar.b.oddfrm  = 1;
      USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32); 
      pdev->host.URB_State[num] = URB_DONE;  
    }
    
  }
  else if (hcint.b.chhltd)
  {
    MASK_HOST_INT_CHH (num);
    
    if(pdev->host.HC_Status[num] == HC_XFRC)
    {
      pdev->host.URB_State[num] = URB_DONE;      
    }
    
    else if (pdev->host.HC_Status[num] == HC_STALL) 
    {
       pdev->host.URB_State[num] = URB_STALL;
    }   
    
    else if((pdev->host.HC_Status[num] == HC_XACTERR) ||
            (pdev->host.HC_Status[num] == HC_DATATGLERR))
    {
        pdev->host.ErrCnt[num] = 0;
        pdev->host.URB_State[num] = URB_ERROR;  

    }
    else if(hcchar.b.eptype == EP_TYPE_INTR)
    {
      pdev->host.hc[num].toggle_in ^= 1;
    }
    
    CLEAR_HC_INT(hcreg , chhltd);    
    
  }    
  else if (hcint.b.xacterr)
  {
    UNMASK_HOST_INT_CHH (num);
    pdev->host.ErrCnt[num] ++;
    pdev->host.HC_Status[num] = HC_XACTERR;
    USB_OTG_HC_Halt(pdev, num);
    CLEAR_HC_INT(hcreg , xacterr);    
    
  }
  else if (hcint.b.nak)  
  {  
    if(hcchar.b.eptype == EP_TYPE_INTR)
    {
      UNMASK_HOST_INT_CHH (num);
      USB_OTG_HC_Halt(pdev, num);
      CLEAR_HC_INT(hcreg , nak);   
    }
     else if  ((hcchar.b.eptype == EP_TYPE_CTRL)||
             (hcchar.b.eptype == EP_TYPE_BULK))
    {
      /* re-activate the channel  */
      hcchar.b.chen = 1;
      hcchar.b.chdis = 0;
      USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32); 
    }
    pdev->host.HC_Status[num] = HC_NAK;
  }

  
  return 1;
  
}

//--------------------------------------------------------------
//
static uint32_t USB_OTG_USBH_handle_hc_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_HAINT_TypeDef        haint;
  USB_OTG_HCCHAR_TypeDef       hcchar;
  uint32_t i = 0;
  uint32_t retval = 0;
  
  /* Clear appropriate bits in HCINTn to clear the interrupt bit in
  * GINTSTS */
  
  haint.d32 = USB_OTG_ReadHostAllChannels_intr(pdev);
  
  for (i = 0; i < pdev->cfg.host_channels ; i++)
  {
    if (haint.b.chint & (1 << i))
    {
      hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[i]->HCCHAR);
      
      if (hcchar.b.epdir)
      {
        retval |= USB_OTG_USBH_handle_hc_n_In_ISR (pdev, i);
      }
      else
      {
        retval |=  USB_OTG_USBH_handle_hc_n_Out_ISR (pdev, i);
      }
    }
  }
  
  return retval;
}

//--------------------------------------------------------------
/**
* @brief  STM32_USBF_OTG_ISR_Handler
*         handles all USB Interrupts
* @param  pdev: device instance
* @retval status
*/

uint32_t USBD_OTG_HS_ISR_Handler (USB_OTG_CORE_HANDLE *pdev)
{

    USB_OTG_GINTSTS_TypeDef  gintr_status;
    uint32_t retval = 0;

    gintr_status.d32 = USB_OTG_ReadCoreItr(pdev);
   
    if (gintr_status.b.outepintr)
    {
      usbMSCHostAssert(0); // retval |= DCD_HandleOutEP_ISR(pdev);
    }    
    
    if (gintr_status.b.inepint)
    {
      usbMSCHostAssert(0); // retval |= DCD_HandleInEP_ISR(pdev);
    }
    
    if (gintr_status.b.modemismatch)
    {
      usbMSCHostAssert(0);
      
      USB_OTG_GINTSTS_TypeDef  gintsts;
      
      /* Clear interrupt */
      gintsts.d32 = 0;
      gintsts.b.modemismatch = 1;
      USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, gintsts.d32);
    }
    
    if (gintr_status.b.wkupintr)
    {
      usbMSCHostAssert(0); // retval |= DCD_HandleResume_ISR(pdev);
    }
    
    if (gintr_status.b.usbsuspend)
    {
      usbMSCHostAssert(0); // retval |= DCD_HandleUSBSuspend_ISR(pdev);
    }
    if (gintr_status.b.sofintr)
    {
      usbMSCHostAssert(0); // retval |= DCD_HandleSof_ISR(pdev);
      
    }

    if (gintr_status.b.sofintr)
    {
      usbMSCHostAssert(0); // retval |= USB_OTG_USBH_handle_sof_ISR (pdev);
    }
    
    if (gintr_status.b.rxstsqlvl)
    {
      retval |= USB_OTG_USBH_handle_rx_qlvl_ISR (pdev);
    }
    
    if (gintr_status.b.nptxfempty)
    {
      usbMSCHostAssert(0); // retval |= USB_OTG_USBH_handle_nptxfempty_ISR (pdev);
    }
    
    if (gintr_status.b.ptxfempty)
    {
      usbMSCHostAssert(0); // retval |= USB_OTG_USBH_handle_ptxfempty_ISR (pdev);
    }    
    
    if (gintr_status.b.hcintr)
      retval |= USB_OTG_USBH_handle_hc_ISR (pdev);
    
    if (gintr_status.b.portintr)
      retval |= USB_OTG_USBH_handle_port_ISR (pdev);
    
    if (gintr_status.b.disconnect)
    {
      usbMSCHostAssert(0); // retval |= USB_OTG_USBH_handle_Disconnect_ISR (pdev);  
      
    }
    
     if (gintr_status.b.incomplisoout)
      {
         usbMSCHostAssert(0); // retval |= USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR (pdev);
      }
      
    if (gintr_status.b.usbreset)
    {
      usbMSCHostAssert(0); // retval |= DCD_HandleUsbReset_ISR(pdev);
      
    }
    if (gintr_status.b.enumdone)
    {
      usbMSCHostAssert(0); // retval |= DCD_HandleEnumDone_ISR(pdev);
    }
    
    if (gintr_status.b.incomplisoin)
    {
      usbMSCHostAssert(0); // retval |= DCD_IsoINIncomplete_ISR(pdev);
    }

    if (gintr_status.b.incomplisoout)
    {
      usbMSCHostAssert(0); // retval |= DCD_IsoOUTIncomplete_ISR(pdev);
    }    

  return retval;
}

//--------------------------------------------------------------
// USB HS Host irq handler
extern "C" {
  void __irq_usb_hs(void)
  {
	USBD_OTG_HS_ISR_Handler (&USB_OTG_Core_Host);
  }
}

//--------------------------------------------------------------
//
static USBH_Status USBH_SubmitSetupRequest(USBH_HOST *phost,
                                           uint8_t* buff, 
                                           uint16_t length)
{
  
  /* Prepare the Transactions */
  phost->Control.buff = buff; 
  phost->Control.length = length;
  phost->Control.state = CTRL_SETUP;  

  return USBH_OK;  
}

//--------------------------------------------------------------
USBH_Status dd_USBH_HandleControl (USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  uint8_t direction;  
  static uint16_t timeout = 0;
  USBH_Status status = USBH_BUSY;
  URB_STATE URB_Status = URB_IDLE;
  
  switch (phost->Control.state)
  {
  case CTRL_SETUP:
    /* send a SETUP packet */
    USBH_CtlSendSetup     (pdev, 
	                   phost->Control.setup.d8 , 
	                   phost->Control.hc_num_out);  
    phost->Control.state = CTRL_SETUP_WAIT;  
    break; 
    
  case CTRL_SETUP_WAIT:
    
    URB_Status = dd_HCD_GetURB_State(pdev , phost->Control.hc_num_out); 
    /* case SETUP packet sent successfully */
    if(URB_Status == URB_DONE)
    { 
      direction = (phost->Control.setup.b.bmRequestType & USB_REQ_DIR_MASK);
      
      /* check if there is a data stage */
      if (phost->Control.setup.b.wLength.w != 0 )
      {        
        timeout = DATA_STAGE_TIMEOUT;
        if (direction == USB_D2H)
        {
          /* Data Direction is IN */
          phost->Control.state = CTRL_DATA_IN;
        }
        else
        {
          /* Data Direction is OUT */
          phost->Control.state = CTRL_DATA_OUT;
        } 
      }
      /* No DATA stage */
      else
      {
        timeout = NODATA_STAGE_TIMEOUT;
        
        /* If there is No Data Transfer Stage */
        if (direction == USB_D2H)
        {
          /* Data Direction is IN */
          phost->Control.state = CTRL_STATUS_OUT;
        }
        else
        {
          /* Data Direction is OUT */
          phost->Control.state = CTRL_STATUS_IN;
        } 
      }          
      /* Set the delay timer to enable timeout for data stage completion */
      phost->Control.timer = dd_HCD_GetCurrentFrame(pdev);
    }
    else if(URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;     
    }    
    break;
    
  case CTRL_DATA_IN:  
    /* Issue an IN token */ 
    USBH_CtlReceiveData(pdev,
                        phost->Control.buff, 
                        phost->Control.length,
                        phost->Control.hc_num_in);
 
    phost->Control.state = CTRL_DATA_IN_WAIT;
    break;    
    
  case CTRL_DATA_IN_WAIT:
    
    URB_Status = dd_HCD_GetURB_State(pdev , phost->Control.hc_num_in); 
    
    /* check is DATA packet transfered successfully */
    if  (URB_Status == URB_DONE)
    { 
      phost->Control.state = CTRL_STATUS_OUT;
    }
   
    /* manage error cases*/
    if  (URB_Status == URB_STALL) 
    { 
      massert(0);
    }   
    else if (URB_Status == URB_ERROR)
    {
      /* Device error */
      phost->Control.state = CTRL_ERROR;    
    }
    else if ((dd_HCD_GetCurrentFrame(pdev)- phost->Control.timer) > timeout)
    {
      /* timeout for IN transfer */
      phost->Control.state = CTRL_ERROR; 
    }   
    break;
    
  case CTRL_DATA_OUT:
    /* Start DATA out transfer (only one DATA packet)*/
    
    pdev->host.hc[phost->Control.hc_num_out].toggle_out ^= 1; 
    
    USBH_CtlSendData (pdev,
                      phost->Control.buff, 
                      phost->Control.length , 
                      phost->Control.hc_num_out);
    
    phost->Control.state = CTRL_DATA_OUT_WAIT;
    break;
    
  case CTRL_DATA_OUT_WAIT:
    
    URB_Status = dd_HCD_GetURB_State(pdev , phost->Control.hc_num_out);     
    if  (URB_Status == URB_DONE)
    { /* If the Setup Pkt is sent successful, then change the state */
      phost->Control.state = CTRL_STATUS_IN;
    }
    
    /* handle error cases */
    else if  (URB_Status == URB_STALL) 
    { 
      massert(0);
    } 
    else if  (URB_Status == URB_NOTREADY)
    { 
      /* Nack received from device */
      phost->Control.state = CTRL_DATA_OUT;
    }    
    else if (URB_Status == URB_ERROR)
    {
      /* device error */
      phost->Control.state = CTRL_ERROR;      
    } 
    break;
    
    
  case CTRL_STATUS_IN:
    /* Send 0 bytes out packet */
    USBH_CtlReceiveData (pdev,
                         0,
                         0,
                         phost->Control.hc_num_in);
    
    phost->Control.state = CTRL_STATUS_IN_WAIT;
    
    break;
    
  case CTRL_STATUS_IN_WAIT:
    
    URB_Status = dd_HCD_GetURB_State(pdev , phost->Control.hc_num_in); 
    
    if  ( URB_Status == URB_DONE)
    { /* Control transfers completed, Exit the State Machine */
      status = USBH_OK;
    }
    
    else if (URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;  
    }
#if 0    
    else if((dd_HCD_GetCurrentFrame(pdev) - phost->Control.timer) > timeout)
    {
      phost->Control.state = CTRL_ERROR; 
    }
#endif
     else if(URB_Status == URB_STALL)
    {
      /* Control transfers completed, Exit the State Machine */
      massert(0);
      status = USBH_NOT_SUPPORTED;
    }
    break;
    
  case CTRL_STATUS_OUT:
    pdev->host.hc[phost->Control.hc_num_out].toggle_out ^= 1; 
    USBH_CtlSendData (pdev,
                      0,
                      0,
                      phost->Control.hc_num_out);
    
    phost->Control.state = CTRL_STATUS_OUT_WAIT;
    break;
    
  case CTRL_STATUS_OUT_WAIT: 
    
    URB_Status = dd_HCD_GetURB_State(pdev , phost->Control.hc_num_out);  
    if  (URB_Status == URB_DONE)
    { 
      status = USBH_OK;      
    }
    else if  (URB_Status == URB_NOTREADY)
    { 
      phost->Control.state = CTRL_STATUS_OUT;
    }      
    else if (URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;      
    }
    break;
    
  case CTRL_ERROR:
    /* 
    After a halt condition is encountered or an error is detected by the 
    host, a control endpoint is allowed to recover by accepting the next Setup 
    PID; i.e., recovery actions via some other pipe are not required for control
    endpoints. For the Default Control Pipe, a device reset will ultimately be 
    required to clear the halt or error condition if the next Setup PID is not 
    accepted.
    */
    if (++ phost->Control.errorcount <= USBH_MAX_ERROR_COUNT)
    {
      /* Do the transmission again, starting from SETUP Packet */
      phost->Control.state = CTRL_SETUP; 
    }
    else
    {
      massert(0);
      
      status = USBH_FAIL;
    }
    break;
    
  default:
    break;
  }
  return status;
}


//--------------------------------------------------------------
//
USBH_Status USBH_CtlReq     (USB_OTG_CORE_HANDLE *pdev, 
                             USBH_HOST           *phost, 
                             uint8_t             *buff,
                             uint16_t            length)
{
  USBH_Status status;
  
  switch (phost->RequestState) {

  case CMD_SEND:
    /* Start a SETUP transfer */
    USBH_SubmitSetupRequest(phost, buff, length);
    phost->RequestState = CMD_WAIT;
    return USBH_BUSY;
    
  case CMD_WAIT:

    status = dd_USBH_HandleControl(pdev, phost);

    if (status != USBH_BUSY) {
        phost->RequestState = CMD_SEND;
    }
    return status;
    
  default:
      return USBH_NOT_SUPPORTED;
  }
}

//--------------------------------------------------------------
//
void USBH_MSC_InterfaceInit ( USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost) {	 

    // Connected device must support MSC class/protocol 
    usbMSCHostAssert(phost->device_prop.Itf_Desc[0].bInterfaceClass == MSC_CLASS);
    usbMSCHostAssert(phost->device_prop.Itf_Desc[0].bInterfaceProtocol == MSC_PROTOCOL);

    if(phost->device_prop.Ep_Desc[0][0].bEndpointAddress & 0x80)
    {

      // Endpoint 0 is input
      MSC_Machine.MSBulkInEp = (phost->device_prop.Ep_Desc[0][0].bEndpointAddress);
      MSC_Machine.MSBulkInEpSize  = phost->device_prop.Ep_Desc[0][0].wMaxPacketSize;
      // Endpoint 1 is output
      MSC_Machine.MSBulkOutEp = (phost->device_prop.Ep_Desc[0][1].bEndpointAddress);
      MSC_Machine.MSBulkOutEpSize  = phost->device_prop.Ep_Desc[0][1].wMaxPacketSize;      
    }
    else
    {

      // Endpoint 0 is output
      MSC_Machine.MSBulkOutEp = (phost->device_prop.Ep_Desc[0][0].bEndpointAddress);
      MSC_Machine.MSBulkOutEpSize  = phost->device_prop.Ep_Desc[0] [0].wMaxPacketSize;      
      // Endpoint 1 is input
      MSC_Machine.MSBulkInEp = (phost->device_prop.Ep_Desc[0][1].bEndpointAddress);
      MSC_Machine.MSBulkInEpSize  = phost->device_prop.Ep_Desc[0][1].wMaxPacketSize;      
    }
    
    MSC_Machine.hc_num_out = USBH_Alloc_Channel(pdev, MSC_Machine.MSBulkOutEp);
    MSC_Machine.hc_num_in = USBH_Alloc_Channel(pdev, MSC_Machine.MSBulkInEp);  
    
    /* Open the new bulk mode channels */
    USBH_Open_Channel  (pdev,
                        MSC_Machine.hc_num_out,
                        phost->device_prop.address,
                        phost->device_prop.speed,
                        EP_TYPE_BULK,
                        MSC_Machine.MSBulkOutEpSize);  
    
    USBH_Open_Channel  (pdev,
                        MSC_Machine.hc_num_in,
                        phost->device_prop.address,
                        phost->device_prop.speed,
                        EP_TYPE_BULK,
                        MSC_Machine.MSBulkInEpSize);    
}

//--------------------------------------------------------------
//
void dd_USBH_MSC_Init(USB_OTG_CORE_HANDLE *pdev )
{

  USBH_MSC_CBWData.field.CBWSignature = USBH_MSC_BOT_CBW_SIGNATURE;
  USBH_MSC_CBWData.field.CBWTag = USBH_MSC_BOT_CBW_TAG;
  USBH_MSC_CBWData.field.CBWLUN = 0;  /*Only one LUN is supported*/
  usbh_msc.CmdStateMachine = CMD_SEND_STATE;  
}

//--------------------------------------------------------------
//
USBH_Status USBH_MSC_TestUnitReady(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost) {
 
    USBH_Status bot_status;
  
    usbMSCHostAssert(dd_HCD_IsDeviceConnected(pdev));

    switch(usbh_msc.CmdStateMachine) {

    case CMD_SEND_STATE:  
      /*Prepare the CBW and relevent field*/
      USBH_MSC_CBWData.field.CBWTransferLength = 0;       /* No Data Transfer */
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_OUT;
      USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH_TEST_UNIT_READY; // 6

      memset(USBH_MSC_CBWData.field.CBWCB, 0x00, CBW_LENGTH);
      USBH_MSC_CBWData.field.CBWCB[0]  = OPCODE_TEST_UNIT_READY; 

      // Handle state machines:
      // Leave upper level states unchanged,
      // Switch our level states to 'wait'.
      usbh_msc.CmdStateMachine = CMD_WAIT_STATUS;
      // Initialize lower level transfer state to 'cbw sent'
      usbh_msc.BOTState = USBH_BOTSTATE_SENT_CBW;
      
      dd_USBH_BulkSendData (pdev,
                         USBH_MSC_CBWData.CBWArray,          // 31, entire CBW struct
                         USBH_MSC_BOT_CBW_PACKET_LENGTH_31 , // 31
                         MSC_Machine.hc_num_out);
    
      debugCalls.unitReadyCalls++;

      return USBH_BUSY;
      
    case CMD_WAIT_STATUS: 

      /* Process the BOT state machine */
      bot_status = dd_USBH_MSC_HandleBOTXfer(pdev , phost);

      if (bot_status != USBH_BUSY) {
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }

      return bot_status;
      
    default:
      return USBH_NOT_SUPPORTED;
    }
}

//--------------------------------------------------------------
//
USBH_Status USBH_MSC_ReadCapacity10(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost) {

    USBH_Status bot_status;

    usbMSCHostAssert(dd_HCD_IsDeviceConnected(pdev));

    switch(usbh_msc.CmdStateMachine) {

    case CMD_SEND_STATE:
      /*Prepare the CBW and relevent field*/
      USBH_MSC_CBWData.field.CBWTransferLength = XFER_LEN_READ_CAPACITY10; // 8
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
      USBH_MSC_CBWData.field.CBWLength = CBW_LENGTH; // 10
     
      memset(USBH_MSC_CBWData.field.CBWCB, 0x00, CBW_LENGTH);
      USBH_MSC_CBWData.field.CBWCB[0]  = OPCODE_READ_CAPACITY10; 

      // Handle state machines:
      // Leave upper level states unchanged,
      // Switch our level states to 'wait'.
      usbh_msc.CmdStateMachine = CMD_WAIT_STATUS;
      // Initialize lower level transfer state to 'cbw sent'
      usbh_msc.BOTState = USBH_BOTSTATE_SENT_CBW;

      usbh_msc.pRxTxBuff = usbh_msc.packetBuffer;
      
      dd_USBH_BulkSendData (pdev,
                         USBH_MSC_CBWData.CBWArray,          // 31, entire CBW struct
                         USBH_MSC_BOT_CBW_PACKET_LENGTH_31 , // 31
                         MSC_Machine.hc_num_out);

      return USBH_BUSY;
      
    case CMD_WAIT_STATUS:

      /* Process the BOT state machine */
      bot_status = dd_USBH_MSC_HandleBOTXfer(pdev, phost);

      if (bot_status == USBH_OK) {

          /*assign the capacity*/
          (((uint8_t*)&USBH_MSC_Param.MSCapacity )[3]) = usbh_msc.packetBuffer[0];
          (((uint8_t*)&USBH_MSC_Param.MSCapacity )[2]) = usbh_msc.packetBuffer[1];
          (((uint8_t*)&USBH_MSC_Param.MSCapacity )[1]) = usbh_msc.packetBuffer[2];
          (((uint8_t*)&USBH_MSC_Param.MSCapacity )[0]) = usbh_msc.packetBuffer[3];
        
          /*assign the page length*/
          (((uint8_t*)&USBH_MSC_Param.MSPageLength )[1]) = usbh_msc.packetBuffer[6];
          (((uint8_t*)&USBH_MSC_Param.MSPageLength )[0]) = usbh_msc.packetBuffer[7];
          
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }
      else if (bot_status != USBH_BUSY) {
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }

      return bot_status;
      
    default:
      return USBH_NOT_SUPPORTED;
  }
}

//--------------------------------------------------------------
//
USBH_Status USBH_MSC_RequestSense(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost) {

    USBH_Status bot_status;

    usbMSCHostAssert(dd_HCD_IsDeviceConnected(pdev));

    switch(usbh_msc.CmdStateMachine)
    {
    case CMD_SEND_STATE:
      /*Prepare the CBW and relevent field*/
      USBH_MSC_CBWData.field.CBWTransferLength = ALLOCATION_LENGTH_REQUEST_SENSE;
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
      USBH_MSC_CBWData.field.CBWLength = 6;
     
      memset(USBH_MSC_CBWData.field.CBWCB, 0x00, 6);
      USBH_MSC_CBWData.field.CBWCB[0] = OPCODE_REQUEST_SENSE; 
      USBH_MSC_CBWData.field.CBWCB[4] = ALLOCATION_LENGTH_REQUEST_SENSE;

      // Handle state machines:
      // Leave upper level states unchanged,
      // Switch our level states to 'wait'.
      usbh_msc.CmdStateMachine = CMD_WAIT_STATUS;
      // Initialize lower level transfer state to 'cbw sent'
      usbh_msc.BOTState = USBH_BOTSTATE_SENT_CBW;

      usbh_msc.pRxTxBuff = usbh_msc.packetBuffer;
      
      dd_USBH_BulkSendData (pdev,
                         USBH_MSC_CBWData.CBWArray,          // 31, entire CBW struct
                         USBH_MSC_BOT_CBW_PACKET_LENGTH_31 , // 31
                         MSC_Machine.hc_num_out);

      debugCalls.reqSenseCalls++;

      return USBH_BUSY;
      
    case CMD_WAIT_STATUS:
      
      /* Process the BOT state machine */
      bot_status = dd_USBH_MSC_HandleBOTXfer(pdev, phost);

      if (bot_status == USBH_OK) {

            /* See https://www.t10.org/lists/asc-num.htm */
            sensKey.error  = usbh_msc.packetBuffer[0] & 0x7F;  
            sensKey.key  = usbh_msc.packetBuffer[2] & 0x0F;  
            sensKey.asc  = usbh_msc.packetBuffer[12];
            sensKey.ascq = usbh_msc.packetBuffer[13];

            usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }
      else if (bot_status != USBH_BUSY) {
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }

    return bot_status;
      
    default:
      return USBH_NOT_SUPPORTED;
    }
}

#if 0
#define SCSI_START_STOP_UNIT                        0x1B

//--------------------------------------------------------------
//
USBH_Status USBH_MSC_StartStopUnit(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost, int startStop) {

    USBH_Status bot_status;

    usbMSCHostAssert(dd_HCD_IsDeviceConnected(pdev));

    switch(usbh_msc.CmdStateMachine)
    {
    case CMD_SEND_STATE:
      /*Prepare the CBW and relevent field*/
      USBH_MSC_CBWData.field.CBWTransferLength = 0;
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
      USBH_MSC_CBWData.field.CBWLength = 6;
     
      memset(USBH_MSC_CBWData.field.CBWCB, 0x00, 6);
      USBH_MSC_CBWData.field.CBWCB[0] = SCSI_START_STOP_UNIT;
      USBH_MSC_CBWData.field.CBWCB[1] = 1;                                  // return result immediately ?
      USBH_MSC_CBWData.field.CBWCB[4] = startStop;                                  // start

      // Handle state machines:
      // Leave upper level states unchanged,
      // Switch our level states to 'wait'.
      usbh_msc.CmdStateMachine = CMD_WAIT_STATUS;
      // Initialize lower level transfer state to 'cbw sent'
      usbh_msc.BOTState = USBH_BOTSTATE_SENT_CBW;

      // usbh_msc.pRxTxBuff = usbh_msc.packetBuffer;
      
      dd_USBH_BulkSendData (pdev,
                         USBH_MSC_CBWData.CBWArray,          // 31, entire CBW struct
                         USBH_MSC_BOT_CBW_PACKET_LENGTH_31 , // 31
                         MSC_Machine.hc_num_out);

      debugCalls.reqSenseCalls++;

      return USBH_BUSY;
      
    case CMD_WAIT_STATUS:
      
      /* Process the BOT state machine */
      bot_status = dd_USBH_MSC_HandleBOTXfer(pdev, phost);

      if (bot_status == USBH_OK) {
            // any data?
            usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }
      else if (bot_status != USBH_BUSY) {
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }

    return bot_status;
      
    default:
      return USBH_NOT_SUPPORTED;
    }
}
#endif

//--------------------------------------------------------------
//
USBH_Status USBH_BulkReceiveData( USB_OTG_CORE_HANDLE *pdev, 
                                uint8_t *buff, 
                                uint16_t length,
                                uint8_t hc_num)
{
  pdev->host.hc[hc_num].ep_is_in = 1;   
  pdev->host.hc[hc_num].xfer_buff = buff;
  pdev->host.hc[hc_num].xfer_len = length;

  if( pdev->host.hc[hc_num].toggle_in == 0)
  {
    pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
  }
  else
  {
    pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
  }

  dd_HCD_SubmitRequest (pdev , hc_num);  
  return USBH_OK;
}

//--------------------------------------------------------------
//
USBH_MSC_Status_TypeDef USBH_MSC_DecodeCSW(USB_OTG_CORE_HANDLE *pdev , USBH_HOST *phost)
{
    uint32_t dataXferCount = 0;
    USBH_MSC_Status_TypeDef status = USBH_MSC_FAIL;
  
    usbMSCHostAssert(dd_HCD_IsDeviceConnected(pdev));
  
    /*Checking if the transfer length is diffrent than 13*/
    dataXferCount = dd_HCD_GetXferCnt(pdev, MSC_Machine.hc_num_in); 
    
    if(dataXferCount != USBH_MSC_CSW_LENGTH_13)
    {
      /*(4) Hi > Dn (Host expects to receive data from the device,
      Device intends to transfer no data)
      (5) Hi > Di (Host expects to receive data from the device,
      Device intends to send data to the host)
      (9) Ho > Dn (Host expects to send data to the device,
      Device intends to transfer no data)
      (11) Ho > Do  (Host expects to send data to the device,
      Device intends to receive data from the host)*/
      
      status = USBH_MSC_PHASE_ERROR;
    }
    else
    { /* CSW length is Correct */
      
      /* Check validity of the CSW Signature and CSWStatus */
      if(USBH_MSC_CSWData.field.CSWSignature == USBH_MSC_BOT_CSW_SIGNATURE)
      {/* Check Condition 1. dCSWSignature is equal to 53425355h */
        
        if(USBH_MSC_CSWData.field.CSWTag == USBH_MSC_CBWData.field.CBWTag)
        {
          /* Check Condition 3. dCSWTag matches the dCBWTag from the 
          corresponding CBW */
          
          if(USBH_MSC_CSWData.field.CSWStatus == USBH_MSC_OK) 
          {
            /* Refer to USB Mass-Storage Class : BOT (www.usb.org) 
            
            Hn Host expects no data transfers
            Hi Host expects to receive data from the device
            Ho Host expects to send data to the device
            
            Dn Device intends to transfer no data
            Di Device intends to send data to the host
            Do Device intends to receive data from the host
            
            Section 6.7 
            (1) Hn = Dn (Host expects no data transfers,
            Device intends to transfer no data)
            (6) Hi = Di (Host expects to receive data from the device,
            Device intends to send data to the host)
            (12) Ho = Do (Host expects to send data to the device, 
            Device intends to receive data from the host)
            
            */
            status = USBH_MSC_OK;
          }
          else if(USBH_MSC_CSWData.field.CSWStatus == USBH_MSC_FAIL)
          {
            status = USBH_MSC_FAIL;
          }
          
          else if(USBH_MSC_CSWData.field.CSWStatus == USBH_MSC_PHASE_ERROR)
          { 
            /* Refer to USB Mass-Storage Class : BOT (www.usb.org) 
            Section 6.7 
            (2) Hn < Di ( Host expects no data transfers, 
            Device intends to send data to the host)
            (3) Hn < Do ( Host expects no data transfers, 
            Device intends to receive data from the host)
            (7) Hi < Di ( Host expects to receive data from the device, 
            Device intends to send data to the host)
            (8) Hi <> Do ( Host expects to receive data from the device, 
            Device intends to receive data from the host)
            (10) Ho <> Di (Host expects to send data to the device,
            Di Device intends to send data to the host)
            (13) Ho < Do (Host expects to send data to the device, 
            Device intends to receive data from the host)
            */
            
            status = USBH_MSC_PHASE_ERROR;
          }
        } /* CSW Tag Matching is Checked  */
      } /* CSW Signature Correct Checking */
      else
      {
        /* If the CSW Signature is not valid, We sall return the Phase Error to
        Upper Layers for Reset Recovery */
        
        status = USBH_MSC_PHASE_ERROR;
      }
    } /* CSW Length Check*/
  
  // usbh_msc.isthis_needed_BOTXferStatus  = status;
  return status;
}

//--------------------------------------------------------------
USBH_Status USBH_ClrFeature(USB_OTG_CORE_HANDLE *pdev,
                            USBH_HOST *phost,
                            uint8_t ep_num, 
                            uint8_t hc_num) 
{
  
  phost->Control.setup.b.bmRequestType = USB_H2D | 
                                         USB_REQ_RECIPIENT_ENDPOINT |
                                         USB_REQ_TYPE_STANDARD;
  
  phost->Control.setup.b.bRequest = USB_REQ_CLEAR_FEATURE;
  phost->Control.setup.b.wValue.w = FEATURE_SELECTOR_ENDPOINT;
  phost->Control.setup.b.wIndex.w = ep_num;
  phost->Control.setup.b.wLength.w = 0;           
  
  if ((ep_num & USB_REQ_DIR_MASK ) == USB_D2H)
  { /* EP Type is IN */
    pdev->host.hc[hc_num].toggle_in = 0; 
  }
  else
  {/* EP Type is OUT */
    pdev->host.hc[hc_num].toggle_out = 0; 
  }
  
  return USBH_CtlReq(pdev, phost, 0 , 0 );   
}

//--------------------------------------------------------------
USBH_Status USBH_MSC_BOT_Abort(USB_OTG_CORE_HANDLE *pdev, 
                               USBH_HOST *phost,
                               uint8_t direction)
{
  USBH_Status status;
  
  status = USBH_BUSY;
  
  switch (direction)
  {
  case USBH_MSC_DIR_IN :
    /* send ClrFeture on Bulk IN endpoint */
    status = USBH_ClrFeature(pdev,
                             phost,
                             MSC_Machine.MSBulkInEp,
                             MSC_Machine.hc_num_in);
    
    break;
    
  case USBH_MSC_DIR_OUT :
    /*send ClrFeature on Bulk OUT endpoint */
    status = USBH_ClrFeature(pdev, 
                             phost,
                             MSC_Machine.MSBulkOutEp,
                             MSC_Machine.hc_num_out);
    break;
    
  default:
    break;
  }
  
  return status;
}

//--------------------------------------------------------------
//
USBH_Status USBH_MSC_ReadInquiry(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost) {

  USBH_Status bot_status;
  
  usbMSCHostAssert(dd_HCD_IsDeviceConnected(pdev));

  switch(usbh_msc.CmdStateMachine)
  {
    case CMD_SEND_STATE:
      /*Prepare the CBW and relevent field*/
      USBH_MSC_CBWData.field.CBWTransferLength = DATA_LEN_INQUIRY; // Expected result length
      USBH_MSC_CBWData.field.CBWFlags = USB_EP_DIR_IN;
      USBH_MSC_CBWData.field.CBWLength = 6; // CBW_LENGTH; // 10

      memset(USBH_MSC_CBWData.field.CBWCB, 0x00, CBW_LENGTH);
      USBH_MSC_CBWData.field.CBWCB[0] = OPCODE_INQUIRY;
      USBH_MSC_CBWData.field.CBWCB[4] = 0x24;               // Allocation Length (24h)

      // Handle state machines:
      // Leave upper level states unchanged,
      // Switch our level states to 'wait'.
      usbh_msc.CmdStateMachine = CMD_WAIT_STATUS;
      // Initialize lower level transfer state to 'cbw sent'
      usbh_msc.BOTState = USBH_BOTSTATE_SENT_CBW;
      
      usbh_msc.pRxTxBuff = usbh_msc.packetBuffer;

      dd_USBH_BulkSendData (pdev,
                         USBH_MSC_CBWData.CBWArray,          // 31, entire CBW struct
                         USBH_MSC_BOT_CBW_PACKET_LENGTH_31 , // 31
                         MSC_Machine.hc_num_out);

      return USBH_BUSY;
      
    case CMD_WAIT_STATUS:

      /* Process the BOT state machine */
      bot_status = dd_USBH_MSC_HandleBOTXfer(pdev , phost);

      if (bot_status == USBH_OK) {
          // Inquery ok, assign Inquiry Data
          inquiry.DeviceType = usbh_msc.packetBuffer[0] & 0x1F;
          inquiry.PeripheralQualifier = usbh_msc.packetBuffer[0] >> 5;  
          inquiry.RemovableMedia = (usbh_msc.packetBuffer[1] & 0x80) == 0x80;
          memcpy (inquiry.vendor_id, usbh_msc.packetBuffer+8, 8);
          memcpy (inquiry.product_id, usbh_msc.packetBuffer+16, 16);
          memcpy (inquiry.revision_id, usbh_msc.packetBuffer+32, 4);    

          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
          return USBH_OK;
      }
      else if (bot_status != USBH_BUSY) {
          usbh_msc.CmdStateMachine = CMD_SEND_STATE;
      }

      return bot_status;
      
    default:
      return USBH_NOT_SUPPORTED;
    }
}

//--------------------------------------------------------------
//
static USBH_Status USBH_MSC_Handle(USB_OTG_CORE_HANDLE *pdev , USBH_HOST *phost)
{
    
    static uint32_t requestSenseCalled = 0;

    USBH_Status retStatus = USBH_BUSY;
    USBH_Status subStatus;
  
    switch(usbh_msc.MSCState)
    {

    case USBH_MSC_BOT_INIT_STATE:
      dd_USBH_MSC_Init(pdev);
      usbh_msc.MSCState = USBH_MSC_BOT_READ_INQUIRY;  
      break;

    case USBH_MSC_BOT_READ_INQUIRY:   

      subStatus = USBH_MSC_ReadInquiry(pdev, phost);
      
      if (subStatus == USBH_OK ) {

        usbh_msc.MSCState = USBH_MSC_TEST_UNIT_READY;
      }
      else {
          usbMSCHostAssert(subStatus == USBH_BUSY);
      }
      break;

    case USBH_MSC_TEST_UNIT_READY:

      /* Issue SCSI command TestUnitReady */ 
      subStatus = USBH_MSC_TestUnitReady(pdev, phost);
      
      if(subStatus == USBH_OK ) {

        usbh_msc.MSCState = USBH_MSC_READ_CAPACITY10;
      }
      else if (subStatus == USBH_FAIL) {
            /* Media not ready, so try to check again */
            usbMSCHostAssert(requestSenseCalled++ < 1000);
            usbh_msc.MSCState = USBH_MSC_REQUEST_SENSE;
      }
      else if (subStatus == USBH_UNRECOVERED_ERROR) {
            massert(0);
      }
      else if (subStatus == USBH_ERROR_SPEED_UNKNOWN) {
            massert(0);
            // retstart
      }
      else if (subStatus >=USBH_NOT_SUPPORTED) {
            massert(0);
      }
      else
          usbMSCHostAssert(subStatus == USBH_BUSY);
      break;

    case USBH_MSC_READ_CAPACITY10:

      /* Issue READ_CAPACITY10 SCSI command */
      subStatus = USBH_MSC_ReadCapacity10(pdev, phost);

      if(subStatus == USBH_OK ) {
        usbh_msc.MSCState = USBH_MSC_DEFAULT_APPLI_STATE;
      }
      else {
          usbMSCHostAssert(subStatus == USBH_BUSY);
      }
      break;

    case USBH_MSC_REQUEST_SENSE:

      /* Issue RequestSense SCSI command for retreiving error code */
      subStatus = USBH_MSC_RequestSense(pdev, phost);

      if (subStatus == USBH_OK )
      {
            // 0x6 / 0x28: NOT READY TO READY CHANGE, MEDIUM MAY HAVE CHANGED
            // 0x2 / 0x3A: NOT_READY, MEDIUM NOT PRESENT
            if ((sensKey.key == 0x2) && (sensKey.asc == 0x3a)) {
                // usbh_msc.MSCState = USBH_MSC_START_STOP_UNIT;
                // usbh_msc.MSCState = USBH_MSC_STOP_UNIT;
                usbh_msc.MSCState = USBH_MSC_TEST_UNIT_READY;
            }
            else {
                usbh_msc.MSCState = USBH_MSC_TEST_UNIT_READY;
            }
      }
      else {
          usbMSCHostAssert(subStatus == USBH_BUSY);
      }
      break;

#if 0
    case USBH_MSC_STOP_UNIT:

      subStatus = USBH_MSC_StartStopUnit(pdev, phost, 0);

      if (subStatus == USBH_OK )
      {
            usbh_msc.MSCState = USBH_MSC_START_UNIT;
      }
      else {
            usbMSCHostAssert(subStatus == USBH_BUSY);
      }
      break;

    case USBH_MSC_START_UNIT:

      subStatus = USBH_MSC_StartStopUnit(pdev, phost, 1);

      if (subStatus == USBH_OK )
      {
            usbh_msc.MSCState = USBH_MSC_TEST_UNIT_READY;
      }
      else {
            usbMSCHostAssert(subStatus == USBH_BUSY);
      }
      break;

    case USBH_MSC_START_STOP_UNIT:

      subStatus = USBH_MSC_StartStopUnit(pdev, phost, 1);

      if (subStatus == USBH_OK )
      {
            usbh_msc.MSCState = USBH_MSC_TEST_UNIT_READY;
      }
      else {
            usbMSCHostAssert(subStatus == USBH_BUSY);
      }
      break;
#endif

    case USBH_MSC_DEFAULT_APPLI_STATE:
      /* Process Application callback for MSC */
      retStatus = USBH_OK;
      break;

    default:
      usbMSCHostAssert(0);
      break; 
    }

   return retStatus;
}

static USBH_Status USBH_HandleEnum(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);

//--------------------------------------------------------------
//
void dd_USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{

  gpio_set_mode(DD_BOARD_USB_DM_PIN, (gpio_pin_mode)(GPIO_MODE_AF | GPIO_OTYPE_PP | GPIO_OSPEED_100MHZ));
  gpio_set_mode(DD_BOARD_USB_DP_PIN, (gpio_pin_mode)(GPIO_MODE_AF | GPIO_OTYPE_PP | GPIO_OSPEED_100MHZ));
  gpio_set_af_mode(DD_BOARD_USB_DM_PIN, GPIO_AFMODE_OTG_HS) ;    // OTG_FS_DM
  gpio_set_af_mode(DD_BOARD_USB_DP_PIN, GPIO_AFMODE_OTG_HS) ;    // OTG_FS_DP

  rcc_clk_enable(RCC_USBHS);
}

// xxx
#define USB_OTG_HCCHAR_CHENA_Pos                 (31U)                         
#define USB_OTG_HCCHAR_CHENA_Msk                 (0x1UL << USB_OTG_HCCHAR_CHENA_Pos) /*!< 0x80000000 */
#define USB_OTG_HCCHAR_CHENA                     USB_OTG_HCCHAR_CHENA_Msk      /*!< Channel enable */

/**
  * @brief  Stop Host Core
  * @param  USBx : Selected device
  * @retval HAL state
  */
void USB_StopHost(USB_OTG_CORE_HANDLE *pdev) // , USB_OTG_GlobalTypeDef *USBx)
{
  uint8_t i;
  uint32_t count = 0;
  USB_OTG_HCCHAR_TypeDef  hcchar;
  
  USB_OTG_BSP_DisableInterrupt(pdev);
  // USB_DisableGlobalInt(USBx);
  
    /* Flush FIFO */
  // USB_FlushTxFifo(USBx, 0x10);
  // USB_FlushRxFifo(USBx);
  USB_OTG_FlushTxFifo(pdev ,  0x10 );  
  USB_OTG_FlushRxFifo(pdev);
  
  /* Flush out any leftover queued requests. */
#if 0
  for (i = 0; i <= 15; i++)
  {   
    value = USBx_HC(i)->HCCHAR ;
    value |=  USB_OTG_HCCHAR_CHDIS;
    value &= ~USB_OTG_HCCHAR_CHENA;  
    value &= ~USB_OTG_HCCHAR_EPDIR;
    USBx_HC(i)->HCCHAR = value;
  }
#endif

  for (i = 0; i < USB_OTG_MAX_TX_FIFOS; i++)
  {
    hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[i]->HCCHAR);
    hcchar.b.chdis = 1;
    hcchar.b.chen = 0;
    hcchar.b.epdir = 0;
    USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[i]->HCCHAR, hcchar.d32);
  }
  
  /* Halt all channels to put them into a known state. */  
  for (i = 0; i <= USB_OTG_MAX_TX_FIFOS; i++)
  {   

    // value = USBx_HC(i)->HCCHAR ;
    hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[i]->HCCHAR);
    
    // value |= USB_OTG_HCCHAR_CHDIS;
    hcchar.b.chdis = 1;
    // value |= USB_OTG_HCCHAR_CHENA;  
    hcchar.b.chen = 1;
    // value &= ~USB_OTG_HCCHAR_EPDIR;
    hcchar.b.epdir = 0;
    
    // USBx_HC(i)->HCCHAR = value;
    USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[i]->HCCHAR, hcchar.d32);
    do 
    {
      if (++count > 1000) 
      {
        break;
      }
    } 
    while ((USB_OTG_READ_REG32(&pdev->regs.HC_REGS[i]->HCCHAR) & USB_OTG_HCCHAR_CHENA) == USB_OTG_HCCHAR_CHENA);
  }

  /* Clear any pending Host interrups */  
  // USBx_HOST->HAINT = 0xFFFFFFFF;
  USB_OTG_WRITE_REG32( &pdev->regs.HREGS->HAINT, 0xFFFFFFFF);
  // USBx->GINTSTS = 0xFFFFFFFF;
  USB_OTG_WRITE_REG32( &pdev->regs.GREGS->GINTSTS, 0xFFFFFFFF);

  rcc_clk_disable(RCC_USBHS);

  // USB_EnableGlobalInt(USBx);
  USB_OTG_BSP_mDelay(250); 
}

//--------------------------------------------------------------
void dd_USBH_Init(USB_OTG_CORE_HANDLE *pdev,
               USB_OTG_CORE_ID_TypeDef coreID,
               USBH_HOST *phost) {
    
  USB_StopHost(pdev);

  /* Hardware Init */
  dd_USB_OTG_BSP_Init(pdev);  
  
  /* Host de-initializations */
  USBH_DeInit(pdev, phost);
  
  /* Start the USB OTG core */     
  dd_HCD_Init(pdev , coreID);
   
  /* Enable Interrupts */
  // USB_OTG_BSP_EnableInterrupt(pdev);
  nvic_irq_set_priority((nvic_irq_num)OTG_HS_IRQn, 3);
  nvic_irq_enable((nvic_irq_num)OTG_HS_IRQn);
}

//--------------------------------------------------------------
USBH_Status USBH_DeInit(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  /* Software Init */
  
  phost->gState = HOST_IDLE;
  phost->EnumState = ENUM_IDLE;
  phost->RequestState = CMD_SEND;  
  
  phost->Control.state = CTRL_SETUP;
  phost->Control.ep0size = USB_OTG_MAX_EP0_SIZE;  
  
  phost->device_prop.address = USBH_DEVICE_ADDRESS_DEFAULT;
  phost->device_prop.speed = HPRT0_PRTSPD_FULL_SPEED;
  
  USBH_Free_Channel  (pdev, phost->Control.hc_num_in);
  USBH_Free_Channel  (pdev, phost->Control.hc_num_out);  
  return USBH_OK;
}

//--------------------------------------------------------------
void USBH_Process(USB_OTG_CORE_HANDLE *pdev , USBH_HOST *phost)
{
  switch (phost->gState)
  {

  case HOST_IDLE :
    
    if (dd_HCD_IsDeviceConnected(pdev))  
    {
      /* Wait for USB Connect Interrupt void USBH_ISR_Connected(void) */     

      // Unset connected state to detect reconnect
      pdev->host.ConnSts = 0;
      phost->gState = HOST_DEV_WAIT_FOR_ATTACHMENT;

      USB_OTG_BSP_mDelay(200); 

      dd_HCD_ResetPort(pdev);
    }
    break;
 
  case HOST_DEV_WAIT_FOR_ATTACHMENT:
    
    if (dd_HCD_IsDeviceConnected(pdev))  
    {
      /* Wait for USB Connect Interrupt void USBH_ISR_Connected(void) */     
      USBH_DeAllocate_AllChannel(pdev);
      phost->gState = HOST_DEV_ATTACHED;
    }
    break;
 
  case HOST_DEV_ATTACHED :
    
        /* Wait for 100 ms after Reset */
        USB_OTG_BSP_mDelay(100); 

        phost->Control.hc_num_out = USBH_Alloc_Channel(pdev, 0x00);
        phost->Control.hc_num_in = USBH_Alloc_Channel(pdev, 0x80);  
  
        /*  Wait for USB USBH_ISR_PrtEnDisableChange()  
            Host is Now ready to start the Enumeration 
        */
      
        phost->device_prop.speed = dd_HCD_GetCurrentSpeed(pdev);
      
        phost->gState = HOST_ENUMERATION;

        /* Open Control pipes */
        USBH_Open_Channel (pdev,
                           phost->Control.hc_num_in,
                           phost->device_prop.address,
                           phost->device_prop.speed,
                           EP_TYPE_CTRL,
                           phost->Control.ep0size); 
      
        /* Open Control pipes */
        USBH_Open_Channel (pdev,
                           phost->Control.hc_num_out,
                           phost->device_prop.address,
                           phost->device_prop.speed,
                           EP_TYPE_CTRL,
                           phost->Control.ep0size);          
    break;
    
  case HOST_ENUMERATION:     

    /* Check for enumeration status */  
    if ( USBH_HandleEnum(pdev , phost) == USBH_OK)
    { 
        usbMSCHostAssert(phost->device_prop.Dev_Desc.bNumConfigurations > 0);

        phost->gState  = HOST_SET_CONFIGURATION;    
    }
    break;
    
  case HOST_SET_CONFIGURATION:     

    /* set configuration  (default config) */
    if (USBH_SetCfg(pdev, 
                    phost,
                    phost->device_prop.Cfg_Desc.bConfigurationValue) == USBH_OK)
    {
        USBH_MSC_InterfaceInit(pdev, phost);
        phost->gState  = HOST_CLASS;     
    }
    break;
    
  case HOST_CLASS: {
    /* process class state machine */
    
    USBH_Status status = USBH_MSC_Handle(pdev, phost);
    usbMSCHostAssert((status == USBH_OK) || (status == USBH_BUSY));
    }
    break;       
    
  default :
    usbMSCHostAssert(0);
    break;
  }
}

//--------------------------------------------------------------
//
USBH_Status USBH_GetDescriptor(USB_OTG_CORE_HANDLE *pdev,
                               USBH_HOST           *phost,                                
                               uint8_t  req_type,
                               uint16_t value_idx, 
                               uint8_t* buff, 
                               uint16_t length )
{ 
  phost->Control.setup.b.bmRequestType = USB_D2H | req_type;
  phost->Control.setup.b.bRequest = USB_REQ_GET_DESCRIPTOR;
  phost->Control.setup.b.wValue.w = value_idx;
  
  if ((value_idx & 0xff00) == USB_DESC_STRING)
  {
    phost->Control.setup.b.wIndex.w = 0x0409;
  }
  else
  {
    phost->Control.setup.b.wIndex.w = 0;
  }
  phost->Control.setup.b.wLength.w = length;           
  return USBH_CtlReq(pdev, phost, buff , length );     
}

//--------------------------------------------------------------
//
USBH_Status USBH_SetAddress(USB_OTG_CORE_HANDLE *pdev, 
                            USBH_HOST *phost,
                            uint8_t DeviceAddress)
{
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_DEVICE | \
    USB_REQ_TYPE_STANDARD;
  
  phost->Control.setup.b.bRequest = USB_REQ_SET_ADDRESS;
  
  phost->Control.setup.b.wValue.w = (uint16_t)DeviceAddress;
  phost->Control.setup.b.wIndex.w = 0;
  phost->Control.setup.b.wLength.w = 0;
  
  return USBH_CtlReq(pdev, phost, 0 , 0 );
}

//--------------------------------------------------------------
//
static  USBH_DescHeader_t  *USBH_GetNextDesc (uint8_t   *pbuf, uint16_t  *ptr)
{
  USBH_DescHeader_t  *pnext;
 
  *ptr += ((USBH_DescHeader_t *)pbuf)->bLength;  
  pnext = (USBH_DescHeader_t *)((uint8_t *)pbuf + \
         ((USBH_DescHeader_t *)pbuf)->bLength);
 
  return(pnext);
}

//--------------------------------------------------------------
//
static void  USBH_ParseInterfaceDesc (USBH_InterfaceDesc_TypeDef *if_descriptor, 
                                      uint8_t *buf)
{
  if_descriptor->bLength            = *(uint8_t  *) (buf + 0);
  if_descriptor->bDescriptorType    = *(uint8_t  *) (buf + 1);
  if_descriptor->bInterfaceNumber   = *(uint8_t  *) (buf + 2);
  if_descriptor->bAlternateSetting  = *(uint8_t  *) (buf + 3);
  if_descriptor->bNumEndpoints      = *(uint8_t  *) (buf + 4);
  if_descriptor->bInterfaceClass    = *(uint8_t  *) (buf + 5);
  if_descriptor->bInterfaceSubClass = *(uint8_t  *) (buf + 6);
  if_descriptor->bInterfaceProtocol = *(uint8_t  *) (buf + 7);
  if_descriptor->iInterface         = *(uint8_t  *) (buf + 8);
}

//--------------------------------------------------------------
//
static void  USBH_ParseEPDesc (USBH_EpDesc_TypeDef  *ep_descriptor, 
                               uint8_t *buf)
{
  
  ep_descriptor->bLength          = *(uint8_t  *) (buf + 0);
  ep_descriptor->bDescriptorType  = *(uint8_t  *) (buf + 1);
  ep_descriptor->bEndpointAddress = *(uint8_t  *) (buf + 2);
  ep_descriptor->bmAttributes     = *(uint8_t  *) (buf + 3);
  ep_descriptor->wMaxPacketSize   = LE16 (buf + 4);
  ep_descriptor->bInterval        = *(uint8_t  *) (buf + 6);
}

//--------------------------------------------------------------
//
static void  USBH_ParseCfgDesc (USBH_CfgDesc_TypeDef* cfg_desc,
                                USBH_InterfaceDesc_TypeDef* itf_desc,
                                USBH_EpDesc_TypeDef*  ep_desc, 
                                uint8_t *buf, 
                                uint16_t length)
{  
  USBH_InterfaceDesc_TypeDef    *pif ;
  USBH_EpDesc_TypeDef           *pep;  
  USBH_DescHeader_t             *pdesc = (USBH_DescHeader_t *)buf;
  uint16_t                      ptr;
  int8_t                        if_ix;
  int8_t                        ep_ix;  
  
  pdesc   = (USBH_DescHeader_t *)buf;
  
  /* Parse configuration descriptor */
  cfg_desc->bLength             = *(uint8_t  *) (buf + 0);
  cfg_desc->bDescriptorType     = *(uint8_t  *) (buf + 1);
  cfg_desc->wTotalLength        = LE16 (buf + 2);
  cfg_desc->bNumInterfaces      = *(uint8_t  *) (buf + 4);
  cfg_desc->bConfigurationValue = *(uint8_t  *) (buf + 5);
  cfg_desc->iConfiguration      = *(uint8_t  *) (buf + 6);
  cfg_desc->bmAttributes        = *(uint8_t  *) (buf + 7);
  cfg_desc->bMaxPower           = *(uint8_t  *) (buf + 8);    
  
  
  if (length > USB_CONFIGURATION_DESC_SIZE)
  {
    ptr = USB_LEN_CFG_DESC;
    
    if ( cfg_desc->bNumInterfaces <= USBH_MAX_NUM_INTERFACES) 
    {
      if_ix = 0;
      pif = (USBH_InterfaceDesc_TypeDef *)0;
      
      /* Parse Interface descriptor relative to the current configuration */
      if(cfg_desc->bNumInterfaces <= USBH_MAX_NUM_INTERFACES)
      {
        while (if_ix < cfg_desc->bNumInterfaces) 
        {
          pdesc = USBH_GetNextDesc((uint8_t *)pdesc, &ptr);
          if (pdesc->bDescriptorType   == USB_DESC_TYPE_INTERFACE) 
          {  
            pif               = &itf_desc[if_ix];
            USBH_ParseInterfaceDesc (pif, (uint8_t *)pdesc);
            ep_ix = 0;
            
            /* Parse Ep descriptors relative to the current interface */
            if(pif->bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS)
            {          
              while (ep_ix < pif->bNumEndpoints) 
              {
                pdesc = USBH_GetNextDesc((uint8_t* )pdesc, &ptr);
                if (pdesc->bDescriptorType   == USB_DESC_TYPE_ENDPOINT) 
                {  
                  pep               = &ep_desc[ep_ix];
                  USBH_ParseEPDesc (pep, (uint8_t *)pdesc);
                  ep_ix++;
                }
                else
                {
                  ptr += pdesc->bLength;
                }
              }
            }
            if_ix++;
          }
          else
          {
            ptr += pdesc->bLength;
          }
        }
      }
    }
  }  
}

//--------------------------------------------------------------
//
USBH_Status USBH_Get_CfgDesc(USB_OTG_CORE_HANDLE *pdev, 
                             USBH_HOST           *phost,                      
                             uint16_t length)

{
  USBH_Status status;
  
  if((status = USBH_GetDescriptor(pdev,
                                  phost,
                                  USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,                          
                                  USB_DESC_CONFIGURATION, 
                                  pdev->host.Rx_Buffer,
                                  length)) == USBH_OK)
  {
    /* Commands successfully sent and Response Received  */       
    USBH_ParseCfgDesc (&phost->device_prop.Cfg_Desc,
                       phost->device_prop.Itf_Desc,
                       phost->device_prop.Ep_Desc[0], 
                       pdev->host.Rx_Buffer,
                       length); 
    
  }
  return status;
}

//--------------------------------------------------------------
//
USBH_Status USBH_SetCfg(USB_OTG_CORE_HANDLE *pdev, 
                        USBH_HOST *phost,
                        uint16_t cfg_idx)
{
  
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_DEVICE |\
    USB_REQ_TYPE_STANDARD;
  phost->Control.setup.b.bRequest = USB_REQ_SET_CONFIGURATION;
  phost->Control.setup.b.wValue.w = cfg_idx;
  phost->Control.setup.b.wIndex.w = 0;
  phost->Control.setup.b.wLength.w = 0;           
  
  return USBH_CtlReq(pdev, phost, 0 , 0 );      
}

//--------------------------------------------------------------
//
static USBH_Status USBH_HandleEnum(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  USBH_Status Status = USBH_BUSY;  
  
  switch (phost->EnumState)
  {
  case ENUM_IDLE:  
    /* Get Device Desc for only 1st 8 bytes : To get EP0 MaxPacketSize */
    if ( USBH_Get_DevDesc(pdev , phost, 8) == USBH_OK)
    {
      phost->Control.ep0size = phost->device_prop.Dev_Desc.bMaxPacketSize;
      
      phost->EnumState = ENUM_GET_FULL_DEV_DESC;
      
      /* modify control channels configuration for MaxPacket size */
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_out,
                           0,
                           0,
                           0,
                           phost->Control.ep0size);
      
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_in,
                           0,
                           0,
                           0,
                           phost->Control.ep0size);      
    }
    break;
    
  case ENUM_GET_FULL_DEV_DESC:  
    /* Get FULL Device Desc  */
    if ( USBH_Get_DevDesc(pdev, phost, USB_DEVICE_DESC_SIZE) == USBH_OK)
    {
      phost->EnumState = ENUM_SET_ADDR;
    }
    break;
   
  case ENUM_SET_ADDR: 
    /* set address */
    if ( USBH_SetAddress(pdev, phost, USBH_DEVICE_ADDRESS) == USBH_OK)
    {

        USB_OTG_BSP_mDelay(2); 
      phost->device_prop.address = USBH_DEVICE_ADDRESS;
      
      phost->EnumState = ENUM_GET_CFG_DESC;
      
      /* modify control channels to update device address */
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_in,
                           phost->device_prop.address,
                           0,
                           0,
                           0);
      
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_out,
                           phost->device_prop.address,
                           0,
                           0,
                           0);         
    }
    break;
    
  case ENUM_GET_CFG_DESC:  
    /* get standard configuration descriptor */
    if ( USBH_Get_CfgDesc(pdev, phost,
                          USB_CONFIGURATION_DESC_SIZE) == USBH_OK)
    {
      phost->EnumState = ENUM_GET_FULL_CFG_DESC;
    }
    break;
    
  case ENUM_GET_FULL_CFG_DESC:  
    /* get FULL config descriptor (config, interface, endpoints) */
    if (USBH_Get_CfgDesc(pdev, 
                         phost,
                         phost->device_prop.Cfg_Desc.wTotalLength) == USBH_OK)
    {
        Status = USBH_OK;
    }
    break;
    
  default:
    break;
  }  
  return Status;
}

//--------------------------------------------------------------
//
USBH_Status USBH_CtlSendSetup ( USB_OTG_CORE_HANDLE *pdev, 
                                uint8_t *buff, 
                                uint8_t hc_num){
  pdev->host.hc[hc_num].ep_is_in = 0;
  pdev->host.hc[hc_num].data_pid = HC_PID_SETUP;   
  pdev->host.hc[hc_num].xfer_buff = buff;
  pdev->host.hc[hc_num].xfer_len = USBH_SETUP_PKT_SIZE;   

  return (USBH_Status)dd_HCD_SubmitRequest (pdev , hc_num);   
}

//--------------------------------------------------------------
//
USBH_Status USBH_CtlReceiveData(USB_OTG_CORE_HANDLE *pdev, 
                                uint8_t* buff, 
                                uint8_t length,
                                uint8_t hc_num)
{

  pdev->host.hc[hc_num].ep_is_in = 1;
  pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
  pdev->host.hc[hc_num].xfer_buff = buff;
  pdev->host.hc[hc_num].xfer_len = length;  

  dd_HCD_SubmitRequest (pdev , hc_num);   
  
  return USBH_OK;
}

//--------------------------------------------------------------
//
USBH_Status USBH_CtlSendData ( USB_OTG_CORE_HANDLE *pdev, 
                                uint8_t *buff, 
                                uint8_t length,
                                uint8_t hc_num)
{
  pdev->host.hc[hc_num].ep_is_in = 0;
  pdev->host.hc[hc_num].xfer_buff = buff;
  pdev->host.hc[hc_num].xfer_len = length;
 
  if ( length == 0 )
  { /* For Status OUT stage, Length==0, Status Out PID = 1 */
    pdev->host.hc[hc_num].toggle_out = 1;   
  }
 
 /* Set the Data Toggle bit as per the Flag */
  if ( pdev->host.hc[hc_num].toggle_out == 0)
  { /* Put the PID 0 */
      pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;    
  }
 else
 { /* Put the PID 1 */
      pdev->host.hc[hc_num].data_pid = HC_PID_DATA1 ;
 }

  dd_HCD_SubmitRequest (pdev , hc_num);   
   
  return USBH_OK;
}

//--------------------------------------------------------------
//
static void  USBH_ParseDevDesc (USBH_DevDesc_TypeDef* dev_desc,
                                uint8_t *buf, 
                                uint16_t length)
{
  dev_desc->bLength            = *(uint8_t  *) (buf +  0);
  dev_desc->bDescriptorType    = *(uint8_t  *) (buf +  1);
  dev_desc->bcdUSB             = LE16 (buf +  2);
  dev_desc->bDeviceClass       = *(uint8_t  *) (buf +  4);
  dev_desc->bDeviceSubClass    = *(uint8_t  *) (buf +  5);
  dev_desc->bDeviceProtocol    = *(uint8_t  *) (buf +  6);
  dev_desc->bMaxPacketSize     = *(uint8_t  *) (buf +  7);
  
  if (length > 8)
  { /* For 1st time after device connection, Host may issue only 8 bytes for 
    Device Descriptor Length  */
    dev_desc->idVendor           = LE16 (buf +  8);
    dev_desc->idProduct          = LE16 (buf + 10);
    dev_desc->bcdDevice          = LE16 (buf + 12);
    dev_desc->iManufacturer      = *(uint8_t  *) (buf + 14);
    dev_desc->iProduct           = *(uint8_t  *) (buf + 15);
    dev_desc->iSerialNumber      = *(uint8_t  *) (buf + 16);
    dev_desc->bNumConfigurations = *(uint8_t  *) (buf + 17);
  }
}

//--------------------------------------------------------------
//
USBH_Status USBH_Get_DevDesc(USB_OTG_CORE_HANDLE *pdev,
                             USBH_HOST *phost,
                             uint8_t length)
{
  
  USBH_Status status;
  
  if((status = USBH_GetDescriptor(pdev, 
                                  phost,
                                  USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,                          
                                  USB_DESC_DEVICE, 
                                  pdev->host.Rx_Buffer,
                                  length)) == USBH_OK)
  {
    /* Commands successfully sent and Response Received */       
    USBH_ParseDevDesc(&phost->device_prop.Dev_Desc, pdev->host.Rx_Buffer, length);
  }
  return status;      
}

//--------------------------------------------------------------


