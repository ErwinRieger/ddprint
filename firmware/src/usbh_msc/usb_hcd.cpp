//--------------------------------------------------------------
// File     : usb_hcd.c
//-------------------------------------------------------------- 

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------

extern "C" {
    #include <usb_core.h>
    #include <usb_bsp.h>
}

#include "usb_hcd.h"

//--------------------------------------------------------------
uint32_t dd_HCD_Init(USB_OTG_CORE_HANDLE *pdev , USB_OTG_CORE_ID_TypeDef coreID)
{
  uint8_t i = 0;
  pdev->host.ConnSts = 0;
  
  for (i= 0; i< USB_OTG_MAX_TX_FIFOS; i++)
  {
  pdev->host.ErrCnt[i]  = 0;
  pdev->host.XferCnt[i]   = 0;
  pdev->host.HC_Status[i]   = HC_IDLE;
  }
  pdev->host.hc[0].max_packet  = 8; 

  USB_OTG_SelectCore(pdev, coreID);

  USB_OTG_DisableGlobalInt(pdev);
  USB_OTG_CoreInit(pdev);

  /* Force Host Mode*/
  USB_OTG_SetCurrentMode(pdev , HOST_MODE);
  USB_OTG_CoreInitHost(pdev);
  USB_OTG_EnableGlobalInt(pdev);

  return 0;
}


//--------------------------------------------------------------
uint32_t dd_HCD_GetCurrentSpeed (USB_OTG_CORE_HANDLE *pdev)
{    
    USB_OTG_HPRT0_TypeDef  HPRT0;
    HPRT0.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);
    
    return HPRT0.b.prtspd;
}

//--------------------------------------------------------------
uint32_t dd_HCD_ResetPort(USB_OTG_CORE_HANDLE *pdev)
{
  /*
  Before starting to drive a USB reset, the application waits for the OTG 
  interrupt triggered by the debounce done bit (DBCDNE bit in OTG_FS_GOTGINT), 
  which indicates that the bus is stable again after the electrical debounce 
  caused by the attachment of a pull-up resistor on DP (FS) or DM (LS).
  */
  
  USB_OTG_ResetPort(pdev); 
  return 0;
}

//--------------------------------------------------------------
uint32_t dd_HCD_IsDeviceConnected(USB_OTG_CORE_HANDLE *pdev)
{
  return (pdev->host.ConnSts);
}

//--------------------------------------------------------------
URB_STATE dd_HCD_GetURB_State (USB_OTG_CORE_HANDLE *pdev , uint8_t ch_num) 
{
  return pdev->host.URB_State[ch_num] ;
}

//--------------------------------------------------------------
uint32_t dd_HCD_GetXferCnt (USB_OTG_CORE_HANDLE *pdev, uint8_t ch_num) 
{
  return pdev->host.XferCnt[ch_num] ;
}



//--------------------------------------------------------------
HC_STATUS HCD_GetHCState (USB_OTG_CORE_HANDLE *pdev ,  uint8_t ch_num) 
{
  return pdev->host.HC_Status[ch_num] ;
}

//--------------------------------------------------------------
uint32_t HCD_HC_Init (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num) 
{
  return USB_OTG_HC_Init(pdev, hc_num);  
}

//--------------------------------------------------------------
USB_OTG_STS dd_HCD_SubmitRequest (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num) 
{
  
  pdev->host.URB_State[hc_num] =   URB_IDLE;  
  pdev->host.hc[hc_num].xfer_count = 0 ;
  return USB_OTG_HC_StartXfer(pdev, hc_num);
}



