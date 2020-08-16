//--------------------------------------------------------------
// File     : usbh_hcs.c
//--------------------------------------------------------------

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "usbh_hcs.h"


//--------------------------------------------------------------
static uint16_t USBH_GetFreeChannel (USB_OTG_CORE_HANDLE *pdev);

//--------------------------------------------------------------
uint8_t USBH_Open_Channel  (USB_OTG_CORE_HANDLE *pdev,
                            uint8_t hc_num,
                            uint8_t dev_address,
                            uint8_t speed,
                            uint8_t ep_type,
                            uint16_t mps)
{

  pdev->host.hc[hc_num].ep_num = pdev->host.channel[hc_num]& 0x7F;
  pdev->host.hc[hc_num].ep_is_in = (pdev->host.channel[hc_num] & 0x80 ) == 0x80;  
  pdev->host.hc[hc_num].dev_addr = dev_address;  
  pdev->host.hc[hc_num].ep_type = ep_type;  
  pdev->host.hc[hc_num].max_packet = mps; 
  pdev->host.hc[hc_num].speed = speed; 
  pdev->host.hc[hc_num].toggle_in = 0; 
  pdev->host.hc[hc_num].toggle_out = 0;   
  if(speed == HPRT0_PRTSPD_HIGH_SPEED)
  {
    pdev->host.hc[hc_num].do_ping = 1;
  }
  
  USB_OTG_HC_Init(pdev, hc_num) ;
  
  return HC_OK; 

}

//--------------------------------------------------------------
uint8_t USBH_Modify_Channel (USB_OTG_CORE_HANDLE *pdev,
                            uint8_t hc_num,
                            uint8_t dev_address,
                            uint8_t speed,
                            uint8_t ep_type,
                            uint16_t mps)
{
  
  if(dev_address != 0)
  {
    pdev->host.hc[hc_num].dev_addr = dev_address;  
  }
  
  if((pdev->host.hc[hc_num].max_packet != mps) && (mps != 0))
  {
    pdev->host.hc[hc_num].max_packet = mps; 
  }
  
  if((pdev->host.hc[hc_num].speed != speed ) && (speed != 0 )) 
  {
    pdev->host.hc[hc_num].speed = speed; 
  }
  
  USB_OTG_HC_Init(pdev, hc_num);
  return HC_OK; 

}

//--------------------------------------------------------------
uint8_t USBH_Alloc_Channel  (USB_OTG_CORE_HANDLE *pdev, uint8_t ep_addr)
{
  uint16_t hc_num;
  
  hc_num =  USBH_GetFreeChannel(pdev);

  if (hc_num != HC_ERROR)
  {
	pdev->host.channel[hc_num] = HC_USED | ep_addr;
  }
  return hc_num;
}

//--------------------------------------------------------------
uint8_t USBH_Free_Channel  (USB_OTG_CORE_HANDLE *pdev, uint8_t idx)
{
   if(idx < HC_MAX)
   {
	 pdev->host.channel[idx] &= HC_USED_MASK;
   }
   return USBH_OK;
}


//--------------------------------------------------------------
uint8_t USBH_DeAllocate_AllChannel  (USB_OTG_CORE_HANDLE *pdev)
{
   uint8_t idx;
   
   for (idx = 2; idx < HC_MAX ; idx ++)
   {
	 pdev->host.channel[idx] = 0;
   }
   return USBH_OK;
}

//--------------------------------------------------------------
static uint16_t USBH_GetFreeChannel (USB_OTG_CORE_HANDLE *pdev)
{
  uint8_t idx = 0;
  
  for (idx = 0 ; idx < HC_MAX ; idx++)
  {
	if ((pdev->host.channel[idx] & HC_USED) == 0)
	{
	   return idx;
	} 
  }
  return HC_ERROR;
}



