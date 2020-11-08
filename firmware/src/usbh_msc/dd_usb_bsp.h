//--------------------------------------------------------------
// File     : usb_bsp.h
//--------------------------------------------------------------

#ifndef __USB_BSP__H__
#define __USB_BSP__H__

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
extern "C" {
    // Maple include
    // #include <usb_core.h>
}

void USB_OTG_BSP_Init (USB_OTG_CORE_HANDLE *pdev);
void USB_OTG_BSP_EnableInterrupt (USB_OTG_CORE_HANDLE *pdev);
void USB_OTG_BSP_DisableInterrupt (USB_OTG_CORE_HANDLE *pdev);

#endif //__USB_BSP__H__


