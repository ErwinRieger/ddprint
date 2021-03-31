//--------------------------------------------------------------
// File     : usbh_stdreq.h
//--------------------------------------------------------------


#ifndef __USBH_STDREQ_H
#define __USBH_STDREQ_H

//--------------------------------------------------------------
// Includes
//-------------------------------------------------------------- 
extern "C" {
    // Maple include
    #include "usb_hcd.h"
}

#include "usbh_core.h"
#include "usbh_def.h"


/*Standard Feature Selector for clear feature command*/
#define FEATURE_SELECTOR_ENDPOINT         0X00
#define FEATURE_SELECTOR_DEVICE           0X01


#define INTERFACE_DESC_TYPE               0x04
#define ENDPOINT_DESC_TYPE                0x05
#define INTERFACE_DESC_SIZE               0x09


#define USBH_HID_CLASS                    0x03


USBH_Status USBH_GetDescriptor(USB_OTG_CORE_HANDLE *pdev, 
                               USBH_HOST           *phost,                                
                               uint8_t  req_type,
                               uint16_t value_idx, 
                               uint8_t* buff, 
                               uint16_t length );

USBH_Status USBH_Get_DevDesc(USB_OTG_CORE_HANDLE *pdev,
                             USBH_HOST *phost,
                             uint8_t length);

USBH_Status USBH_Get_StringDesc(USB_OTG_CORE_HANDLE *pdev, 
                                USBH_HOST           *phost,                                 
                                uint8_t string_index, 
                                uint8_t *buff, 
                                uint16_t length);

USBH_Status USBH_SetCfg(USB_OTG_CORE_HANDLE *pdev, 
                        USBH_HOST *phost,
                        uint16_t configuration_value);

USBH_Status USBH_Get_CfgDesc(USB_OTG_CORE_HANDLE *pdev,
                             USBH_HOST           *phost,                                 
                             uint16_t length);

USBH_Status USBH_SetAddress(USB_OTG_CORE_HANDLE *pdev, 
                            USBH_HOST           *phost,                             
                            uint8_t DeviceAddress);

USBH_Status USBH_ClrFeature(USB_OTG_CORE_HANDLE *pdev,
                            USBH_HOST           *phost,                             
                            uint8_t ep_num, uint8_t hc_num); 

USBH_Status USBH_Issue_ClrFeature(USB_OTG_CORE_HANDLE *pdev, 
                                  USBH_HOST           *phost, 
                                  uint8_t ep_num);


#endif /* __USBH_STDREQ_H */



