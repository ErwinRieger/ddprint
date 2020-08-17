//--------------------------------------------------------------
// File     : usbh_core.h
//--------------------------------------------------------------

#ifndef __USBH_CORE_H
#define __USBH_CORE_H

//--------------------------------------------------------------
// Includes
//-------------------------------------------------------------- 
#include "usb_hcd.h"
#include "usbh_def.h"

#define MSC_CLASS                         0x08
#define HID_CLASS                         0x03
#define MSC_PROTOCOL                      0x50
#define CBI_PROTOCOL                      0x01

#define USBH_MAX_ERROR_COUNT                            2
#define USBH_DEVICE_ADDRESS_DEFAULT                     0
#define USBH_DEVICE_ADDRESS                             1

#define USBH_MAX_NUM_ENDPOINTS                2
#define USBH_MAX_NUM_INTERFACES               2

typedef enum {
  USBH_OK   = 0,
  USBH_BUSY,
  USBH_FAIL,
  USBH_NOT_SUPPORTED,
  USBH_UNRECOVERED_ERROR,
  USBH_ERROR_SPEED_UNKNOWN,
  USBH_APPLY_DEINIT
}USBH_Status;

/* Following states are used for gState */
typedef enum {
  HOST_IDLE =0,
  // HOST_ISSUE_CORE_RESET,
  HOST_DEV_WAIT_FOR_ATTACHMENT,  
  HOST_DEV_ATTACHED,
  HOST_DEV_DISCONNECTED,  
  // HOST_ISSUE_RESET,
  HOST_DETECT_DEVICE_SPEED,
  HOST_ENUMERATION,
  HOST_SET_CONFIGURATION,
  HOST_CLASS_REQUEST,  
  HOST_CLASS,
  // HOST_CTRL_XFER,
  HOST_USR_INPUT,
  HOST_SUSPENDED,
  HOST_ERROR_STATE  
}HOST_State;  

/* Following states are used for EnumerationState */
typedef enum {
  ENUM_IDLE = 0,
  ENUM_GET_FULL_DEV_DESC,
  ENUM_SET_ADDR,
  ENUM_GET_CFG_DESC,
  ENUM_GET_FULL_CFG_DESC,
} ENUM_State;  



/* Following states are used for CtrlXferStateMachine */
typedef enum {
  CTRL_IDLE =0,
  CTRL_SETUP,
  CTRL_SETUP_WAIT,
  CTRL_DATA_IN,
  CTRL_DATA_IN_WAIT,
  CTRL_DATA_OUT,
  CTRL_DATA_OUT_WAIT,
  CTRL_STATUS_IN,
  CTRL_STATUS_IN_WAIT,
  CTRL_STATUS_OUT,
  CTRL_STATUS_OUT_WAIT,
  CTRL_ERROR
}
CTRL_State;  

typedef enum {
  USBH_USR_NO_RESP   = 0,
  USBH_USR_RESP_OK = 1,
}
USBH_USR_Status;

/* Following states are used for RequestState */
typedef enum {
  CMD_IDLE =0,
  CMD_SEND,
  CMD_WAIT
} CMD_State;  



typedef struct _Ctrl
{
  uint8_t               hc_num_in; 
  uint8_t               hc_num_out; 
  uint8_t               ep0size;  
  uint8_t               *buff;
  uint16_t              length;
  uint8_t               errorcount;
  uint16_t              timer;  
  CTRL_STATUS           status;
  USB_Setup_TypeDef     setup;
  CTRL_State            state;  

} USBH_Ctrl_TypeDef;

typedef struct _DeviceProp
{
  
  uint8_t                           address;
  uint8_t                           speed;
  USBH_DevDesc_TypeDef              Dev_Desc;
  USBH_CfgDesc_TypeDef              Cfg_Desc;  
  USBH_InterfaceDesc_TypeDef        Itf_Desc[USBH_MAX_NUM_INTERFACES];
  USBH_EpDesc_TypeDef               Ep_Desc[USBH_MAX_NUM_INTERFACES][USBH_MAX_NUM_ENDPOINTS];
  USBH_HIDDesc_TypeDef              HID_Desc;
  
}USBH_Device_TypeDef;

typedef struct _Host_TypeDef
{
  HOST_State            gState;       /*  Host State Machine Value */
  ENUM_State            EnumState;    /* Enumeration state Machine */
  CMD_State             RequestState;       
  USBH_Ctrl_TypeDef     Control;
  
  USBH_Device_TypeDef   device_prop; 
  
  void                  *reserve1; // USBH_Class_cb_TypeDef               *class_cb;  
  void                  *reserve2; // USBH_Usr_cb_TypeDef  	              *usr_cb;
} USBH_HOST, *pUSBH_HOST;

USBH_Status USBH_DeInit(USB_OTG_CORE_HANDLE *pdev, 
                        USBH_HOST *phost);
void USBH_Process(USB_OTG_CORE_HANDLE *pdev , 
                  USBH_HOST *phost);
void USBH_ErrorHandle(USBH_HOST *phost, 
                      USBH_Status errType);


#endif /* __USBH_CORE_H */


