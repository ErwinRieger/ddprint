

#pragma once

#include "usbh_msc_scsi.h"

#define isThisUsedAssert() assert(0)

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

#include <VCP/misc.h>

typedef int IRQn_Type;
#define __NVIC_PRIO_BITS          4
#define __Vendor_SysTickConfig    1
#include <VCP/core_cm4.h>

// Note: redefine them here to be sure the rest of libmaple
// uses the same pins.
#define DD_BOARD_USB_DM_PIN PB14
#define DD_BOARD_USB_DP_PIN PB15

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

#define usbMSCHostAssert(expr) { if (! (expr) ) assert(0); }

// #if ! defined(max)
    // #define max(X,Y) ((X) > (Y) ? (X) : (Y))
// #endif

extern USB_OTG_CORE_HANDLE  USB_OTG_Core_Host;
extern USBH_HOST            USB_Host;

// INQUIRY data.
typedef struct {
  uint8_t PeripheralQualifier;
  uint8_t DeviceType;
  uint8_t RemovableMedia;
  uint8_t vendor_id[9];
  uint8_t product_id[17];
  uint8_t revision_id[5];
} SCSI_StdInquiryDataTypeDef;

typedef union _USB_OTG_HCGINTMSK_TypeDef {
  uint32_t d32;
  struct
  {
uint32_t xfercompl :
    1;
uint32_t chhltd :
    1;
uint32_t ahberr :
    1;
uint32_t stall :
    1;
uint32_t nak :
    1;
uint32_t ack :
    1;
uint32_t nyet :
    1;
uint32_t xacterr :
    1;
uint32_t bblerr :
    1;
uint32_t frmovrun :
    1;
uint32_t datatglerr :
    1;
uint32_t Reserved :
    21;
  }
  b;
} USB_OTG_HCGINTMSK_TypeDef;


bool usbhMscInitialized();

uint32_t usbhMscSizeInBlocks();

void dd_USBH_Init(USB_OTG_CORE_HANDLE *pdev,
               USB_OTG_CORE_ID_TypeDef coreID, 
               USBH_HOST *phost);
               
USBH_Status USB_disk_read(
        USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost,
        uint8_t *buff, uint32_t sector);

USBH_Status USB_disk_write(
        USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost,
        uint8_t *buff, uint32_t sector);

//--------------------------------------------------------------
// Status der USB-Verbindung
//--------------------------------------------------------------
typedef enum {
  USB_MSC_HOST_NO_INIT =0,   // USB-Schnittstelle noch nicht initialisiert
  USB_MSC_DEV_DETACHED,      // kein Device angeschlossen
  USB_MSC_DEV_ATTACHED,      // kein Device angeschlossen
  USB_MSC_SPEED_ERROR,       // USB-Speed wird nicht unterstützt
  USB_MSC_DEV_NOT_SUPPORTED, // Device wird nicht untersützt
  USB_MSC_DEV_WRITE_PROTECT, // Device ist schreibgeschützt
  USB_MSC_OVER_CURRENT,      // Überstrom erkannt
  USB_MSC_DEV_CONNECTED      // Device verbunden und bereit
}USB_MSC_HOST_STATUS_t;
extern USB_MSC_HOST_STATUS_t USB_MSC_HOST_STATUS;



//--------------------------------------------------------------
// Globale Funktionen
//--------------------------------------------------------------
void UB_USB_MSC_HOST_Init(void);
USB_MSC_HOST_STATUS_t UB_USB_MSC_HOST_Do(void);



