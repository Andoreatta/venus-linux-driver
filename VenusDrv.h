/* NITGEN USB Fingkey Hamster (hfdu04/hfdu06) driver  - 1.03
 * Copyright(C) 2012, NITGEN&COMPANY CO., Ltd.
 * History: 
 *      29/10/2004 : first release  
 *      10/10/2009 : Added FDU06 device
 *      04/08/2011 : Added i/o control macro
 *      22/03/2012 : Modified i/o control macro
 */

#ifndef __LINUX_HFDU04__
#define __LINUX_HFDU04__

#include <asm/ioctl.h>
#include <asm/types.h>

#define _USER_COMMAND_LEN              73

#define _BULK_DATA_LEN                 64
typedef struct
{
        unsigned char data[_BULK_DATA_LEN];
        unsigned int size;
        unsigned int pipe;
}bulk_transfer_t,*pbulk_transfer_t;

/* I/O Control macros */
#define EZUSB_BULK_WRITE               _IOW('E',0x01,bulk_transfer_t )
#define EZUSB_BULK_READ                _IOR('E',0x02,bulk_transfer_t )
#define EZUSB_RESET_PIPE               _IO('E',0x03)

#define EZUSB_GET_DEVICE_DESCRIPTOR    0x04  
// Added by khan, 2011.08.04
#define GET_MODULE_VERSION             0x05
// Modified by khan, 2012.03.22
#define FDU04_USER_COMMAND             0x06

#endif /*  __LINUX_HFDU04__ */ 
