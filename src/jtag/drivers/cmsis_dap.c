//===========================================================================
//
//   Copyright (C) 2012 by mike brown
//   mike@mikebrown.org.uk
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program; if not, write to the
//   Free Software Foundation, Inc.,
//   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//===========================================================================

#ifdef HAVE_CONFIG_H
 #include "config.h"
#endif

#include <transport/transport.h>
#include <jtag/swd.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include "libusb_common.h"

//======================================
//  See CMSIS-DAP documentation:
//    Version 0.01 - Beta.
//======================================


//===========================================================================
// USB Config

// Known vid/pid pairs:
//  VID 0xC251: Keil Software
//  PID 0xF001: LPC-Link-II CMSIS_DAP
//  PID 0xF001: OPEN-SDA CMSIS_DAP
#define VID               0xC251, 0xC251
#define PID               0xF001, 0xF002

#define WRITE_EP          0x01
#define READ_EP           0x81
#define BUFFER_SIZE       64
#define USB_TIMEOUT       500


//===========================================================================
// CMSIS-DAP General Commands 
#define CMD_DAP_INFO              0x00
#define CMD_DAP_LED               0x01
#define CMD_DAP_CONNECT           0x02
#define CMD_DAP_DISCONNECT        0x03
#define CMD_DAP_WRITE_ABORT       0x08
#define CMD_DAP_DELAY             0x09
#define CMD_DAP_RESET_TARGET      0x0A

// CMD_INFO
#define INFO_ID_VID               0x00      // string
#define INFO_ID_PID               0x02      // string
#define INFO_ID_SERNUM            0x03      // string
#define INFO_ID_FW_VER            0x04      // string
#define INFO_ID_TD_VEND           0x05      // string
#define INFO_ID_TD_NAME           0x06      // string
#define INFO_ID_CAPS              0xf0      // byte
#define INFO_ID_PKT_CNT           0xfe      // byte
#define INFO_ID_PKT_SZ            0xff      // short

#define INFO_CAPS_SWD             0x01
#define INFO_CAPS_JTAG            0x02

// CMD_LED
#define LED_ID_CONNECT            0x00
#define LED_ID_RUN                0x01

#define LED_OFF                   0x00
#define LED_ON                    0x01

// CMD_CONNECT
#define CONNECT_SWD               0x01
#define CONNECT_JTAG              0x02

//======================================
// CMSIS-DAP Common SWD/JTAG Commands 
#define CMD_DAP_SWJ_PINS          0x10
#define CMD_DAP_SWJ_CLOCK         0x11
#define CMD_DAP_SWJ_SEQ           0x12

// PINS
//Bit 0: SWCLK/TCK
//Bit 1: SWDIO/TMS
//Bit 2: TDI
//Bit 3: TDO
//Bit 5: nTRST
//Bit 7: nRESET

//======================================
// CMSIS-DAP SWD Commands 
#define CMD_DAP_SWD_CONFIGURE     0x13

//======================================
// CMSIS-DAP JTAG Commands 
#define CMD_DAP_JTAG_SEQ          0x14
#define CMD_DAP_JTAG_CONFIGURE    0x15
#define CMD_DAP_JTAG_IDCODE       0x16

//======================================
// CMSIS-DAP Transfer Commands 
#define CMD_DAP_TFER_CONFIGURE    0x04
#define CMD_DAP_TFER              0x05
#define CMD_DAP_TFER_BLOCK        0x06
#define CMD_DAP_TFER_ABORT        0x07

//======================================
// CMSIS-DAP Vendor Commands 
// None as yet...


//======================================
static char *info_caps_str[] = {
  "SWD  Supported",
  "JTAG Supported"
};


//===========================================================================
// max clock speed (kHz)
#define DAP_MAX_CLOCK             5000


//===========================================================================
struct cmsis_dap
{
  struct jtag_libusb_device_handle *dev_handle;
  uint8_t data[BUFFER_SIZE];
  uint8_t caps;
  uint8_t mode;
};

static struct cmsis_dap *cmsis_dap_handle = NULL;
static uint32_t write_ep = WRITE_EP;
static uint32_t read_ep = READ_EP;

//================================
// vid/pid can be specified at runtime
static uint16_t vids[] = { VID, 0 };
static uint16_t pids[] = { PID, 0 };


//===========================================================================
static int cmsis_dap_usb_open(void)
{
  struct jtag_libusb_device_handle *dev;
  int result;
  
  LOG_INFO("CMSIS-DAP: cmsis_dap_usb_open");
  result = jtag_libusb_open(vids, pids, &dev);
  if( result != ERROR_OK )
  {
    LOG_ERROR("Can't find CMSIS-DAP Interface! Please check "
              "connection and permissions.");
    return result;
  }

  struct jtag_libusb_device *udev = jtag_libusb_get_device(dev);
  jtag_libusb_set_configuration(dev, 0);
  jtag_libusb_claim_interface(dev, 0);

  jtag_libusb_get_endpoints(udev, &read_ep, &write_ep);

  struct cmsis_dap *dap = malloc(sizeof(struct cmsis_dap));
  if( dap )
  {
    dap->dev_handle = dev;
    dap->caps = 0;
    dap->mode = 0;
  }
  cmsis_dap_handle = dap;
  
  return ERROR_OK;
}

//================================
static void cmsis_dap_usb_close( struct cmsis_dap *dap )
{
  int r;
  
  LOG_INFO("CMSIS-DAP: cmsis_dap_usb_close");
  r = libusb_release_interface( dap->dev_handle, 0 );
  if (r != 0)
  {
    printf("release interface (%d)\n", r);
    return;
  }

  libusb_close( dap->dev_handle );
  libusb_exit( NULL );
  cmsis_dap_handle = NULL;
  
  return;
}

//================================
// Send a message and receive the reply
static int cmsis_dap_usb_xfer( struct cmsis_dap *dap, int len )
{
  int result;
  int xfrd;

  LOG_INFO("CMSIS-DAP: cmsis_dap_usb_xfer");
  result = libusb_interrupt_transfer( dap->dev_handle, write_ep,
                                      dap->data, len,
                                      &xfrd, USB_TIMEOUT );
  if( result )
    return result;

  result = libusb_interrupt_transfer( dap->dev_handle, read_ep,
                                      dap->data, BUFFER_SIZE,
                                      &xfrd, USB_TIMEOUT );


  for( int i=0; i<8; i++ )
  {
    uint8_t *buffer = dap->data;
  
    printf( "%02x (%c) ", buffer[i], buffer[i]>32?buffer[i]:'-' );
  }
  printf( "\n" );

  return result;
}


//===========================================================================

//======================================
static int cmsis_dap_cmd_DAP_SWJ_Pins( uint8_t pins, uint8_t mask,
                                   uint16_t wait, uint8_t *input )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_cmd_DAP_SWJ_Pins");
  buffer[0] = CMD_DAP_SWJ_PINS;
  buffer[1] = pins;
  buffer[2] = mask;
  buffer[3] = wait&0xff;
  buffer[4] = (wait>>8)&0xff;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 5 );

  if( result )
  {
    LOG_ERROR("CMSIS-DAP command CMD_DAP_SWJ_PINS failed (%d)", result);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  *input = buffer[1];

  return ERROR_OK;
}

//======================================
static int cmsis_dap_cmd_DAP_SWJ_Clock( uint16_t clock )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_cmd_DAP_SWJ_Clock");
  // set clock in Hz
  clock *= 1000;
  buffer[0] = CMD_DAP_SWJ_CLOCK;
  buffer[1] = clock&0xff;
  buffer[2] = (clock>>8)&0xff;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 3 );

  if( result )
  {
    LOG_ERROR("CMSIS-DAP command CMD_DAP_SWJ_CLOCK failed (%d)", result);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  return ERROR_OK;
}


//======================================
static int cmsis_dap_cmd_DAP_Info( uint8_t info, uint8_t **data )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_cmd_DAP_Info");
  buffer[0] = CMD_DAP_INFO;
  buffer[1] = info;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 2 );

  if( result )
  {
    LOG_ERROR("CMSIS-DAP command CMD_INFO failed (%d)", result);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  *data = &(buffer[1]);

  return ERROR_OK;
}

//======================================
static int cmsis_dap_cmd_DAP_LED( uint8_t leds )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_cmd_DAP_LED");
  buffer[0] = CMD_DAP_LED;
  buffer[1] = 0x00;
  buffer[2] = leds;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 3 );

  if( result )
  {
    LOG_ERROR("CMSIS-DAP command CMD_LED failed (%d)", result);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  return ERROR_OK;
}

//======================================
static int cmsis_dap_cmd_DAP_Connect( uint8_t mode )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_cmd_DAP_Connect");
  buffer[0] = CMD_DAP_CONNECT;
  buffer[1] = mode;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 2 );

  if( result )
  {
    LOG_ERROR("CMSIS-DAP command CMD_CONNECT failed (%d)", result);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  if( buffer[1] != mode )
  {
    LOG_ERROR("CMSIS-DAP failed to connect in mode (%d)", mode);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  return ERROR_OK;
}

//======================================
static int cmsis_dap_cmd_DAP_Disconnect( void )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_cmd_DAP_Disconnect");
  buffer[0] = CMD_DAP_DISCONNECT;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 1 );

  if( result || buffer[1] )
  {
    LOG_ERROR("CMSIS-DAP command CMD_DISCONNECT failed (%d)", result);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  return ERROR_OK;
}

//======================================
static int cmsis_dap_cmd_DAP_TFER_Configure( uint8_t idle,
                                             uint16_t wait,
                                             uint16_t retry )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_cmd_TFER_Configure");
  buffer[0] = CMD_DAP_TFER_CONFIGURE;
  buffer[1] = idle;
  buffer[2] = wait & 0xff;
  buffer[3] = (wait>>8)&0xff;
  buffer[4] = retry & 0xff;
  buffer[5] = (retry>>8)&0xff;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 6 );

  if( result || buffer[1] )
  {
    LOG_ERROR("CMSIS-DAP command CMD_TFER_Configure failed (%d)", result);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  return ERROR_OK;
}




//======================================
static int cmsis_dap_cmd_DAP_SWD_Configure( uint8_t cfg )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_cmd_SWD_Configure");
  buffer[0] = CMD_DAP_SWD_CONFIGURE;
  buffer[1] = cfg;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 2 );

  if( result || buffer[1] )
  {
    LOG_ERROR("CMSIS-DAP command CMD_SWD_Configure failed (%d)", result);
    return ERROR_JTAG_DEVICE_ERROR;
  }

  return ERROR_OK;
}


//======================================
static int cmsis_dap_swd_read_reg(uint8_t cmd, uint32_t *value)
{
  uint8_t *buffer = cmsis_dap_handle->data;
  int result;
  
  LOG_INFO("CMSIS-DAP: SWD Read  Reg %02x",cmd);

  buffer[0] = CMD_DAP_TFER;
  buffer[1] = 0x00;
  buffer[2] = 0x01;
  buffer[3] = cmd;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 4 );

  if( buffer[1] != 0x01 )
  {
    LOG_ERROR("CMSIS-DAP: SWD Read Error %02x",buffer[1]);
    result = buffer[2];
  }
  
  if( value )
  {
    *value = (buffer[3]     | buffer[4]<<8 |
              buffer[5]<<16 | buffer[6]<<24 );
    LOG_INFO("          Read: %08x",*value);
  }

  return result;
}

//======================================
static int cmsis_dap_swd_write_reg(uint8_t cmd, uint32_t value)
{
  uint8_t *buffer = cmsis_dap_handle->data;
  int result;

  LOG_INFO("CMSIS-DAP: SWD Write Reg %02x %08x",cmd,value);

  buffer[0] = CMD_DAP_TFER;
  buffer[1] = 0x00;
  buffer[2] = 0x01;
  buffer[3] = cmd;
  buffer[4] = (value    )&0xff;
  buffer[5] = (value>> 8)&0xff;
  buffer[6] = (value>>16)&0xff;
  buffer[7] = (value>>24)&0xff;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 8 );

  return result;
}


//===========================================================================

//======================================
static int cmsis_dap_get_version_info( void )
{
  int result;
  uint8_t *data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_get_version_info");
  // INFO_ID_FW_VER - string
  result = cmsis_dap_cmd_DAP_Info( INFO_ID_FW_VER, &data );
  if( result )
    return result;
  if( data[0] ) // strlen
    LOG_INFO( "CMSIS-DAP: FW Version = %s", &data[1] );

  // INFO_ID_CAPS - byte
  result = cmsis_dap_cmd_DAP_Info( INFO_ID_CAPS, &data );
  if( result )
    return result;
  if( data[0] == 1 )
  {
    uint8_t caps = data[1];
    cmsis_dap_handle->caps = caps;
    if( caps & INFO_CAPS_SWD )
      LOG_INFO( "CMSIS-DAP: %s", info_caps_str[0] );
    if( caps & INFO_CAPS_JTAG )
      LOG_INFO( "CMSIS-DAP: %s", info_caps_str[1] );
  }
  
  // INFO_ID_PKT_SZ - short
  result = cmsis_dap_cmd_DAP_Info( INFO_ID_PKT_SZ, &data );
  if( result )
    return result;
  if( data[0] == 2 ) // short
    LOG_INFO( "CMSIS-DAP: Packet Size = %d", data[1] + (data[2]<<8) );

  return ERROR_OK;
}

//======================================
static int cmsis_dap_get_status( void )
{
  int result;
  uint8_t d;

  LOG_INFO("CMSIS-DAP: cmsis_dap_get_status");
  result = cmsis_dap_cmd_DAP_SWJ_Pins( 0, 0, 0, &d );

  if( !result )
  {
    LOG_INFO("SWCLK/TCK = %d SWDIO/TMS = %d TDI = %d TDO = %d nTRST = %d nRESET = %d ",
              (d&(0x01<<0))?1:0,   //Bit 0: SWCLK/TCK
              (d&(0x01<<1))?1:0,   //Bit 1: SWDIO/TMS
              (d&(0x01<<2))?1:0,   //Bit 2: TDI
              (d&(0x01<<3))?1:0,   //Bit 3: TDO
              (d&(0x01<<5))?1:0,   //Bit 5: nTRST
              (d&(0x01<<7))?1:0);  //Bit 7: nRESET
  }

  return result;
}

//======================================
static int cmsis_dap_reset_link( void )
{
  int result;
  uint8_t *buffer = cmsis_dap_handle->data;

  LOG_INFO("CMSIS-DAP: cmsis_dap_reset_link");
  
  LOG_INFO("DAP_SWJ Sequence (reset: 50+ '1' followed by 0)" );
  buffer[0] = CMD_DAP_SWJ_SEQ;
  buffer[1] = 0x38;
  buffer[2] = 0xFF;
  buffer[3] = 0xFF;
  buffer[4] = 0xFF;
  buffer[5] = 0xFF;
  buffer[6] = 0xFF;
  buffer[7] = 0xFF;
  buffer[9] = 0x3F;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 10 );

  if( result )
    return result;
    
  LOG_INFO("DAP Read IDCODE" );
  buffer[0] = 0x05;
  buffer[1] = 0x00;
  buffer[2] = 0x01;
  buffer[3] = 0x02;
  result = cmsis_dap_usb_xfer( cmsis_dap_handle, 4 );

  if( result )
    return result;

  if( buffer[1] == 0 )
    return 0x80 + buffer[2];

  return result;
}

//======================================
static int cmsis_dap_init( void )
{
  int result;
  
  LOG_INFO("CMSIS-DAP: cmsis_dap_init");
  if( cmsis_dap_handle == NULL )
  {
    // JTAG init
    result = cmsis_dap_usb_open();
    if( result != ERROR_OK )
      return result;

    result = cmsis_dap_get_version_info();
    if( result != ERROR_OK )
      return result;
      
    LOG_INFO("CMSIS-DAP: Interface Connected");
  }

  //cmsis_dap_get_status();
  
  for( int i=3; i; i-- )
  {
    cmsis_dap_cmd_DAP_SWJ_Clock( 100 );             // 100kHz
    cmsis_dap_cmd_DAP_TFER_Configure( 0, 64, 0 );   // 
//    cmsis_dap_cmd_DAP_SWD_Configure( 0x04 );        // 
    cmsis_dap_cmd_DAP_SWD_Configure( 0x00 );        // 
    cmsis_dap_cmd_DAP_LED( 0x03 );                  // Both LEDs on

    if (cmsis_dap_reset_link() == ERROR_OK)
      break;
    
    // failed to connect... have another go
    cmsis_dap_cmd_DAP_LED( 0x00 );                  // Both LEDs off
    cmsis_dap_cmd_DAP_Disconnect();
    cmsis_dap_cmd_DAP_Connect( CONNECT_SWD );
  }

  // Init APSEL
  cmsis_dap_swd_write_reg( 0x08, 0);

  LOG_INFO("CMSIS-DAP: Interface ready!!");
  return ERROR_OK;
}

//======================================
static int cmsis_dap_swd_init( uint8_t trn )
{
  int result;
  
  LOG_INFO("CMSIS-DAP: cmsis_dap_swd_init");
  if( cmsis_dap_handle == NULL )
  {
    // SWD init
    result = cmsis_dap_usb_open();
    if( result != ERROR_OK )
      return result;

    result = cmsis_dap_get_version_info();
    if( result != ERROR_OK )
      return result;
  }

  if( !(cmsis_dap_handle->caps & INFO_CAPS_SWD) )
  {
    LOG_ERROR("CMSIS-DAP: SWD not supported");
    return ERROR_JTAG_DEVICE_ERROR;
  }
  
  result = cmsis_dap_cmd_DAP_Connect( CONNECT_SWD );
  if( result != ERROR_OK )
    return result;
  
  // Add more setup here....
  
  LOG_INFO("CMSIS-DAP: SWD Interface Connected");
  return ERROR_OK;
}

//======================================
static int cmsis_dap_quit( void )
{

  LOG_INFO("CMSIS-DAP: cmsis_dap_quit");

  cmsis_dap_cmd_DAP_Disconnect();
  cmsis_dap_cmd_DAP_LED( 0x00 );                  // Both LEDs off

  cmsis_dap_usb_close( cmsis_dap_handle );

  return ERROR_OK;
}


//======================================
static int cmsis_dap_speed(int speed)
{

  LOG_INFO("CMSIS-DAP: cmsis_dap_speed");
  if( speed > DAP_MAX_CLOCK )
  {
    LOG_INFO("reduce speed request: %dkHz to %dkHz maximum",
              speed, DAP_MAX_CLOCK );
    speed = DAP_MAX_CLOCK;
  }

  if( speed == 0 )
  {
    LOG_INFO("RTCK not supported");
    return ERROR_JTAG_NOT_IMPLEMENTED;
  }

  return cmsis_dap_cmd_DAP_SWJ_Clock( speed );
}

//======================================
static int cmsis_dap_speed_div( int speed, int* khz )
{
  *khz = speed;
  return ERROR_OK;
}

//======================================
static int cmsis_dap_khz( int khz, int *jtag_speed )
{
  *jtag_speed = khz;
  return ERROR_OK;
}


//============================================================================
COMMAND_HANDLER(cmsis_dap_handle_info_command)
{
  if( cmsis_dap_get_version_info() == ERROR_OK )
  {
    cmsis_dap_get_status();
  }

  return ERROR_OK;
}


//============================================================================
static const struct command_registration cmsis_dap_subcommand_handlers[] =
{
//  {
//    .name = "caps",
//    .handler = &cmsis_dap_handle_caps_command,
//    .mode = COMMAND_EXEC,
//    .help = "show jlink capabilities",
//  },
  {
    .name = "info",
    .handler = &cmsis_dap_handle_info_command,
    .mode = COMMAND_EXEC,
    .usage = "cmsis-dap info",
    .help = "show cmsis-dap info",
  },
//  {
//    .name = "config",
//    .handler = &cmsis_dap_handle_config_command,
//    .mode = COMMAND_EXEC,
//    .help = "access cmsis_dap configuration, "
//    "if no argument this will dump the config",
//    .chain = cmsis_dap_config_subcommand_handlers,
//  },
  COMMAND_REGISTRATION_DONE
};

//======================================
static const struct command_registration cmsis_dap_command_handlers[] =
{
  {
    .name = "cmsis-dap",
    .mode = COMMAND_ANY,
    .help = "perform CMSIS-DAP management",
    .usage = "cmsis-dap <cmd>",
    .chain = cmsis_dap_subcommand_handlers,
  },
  COMMAND_REGISTRATION_DONE
};

//======================================
struct swd_driver cmsis_dap_swd_driver = 
{
  .init = cmsis_dap_swd_init,
  .read_reg = cmsis_dap_swd_read_reg,
  .write_reg = cmsis_dap_swd_write_reg,
  //.trace,
};

const char *cmsis_dap_transport[] = { "cmsis-dap", NULL };

//======================================
struct jtag_interface cmsis_dap_interface =
{
  .name = "cmsis-dap",
  .commands = cmsis_dap_command_handlers,
  .swd = &cmsis_dap_swd_driver,
  .transports = cmsis_dap_transport,

//  .execute_queue = jlink_execute_queue,

  .speed = cmsis_dap_speed,
  .speed_div = cmsis_dap_speed_div,
  .khz = cmsis_dap_khz,
  .init = cmsis_dap_init,
  .quit = cmsis_dap_quit,
};


//===========================================================================
// The End
//===========================================================================

