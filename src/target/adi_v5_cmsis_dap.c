/***************************************************************************
 *
 *   Copyright (C) 2010 by David Brownell
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the
 *   Free Software Foundation, Inc.,
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 ***************************************************************************/

/**
 * @file
 * Utilities to support ARM "Serial Wire Debug" (SWD), a low pin-count debug
 * link protocol used in cases where JTAG is not wanted.  This is coupled to
 * recent versions of ARM's "CoreSight" debug framework.  This specific code
 * is a transport level interface, with "target/arm_adi_v5.[hc]" code
 * understanding operation semantics, shared with the JTAG transport.
 *
 * Single-DAP support only.
 *
 * for details, see "ARM IHI 0031A"
 * ARM Debug Interface v5 Architecture Specification
 * especially section 5.3 for SWD protocol
 *
 * On many chips (most current Cortex-M3 parts) SWD is a run-time alternative
 * to JTAG.  Boards may support one or both.  There are also SWD-only chips,
 * (using SW-DP not SWJ-DP).
 *
 * Even boards that also support JTAG can benefit from SWD support, because
 * usually there's no way to access the SWO trace view mechanism in JTAG mode.
 * That is, trace access may require SWD support.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>

#include <transport/transport.h>
#include <jtag/interface.h>

#include <jtag/swd.h>

#define CMSIS_CMD_DP            (0   << 0)    /* set only for AP access */
#define CMSIS_CMD_AP            (1   << 0)    /* set only for AP access */
#define CMSIS_CMD_READ          (1   << 1)    /* set only for read access */
#define CMSIS_CMD_WRITE         (0   << 1)    /* set only for read access */
#define CMSIS_CMD_A32(n)        ((n)&0x0C)    /* bits A[3:2] of register addr */
#define CMSIS_CMD_VAL_MATCH     (1   << 4)    /* Value Match */
#define CMSIS_CMD_MATCH_MSK     (1   << 5)    /* Match Mask */


// YUK! - but this is currectly a global....
extern struct jtag_interface *jtag_interface;

static int cmsis_dap_queue_dp_read(struct adiv5_dap *dap, unsigned reg,
                                    uint32_t *data)
{
  LOG_INFO("CMSIS-ADI: cmsis_dap_queue_dp_read %d", reg );
  /* REVISIT status return vs ack ... */
  return jtag_interface->swd->read_reg( (CMSIS_CMD_DP|
                                          CMSIS_CMD_READ|
                                          CMSIS_CMD_A32(reg)), data);
}

static int cmsis_dap_queue_idcode_read(struct adiv5_dap *dap,
                                        uint8_t *ack, uint32_t *data)
{
  LOG_INFO("CMSIS-ADI: cmsis_dap_queue_idcode_read");
  int status = cmsis_dap_queue_dp_read(dap, DP_IDCODE, data);
  if (status < 0)
    return status;
  *ack = status;
  return ERROR_OK;
}

static int (cmsis_dap_queue_dp_write)(struct adiv5_dap *dap, unsigned reg,
                                      uint32_t data)
{
  LOG_INFO("CMSIS-ADI: cmsis_dap_queue_dp_write %d", reg);
  /* REVISIT status return vs ack ... */
  return jtag_interface->swd->write_reg( (CMSIS_CMD_DP|
                                          CMSIS_CMD_WRITE|
                                          CMSIS_CMD_A32(reg)), data);
}


static int (cmsis_dap_queue_ap_read)(struct adiv5_dap *dap, unsigned reg,
                                      uint32_t *data)
{
  LOG_INFO("CMSIS-ADI: cmsis_dap_queue_ap_read %d", reg);
  /* REVISIT  APSEL ... */
  /* REVISIT status return ... */
  return jtag_interface->swd->read_reg( (CMSIS_CMD_AP|
                                          CMSIS_CMD_READ|
                                          CMSIS_CMD_A32(reg)), data);
}

static int (cmsis_dap_queue_ap_write)(struct adiv5_dap *dap, unsigned reg,
                                      uint32_t data)
{
  LOG_INFO("CMSIS-ADI: cmsis_dap_queue_ap_write %d", reg);
  /* REVISIT  APSEL ... */
  /* REVISIT status return ... */
  return jtag_interface->swd->write_reg( (CMSIS_CMD_AP|
                                          CMSIS_CMD_WRITE|
                                          CMSIS_CMD_A32(reg)), data);
}

static int (cmsis_dap_queue_ap_abort)(struct adiv5_dap *dap, uint8_t *ack)
{
  LOG_INFO("CMSIS-ADI: cmsis_dap_queue_ap_abort");
  return ERROR_FAIL;
}

/** Executes all queued DAP operations. */
static int cmsis_dap_run(struct adiv5_dap *dap)
{
  LOG_INFO("CMSIS-ADI: cmsis_dap_run");
  /* for now the CMSIS-DAP interface hard-wires a zero-size queue.  */

  /* FIXME but we still need to check and scrub
   * any hardware errors ...
   */
  return ERROR_OK;
}

const struct dap_ops cmsis_dap_ops = {
  //.is_swd = true,

  .queue_idcode_read = cmsis_dap_queue_idcode_read,
  .queue_dp_read = cmsis_dap_queue_dp_read,
  .queue_dp_write = cmsis_dap_queue_dp_write,
  .queue_ap_read = cmsis_dap_queue_ap_read,
  .queue_ap_write = cmsis_dap_queue_ap_write,
  .queue_ap_abort = cmsis_dap_queue_ap_abort,
  .run = cmsis_dap_run,
};


COMMAND_HANDLER(handle_swd_wcr)
{
  int retval;
  struct target *target = get_current_target(CMD_CTX);
  struct arm *arm = target_to_arm(target);
  struct adiv5_dap *dap = arm->dap;
  uint32_t wcr;
  unsigned trn, scale = 0;

  switch (CMD_ARGC) {
  /* no-args: just dump state */
  case 0:
    retval = dap_queue_dp_read(dap, DP_WCR, &wcr);
    if (retval == ERROR_OK)
      dap->ops->run(dap);
    if (retval != ERROR_OK) {
      LOG_ERROR("can't read WCR?");
      return retval;
    }

    command_print(CMD_CTX,
      "turnaround=%d, prescale=%d",
      WCR_TO_TRN(wcr),
      WCR_TO_PRESCALE(wcr));
  return ERROR_OK;

  case 2:		/* TRN and prescale */
    COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], scale);
    if (scale > 7) {
      LOG_ERROR("prescale %d is too big", scale);
      return ERROR_FAIL;
    }
    /* FALL THROUGH */

  case 1:		/* TRN only */
    COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], trn);
    if (trn < 1 || trn > 4) {
      LOG_ERROR("turnaround %d is invalid", trn);
      return ERROR_FAIL;
    }

    wcr = ((trn - 1) << 8) | scale;
    /* FIXME
     * write WCR ...
     * then, re-init adapter with new TRN
     */
    LOG_ERROR("can't yet modify WCR");
    return ERROR_FAIL;

  default:	/* too many arguments */
    return ERROR_COMMAND_SYNTAX_ERROR;
  }
}

static const struct command_registration cmsis_dap_commands[] = {
  {
    /*
     * Set up SWD and JTAG targets identically, unless/until
     * infrastructure improves ...  meanwhile, ignore all
     * JTAG-specific stuff like IR length for SWD.
     *
     * REVISIT can we verify "just one SWD DAP" here/early?
     */
    .name = "newdap",
    .jim_handler = jim_jtag_newtap,
    .mode = COMMAND_CONFIG,
    .help = "declare a new CMSIS-DAP"
  },
  {
    .name = "wcr",
    .handler = handle_swd_wcr,
    .mode = COMMAND_ANY,
    .help = "display or update DAP's WCR register",
    .usage = "turnaround (1..4), prescale (0..7)",
  },

  COMMAND_REGISTRATION_DONE
};

static const struct command_registration cmsis_dap_handlers[] = {
  {
    .name = "cmsis-dap",
    .mode = COMMAND_ANY,
    .help = "cmsis_dap command group",
    .chain = cmsis_dap_commands,
  },
  COMMAND_REGISTRATION_DONE
};

static int cmsis_dap_select(struct command_context *ctx)
{
  LOG_INFO("CMSIS-ADI: cmsis_dap_select");
  int retval;

  retval = register_commands(ctx, NULL, cmsis_dap_handlers);

  if (retval != ERROR_OK)
    return retval;

   /* be sure driver is in SWD mode; start
    * with hardware default TRN (1), it can be changed later
    */
  const struct swd_driver *swd = jtag_interface->swd;  
  if (!swd || !swd->read_reg || !swd->write_reg || !swd->init) {
    LOG_DEBUG("no SWD driver?");
    return ERROR_FAIL;
  }

  retval = swd->init(1);
  if (retval != ERROR_OK) {
    LOG_DEBUG("can't init CMSIS-DAP driver");
    return retval;
  }

  if( ctx->current_target )
  {
    /* force DAP into SWD mode (not JTAG) */
  }

  return retval;
}

static int cmsis_dap_init(struct command_context *ctx)
{
  struct target *target = get_current_target(ctx);
  struct arm *arm = target_to_arm(target);
  struct adiv5_dap *dap = arm->dap;
  uint32_t idcode;
  int status;

  LOG_INFO("CMSIS-ADI: cmsis_dap_init !!");
  // Force the DAP's ops vector for CMSIS-DAP mode.
  // messy - is there a better way?
  arm->dap->ops = &cmsis_dap_ops;


  /* FIXME validate transport config ... is the
   * configured DAP present (check IDCODE)?
   * Is *only* one DAP configured?
   *
   * MUST READ IDCODE
   */

  /* Note, debugport_init() does setup too */

  uint8_t ack;

  status = cmsis_dap_queue_idcode_read(dap, &ack, &idcode);

  if (status == ERROR_OK)
    LOG_INFO("CMSIS-DAP IDCODE %#8.8x", idcode);


  return status;

}


static struct transport cmsis_dap_transport = {
  .name = "cmsis-dap",
  .select = cmsis_dap_select,
  .init = cmsis_dap_init,
};

static void cmsis_dap_constructor(void) __attribute__((constructor));
static void cmsis_dap_constructor(void)
{
  transport_register(&cmsis_dap_transport);
}



/** Returns true if the current debug session
 * is using CMSIS-DAP as its transport.
 */
bool transport_is_cmsis_dap(void)
{
  return get_current_transport() == &cmsis_dap_transport;
}


