/*******************************************************************************
 * arch/arm/src/samdl/sam_i2c.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMD/SAML Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *    Copyright (c) 2011, Atmel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX, Atmel, nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/i2c.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/sam_pmc.h"
#include "chip/sam_pinmap.h"

#include "sam_periphclks.h"
#include "sam_pio.h"
#include "sam_i2c.h"

#if defined(CONFIG_SAM_I2C0) || defined(CONFIG_SAM_I2C1) || \
    defined(CONFIG_SAM_I2C2) || defined(CONFIG_SAM_I2C3) || \
    defined(CONFIG_SAM_I2C4) || defined(CONFIG_SAM_I2C5)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/

#ifndef CONFIG_SAM_I2C0_FREQUENCY
#  define CONFIG_SAM_I2C0_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C1_FREQUENCY
 #define CONFIG_SAM_I2C1_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C2_FREQUENCY
 #define CONFIG_SAM_I2C2_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C3_FREQUENCY
 #define CONFIG_SAM_I2C3_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C4_FREQUENCY
 #define CONFIG_SAM_I2C4_FREQUENCY 100000
#endif

#ifndef CONFIG_SAM_I2C5_FREQUENCY
 #define CONFIG_SAM_I2C5_FREQUENCY 100000
#endif

/* Driver internal definitions *************************************************/
/* If verbose I2C debug output is enable, then allow more time before we declare
 * a timeout.  The debug output from i2c_interrupt will really slow things down!
 *
 * With a very slow clock (say 100,000 Hz), less than 100 usec would be required
 * to transfer on byte.  So these define a "long" timeout.
 */

#if defined(CONFIG_DEBUG_I2C) && defined(CONFIG_DEBUG_VERBOSE)
#  define I2C_TIMEOUT_MSPB (50)  /* 50 msec/byte */
#else
#  define I2C_TIMEOUT_MSPB (5)   /* 5 msec/byte */
#endif

/* Clocking to the I2C module(s) is provided by the main clock, divided down
 * as necessary.
 */

#define I2C_MAX_FREQUENCY 66000000   /* Maximum I2C frequency */

/* Debug ***********************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#  define i2cdbg    dbg
#  define i2cvdbg   vdbg
#  define i2clldbg  lldbg
#  define i2cllvdbg llvdbg
#else
#  define i2cdbg(x...)
#  define i2cvdbg(x...)
#  define i2clldbg(x...)
#  define i2cllvdbg(x...)
#endif

/*******************************************************************************
 * Private Types
 *******************************************************************************/
/* Invariant attributes of a I2C bus */

struct i2c_attr_s
{
  uint8_t             i2c;          /* I2C device number (for debug output) */
  uint8_t             sercom;       /* Identifies the SERCOM peripheral */
  uint8_t             irq;          /* SERCOM IRQ number */
  uint8_t             gclkgen;      /* Source GCLK generator */
  uint8_t             slowgen;      /* Slow GCLK generator */
  port_pinset_t       pad0;         /* Pin configuration for PAD0 */
  port_pinset_t       pad1;         /* Pin configuration for PAD1 */
  uint32_t            muxconfig;    /* Pad multiplexing configuration */
  uint32_t            srcfreq;      /* Source clock frequency */
  uintptr_t           base;         /* Base address of I2C registers */
  bool                runinstdby;   /* Run in Stand-by ? */
  uint32_t            sdaholdtime;  /* Hold time after start bit */
  uint32_t            speed;        /* I2C Speed: Standard; Fast; High */
  bool                scllowtout;   /* SCL low timeout */
  uint32_t            inactout;     /* Inactive Timeout */
  bool                sclstretch;   /* SCL stretch only after ACK */
  bool                sclslvextout; /* SCL Slave extened timeout */
  bool                sclmstextout; /* SCL Master extend timeout */
  xcpt_t              handler;      /* I2C interrupt handler */
};

/* State of a I2C bus */

struct sam_i2c_dev_s
{
  struct i2c_dev_s    dev;        /* Generic I2C device */
  const struct i2c_attr_s *attr;  /* Invariant attributes of I2C device */
  struct i2c_msg_s    *msg;       /* Message list */
  uint32_t            i2cclk;     /* I2C input clock frequency */
#ifdef CONFIG_I2C_RESET
  uint32_t            frequency;  /* I2C transfer clock frequency */
#endif
  uint16_t            address;    /* Slave address */
  uint16_t            flags;      /* Transfer flags */
  uint8_t             msgc;       /* Number of message in the message list */

  sem_t               exclsem;    /* Only one thread can access at a time */
  sem_t               waitsem;    /* Wait for I2C transfer completion */
  volatile int        result;     /* The result of the transfer */
  volatile int        xfrd;       /* Number of bytes transfers */

  /* Debug stuff */

#ifdef CONFIG_SAM_I2C_REGDEBUG
   bool               wrlast;     /* Last was a write */
   uint32_t           addrlast;   /* Last address */
   uint32_t           vallast;    /* Last value */
   int                ntimes;     /* Number of times */
#endif
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Low-level helper functions */

static uint8_t  i2c_getreg8(struct sam_i2c_dev_s *priv,
                  unsigned int offset);
static void     i2c_putreg8(struct sam_i2c_dev_s *priv, uint8_t regval,
                  unsigned int offset);
static uint16_t i2c_getreg16(struct sam_i2c_dev_s *priv,
                  unsigned int offset);
static void     i2c_putreg16(struct sam_i2c_dev_s *priv, uint16_t regval,
                  unsigned int offset);
static uint32_t i2c_getreg32(struct sam_i2c_dev_s *priv,
                  unsigned int offset);
static void     i2c_putreg32(struct sam_i2c_dev_s *priv, uint32_t regval,
                  unsigned int offset);

static void i2c_takesem(sem_t *sem);
#define     i2c_givesem(sem) (sem_post(sem))

#ifdef CONFIG_SAM_I2C_REGDEBUG
static bool i2c_checkreg(struct sam_i2c_dev_s *priv, bool wr,
              uint32_t value, uintptr_t address);
static uint32_t i2c_getabs(struct sam_i2c_dev_s *priv, uintptr_t address);
static void i2c_putabs(struct sam_i2c_dev_s *priv, uintptr_t address,
              uint32_t value);
#else
# define    i2c_checkreg(priv,wr,value,address) (false)
# define    i2c_putabs(p,a,v) putreg32(v,a)
# define    i2c_getabs(p,a) getreg32(a)
#endif

static inline uint32_t i2c_getrel(struct sam_i2c_dev_s *priv,
          unsigned int offset);
static inline void i2c_putrel(struct sam_i2c_dev_s *priv, unsigned int offset,
          uint32_t value);

/* I2C transfer helper functions */

static int i2c_wait(struct sam_i2c_dev_s *priv, unsigned int size);
static void i2c_wakeup(struct sam_i2c_dev_s *priv, int result);
static int i2c_interrupt(struct sam_i2c_dev_s *priv);
#ifdef CONFIG_SAM_I2C0
static int i2c0_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAM_I2C1
static int i2c1_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAM_I2C2
static int i2c2_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAM_I2C3
static int i2c3_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAM_I2C4
static int i2c4_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAM_I2C5
static int i2c5_interrupt(int irq, FAR void *context);
#endif
static void i2c_timeout(int argc, uint32_t arg, ...);

static void i2c_startread(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg);
static void i2c_startwrite(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg);
static void i2c_startmessage(struct sam_i2c_dev_s *priv, struct i2c_msg_s *msg);

/* I2C device operations */

static uint32_t i2c_setfrequency(FAR struct i2c_dev_s *dev,
          uint32_t frequency);
static int i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer,
          int buflen);
static int i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen);
#ifdef CONFIG_I2C_WRITEREAD
static int i2c_writeread(FAR struct i2c_dev_s *inst, const uint8_t *wbuffer,
          int wbuflen, uint8_t *rbuffer, int rbuflen);
#endif
#ifdef CONFIG_I2C_TRANSFER
static int i2c_transfer(FAR struct i2c_dev_s *dev,
          FAR struct i2c_msg_s *msgs, int count);
#endif
#ifdef CONFIG_I2C_SLAVE
static int i2c_setownaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int i2c_registercallback(FAR struct i2c_dev_s *dev,
          int (*callback)(FAR void *arg), FAR void *arg);
#endif

/* Initialization */

static uint32_t i2c_hw_setfrequency(struct sam_i2c_dev_s *priv,
          uint32_t frequency);
static void i2c_hw_initialize(struct sam_i2c_dev_s *priv, uint32_t frequency);
static void i2c_wait_synchronization(struct sam_i2c_dev_s *priv);
static void i2c_pad_configure(struct sam_i2c_dev_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

#ifdef CONFIG_SAM_I2C0
static const struct i2c_attr_s g_i2c0attr =
{
  .i2c          = 0,
  .sercom       = 0,
  .irq          = SAM_IRQ_SERCOM0,
  .gclkgen      = BOARD_SERCOM0_GCLKGEN,
  .slowgen      = BOARD_SERCOM0_SLOW_GCLKGEN,
  .pad0         = BOARD_SERCOM0_PINMAP_PAD0,
  .pad1         = BOARD_SERCOM0_PINMAP_PAD1,
  .muxconfig    = BOARD_SERCOM0_MUXCONFIG,
  .srcfreq      = BOARD_SERCOM0_FREQUENCY,
  .base         = SAM_SERCOM0_BASE,
  .runinstdby   = BOARD_SERCOM0_I2C_RUNINSTDBY,
  .sdaholdtime  = BOARD_SERCOM0_I2C_START_HOLD_TIME,
  .speed        = BOARD_SERCOM0_I2C_SPEED,
  .scllowtout   = BOARD_SERCOM0_I2C_SCL_LOW_TIMEOUT,
  .inactout     = BOARD_SERCOM0_I2C_INACTIVE_TIMEOUT,
  .sclstretch   = BOARD_SERCOM0_I2C_SCL_STRETCH_ACK,
  .sclslvextout = BOARD_SERCOM0_I2C_SCL_SLAVE_EXT_TIMEOUT,
  .sclmstextout = BOARD_SERCOM0_I2C_SCL_MASTER_EXT_TIMEOUT,
  .handler      = i2c0_interrupt,
};

static struct sam_i2c_dev_s g_i2c0;
#endif
#ifdef CONFIG_SAM_I2C1
static const struct i2c_attr_s g_i2c1attr =
{
  .i2c          = 1,
  .sercom       = 1,
  .irq          = SAM_IRQ_SERCOM1,
  .gclkgen      = BOARD_SERCOM1_GCLKGEN,
  .slowgen      = BOARD_SERCOM1_SLOW_GCLKGEN,
  .pad0         = BOARD_SERCOM1_PINMAP_PAD0,
  .pad1         = BOARD_SERCOM1_PINMAP_PAD1,
  .muxconfig    = BOARD_SERCOM1_MUXCONFIG,
  .srcfreq      = BOARD_SERCOM1_FREQUENCY,
  .base         = SAM_SERCOM1_BASE,
  .runinstdby   = BOARD_SERCOM1_I2C_RUNINSTDBY,
  .sdaholdtime  = BOARD_SERCOM1_I2C_START_HOLD_TIME,
  .speed        = BOARD_SERCOM1_I2C_SPEED,
  .scllowtout   = BOARD_SERCOM1_I2C_SCL_LOW_TIMEOUT,
  .inactout     = BOARD_SERCOM1_I2C_INACTIVE_TIMEOUT,
  .sclstretch   = BOARD_SERCOM1_I2C_SCL_STRETCH_ACK,
  .sclslvextout = BOARD_SERCOM1_I2C_SCL_SLAVE_EXT_TIMEOUT,
  .sclmstextout = BOARD_SERCOM1_I2C_SCL_MASTER_EXT_TIMEOUT,
  .handler      = i2c1_interrupt,
};

static struct sam_i2c_dev_s g_i2c1;
#endif

#ifdef CONFIG_SAM_I2C2
static const struct i2c_attr_s g_i2c2attr =
{
  .i2c          = 2,
  .sercom       = 2,
  .irq          = SAM_IRQ_SERCOM2,
  .gclkgen      = BOARD_SERCOM2_GCLKGEN,
  .slowgen      = BOARD_SERCOM2_SLOW_GCLKGEN,
  .pad0         = BOARD_SERCOM2_PINMAP_PAD0,
  .pad1         = BOARD_SERCOM2_PINMAP_PAD1,
  .muxconfig    = BOARD_SERCOM2_MUXCONFIG,
  .srcfreq      = BOARD_SERCOM2_FREQUENCY,
  .base         = SAM_SERCOM2_BASE,
  .runinstdby   = BOARD_SERCOM2_I2C_RUNINSTDBY,
  .sdaholdtime  = BOARD_SERCOM2_I2C_START_HOLD_TIME,
  .speed        = BOARD_SERCOM2_I2C_SPEED,
  .scllowtout   = BOARD_SERCOM2_I2C_SCL_LOW_TIMEOUT,
  .inactout     = BOARD_SERCOM2_I2C_INACTIVE_TIMEOUT,
  .sclstretch   = BOARD_SERCOM2_I2C_SCL_STRETCH_ACK,
  .sclslvextout = BOARD_SERCOM2_I2C_SCL_SLAVE_EXT_TIMEOUT,
  .sclmstextout = BOARD_SERCOM2_I2C_SCL_MASTER_EXT_TIMEOUT,
  .handler      = i2c2_interrupt,
};

static struct sam_i2c_dev_s g_i2c2;
#endif

#ifdef CONFIG_SAM_I2C3
static const struct i2c_attr_s g_i2c3attr =
{
  .i2c          = 3,
  .sercom       = 3,
  .irq          = SAM_IRQ_SERCOM3,
  .gclkgen      = BOARD_SERCOM3_GCLKGEN,
  .slowgen      = BOARD_SERCOM3_SLOW_GCLKGEN,
  .pad0         = BOARD_SERCOM3_PINMAP_PAD0,
  .pad1         = BOARD_SERCOM3_PINMAP_PAD1,
  .muxconfig    = BOARD_SERCOM3_MUXCONFIG,
  .srcfreq      = BOARD_SERCOM3_FREQUENCY,
  .base         = SAM_SERCOM3_BASE,
  .runinstdby   = BOARD_SERCOM3_I2C_RUNINSTDBY,
  .sdaholdtime  = BOARD_SERCOM3_I2C_START_HOLD_TIME,
  .speed        = BOARD_SERCOM3_I2C_SPEED,
  .scllowtout   = BOARD_SERCOM3_I2C_SCL_LOW_TIMEOUT,
  .inactout     = BOARD_SERCOM3_I2C_INACTIVE_TIMEOUT,
  .sclstretch   = BOARD_SERCOM3_I2C_SCL_STRETCH_ACK,
  .sclslvextout = BOARD_SERCOM3_I2C_SCL_SLAVE_EXT_TIMEOUT,
  .sclmstextout = BOARD_SERCOM3_I2C_SCL_MASTER_EXT_TIMEOUT,
  .handler      = i2c3_interrupt,
};

static struct sam_i2c_dev_s g_i2c3;
#endif

#ifdef CONFIG_SAM_I2C4
static const struct i2c_attr_s g_i2c4attr =
{
  .i2c          = 4,
  .sercom       = 4,
  .irq          = SAM_IRQ_SERCOM4,
  .gclkgen      = BOARD_SERCOM4_GCLKGEN,
  .slowgen      = BOARD_SERCOM4_SLOW_GCLKGEN,
  .pad0         = BOARD_SERCOM4_PINMAP_PAD0,
  .pad1         = BOARD_SERCOM4_PINMAP_PAD1,
  .muxconfig    = BOARD_SERCOM4_MUXCONFIG,
  .srcfreq      = BOARD_SERCOM4_FREQUENCY,
  .base         = SAM_SERCOM4_BASE,
  .runinstdby   = BOARD_SERCOM4_I2C_RUNINSTDBY,
  .sdaholdtime  = BOARD_SERCOM4_I2C_START_HOLD_TIME,
  .speed        = BOARD_SERCOM4_I2C_SPEED,
  .scllowtout   = BOARD_SERCOM4_I2C_SCL_LOW_TIMEOUT,
  .inactout     = BOARD_SERCOM4_I2C_INACTIVE_TIMEOUT,
  .sclstretch   = BOARD_SERCOM4_I2C_SCL_STRETCH_ACK,
  .sclslvextout = BOARD_SERCOM4_I2C_SCL_SLAVE_EXT_TIMEOUT,
  .sclmstextout = BOARD_SERCOM4_I2C_SCL_MASTER_EXT_TIMEOUT,
  .handler      = i2c4_interrupt,
};

static struct sam_i2c_dev_s g_i2c4;
#endif

#ifdef CONFIG_SAM_I2C5
static const struct i2c_attr_s g_i2c5attr =
{
  .i2c          = 5,
  .sercom       = 5,
  .irq          = SAM_IRQ_SERCOM5,
  .gclkgen      = BOARD_SERCOM5_GCLKGEN,
  .slowgen      = BOARD_SERCOM5_SLOW_GCLKGEN,
  .pad0         = BOARD_SERCOM5_PINMAP_PAD0,
  .pad1         = BOARD_SERCOM5_PINMAP_PAD1,
  .muxconfig    = BOARD_SERCOM5_MUXCONFIG,
  .srcfreq      = BOARD_SERCOM5_FREQUENCY,
  .base         = SAM_SERCOM5_BASE,
  .runinstdby   = BOARD_SERCOM5_I2C_RUNINSTDBY,
  .sdaholdtime  = BOARD_SERCOM5_I2C_START_HOLD_TIME,
  .speed        = BOARD_SERCOM5_I2C_SPEED,
  .scllowtout   = BOARD_SERCOM5_I2C_SCL_LOW_TIMEOUT,
  .inactout     = BOARD_SERCOM5_I2C_INACTIVE_TIMEOUT,
  .sclstretch   = BOARD_SERCOM5_I2C_SCL_STRETCH_ACK,
  .sclslvextout = BOARD_SERCOM5_I2C_SCL_SLAVE_EXT_TIMEOUT,
  .sclmstextout = BOARD_SERCOM5_I2C_SCL_MASTER_EXT_TIMEOUT,
  .handler      = i2c5_interrupt,
};

static struct sam_i2c_dev_s g_i2c5;
#endif

struct i2c_ops_s g_i2cops =
{
  .setfrequency     = i2c_setfrequency,
  .setaddress       = i2c_setaddress,
  .write            = i2c_write,
  .read             = i2c_read,
#ifdef CONFIG_I2C_WRITEREAD
  .writeread        = i2c_writeread,
#endif
#ifdef CONFIG_I2C_TRANSFER
  .transfer         = i2c_transfer
#endif
#ifdef CONFIG_I2C_SLAVE
  .setownaddress    = i2c_setownaddress
  .registercallback = i2c_registercallback
#endif
};

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/

/************************************************************************************
 * Name: i2c_getreg8
 *
 * Description:
 *   Get a 8-bit register value by offset
 *
 ************************************************************************************/
static uint8_t  i2c_getreg8(struct sam_i2c_dev_s *priv,
                            unsigned int offset)
{
  return getreg8(priv->attr->base + offset);
}

/************************************************************************************
 * Name: i2c_putreg8
 *
 * Description:
 *  Put a 8-bit register value by offset
 *
 ************************************************************************************/
static void     i2c_putreg8(struct sam_i2c_dev_s *priv, uint8_t regval,
                            unsigned int offset)
{
  putreg8(regval, priv->attr->base + offset);
}

/************************************************************************************
 * Name: i2c_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ************************************************************************************/
static uint16_t  i2c_getreg16(struct sam_i2c_dev_s *priv,
                              unsigned int offset)
{
  return getreg16(priv->attr->base + offset);
}

/************************************************************************************
 * Name: i2c_putreg16
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ************************************************************************************/
static void     i2c_putreg16(struct sam_i2c_dev_s *priv, uint16_t regval,
                             unsigned int offset)
{
  putreg16(regval, priv->attr->base + offset);
}

/************************************************************************************
 * Name: i2c_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ************************************************************************************/
static uint32_t  i2c_getreg32(struct sam_i2c_dev_s *priv,
                              unsigned int offset)
{
  return getreg32(priv->attr->base + offset);
}

/************************************************************************************
 * Name: i2c_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ************************************************************************************/
static void     i2c_putreg32(struct sam_i2c_dev_s *priv, uint32_t regval,
                             unsigned int offset)
{
  putreg32(regval, priv->attr->base + offset);
}

/****************************************************************************
 * Name: i2c_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wake-ups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: i2c_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   false: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM_I2C_REGDEBUG
static bool i2c_checkreg(struct sam_i2c_dev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      value   == priv->vallast &&  /* Same value? */
      address == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = value;
      priv->addrlast = address;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: i2c_getabs
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 ****************************************************************************/

#ifdef CONFIG_SAM_I2C_REGDEBUG
static uint32_t i2c_getabs(struct sam_i2c_dev_s *priv, uintptr_t address)
{
  uint32_t value = getreg32(address);

  if (i2c_checkreg(priv, false, value, address))
    {
      lldbg("%08x->%08x\n", address, value);
    }

  return value;
}
#endif

/****************************************************************************
 * Name: i2c_putabs
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 ****************************************************************************/

#ifdef CONFIG_SAM_I2C_REGDEBUG
static void i2c_putabs(struct sam_i2c_dev_s *priv, uintptr_t address,
                       uint32_t value)
{
  if (i2c_checkreg(priv, true, value, address))
    {
      lldbg("%08x<-%08x\n", address, value);
    }

  putreg32(value, address);
}
#endif

/****************************************************************************
 * Name: i2c_getrel
 *
 * Description:
 *  Read a I2C register using an offset relative to the I2C base address
 *
 ****************************************************************************/

static inline uint32_t i2c_getrel(struct sam_i2c_dev_s *priv, unsigned int offset)
{
  return i2c_getabs(priv, priv->attr->base + offset);
}

/****************************************************************************
 * Name: i2c_putrel
 *
 * Description:
 *  Write a value to a I2C register using an offset relative to the I2C base
 *  address.
 *
 ****************************************************************************/

static inline void i2c_putrel(struct sam_i2c_dev_s *priv, unsigned int offset,
                              uint32_t value)
{
  i2c_putabs(priv, priv->attr->base + offset, value);
}

/****************************************************************************
 * I2C transfer helper functions
 ****************************************************************************/

/*******************************************************************************
 * Name: i2c_wait
 *
 * Description:
 *   Perform a I2C transfer start
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 *******************************************************************************/

static int i2c_wait(struct i2c_dev_s *priv, unsigned int size)
{
  uint32_t timeout;

  /* Calculate a timeout value based on the size of the transfer
   *
   *   ticks = msec-per-byte * bytes / msec-per-tick
   *
   * There is no concern about arithmetic overflow for reasonable transfer sizes.
   */

  timeout = MSEC2TICK(I2C_TIMEOUT_MSPB);
  if (timeout < 1)
    {
      timeout = 1;
    }

  /* Then start the timeout.  This timeout is needed to avoid hangs if/when an
   * I2C transfer stalls.
   */

  wd_start(priv->timeout, timeout, i2c_timeout, 1, (uint32_t)priv);

  /* Wait for either the I2C transfer or the timeout to complete */

  do
    {
      i2cvdbg("I2C%d Waiting...\n", priv->attr->i2c);
      i2c_takesem(&priv->waitsem);
      i2cvdbg("I2C%d Awakened with result: %d\n",
              priv->attr->i2c, priv->result);
    }
  while (priv->result == -EBUSY);

  /* We get here via i2c_wakeup.  The watchdog timer has been disabled and
   * all further interrupts for the I2C have been disabled.
   */

  return priv->result;
}

/*******************************************************************************
 * Name: i2c_wakeup
 *
 * Description:
 *   A terminal event has occurred.  Wake-up the waiting thread
 *
 *******************************************************************************/

static void i2c_wakeup(struct i2c_dev_s *priv, int result)
{
  /* Cancel any pending timeout */

  wd_cancel(priv->timeout);

  /* Disable any further I2C interrupts */

  i2c_putrel(priv, SAM_I2C_IDR_OFFSET, I2C_INT_ALL);

  /* Wake up the waiting thread with the result of the transfer */

  priv->result = result;
  i2c_givesem(&priv->waitsem);
}

/*******************************************************************************
 * Name: i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 *******************************************************************************/

static int i2c_interrupt(struct i2c_dev_s *priv)
{
  struct i2c_msg_s *msg;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;
  uint32_t regval;

  /* Retrieve masked interrupt status */

  sr      = i2c_getrel(priv, SAM_I2C_SR_OFFSET);
  imr     = i2c_getrel(priv, SAM_I2C_IMR_OFFSET);
  pending = sr & imr;

  i2cllvdbg("I2C%d pending: %08x\n", priv->attr->i2c, pending);

  /* Byte received */

  msg = priv->msg;
  if ((pending & I2C_INT_RXRDY) != 0)
    {
      msg->buffer[priv->xfrd] = i2c_getrel(priv, SAM_I2C_RHR_OFFSET);
      priv->xfrd++;

      /* Check for transfer complete */

      if (priv->xfrd >= msg->length)
        {
          /* The transfer is complete.  Disable the RXRDY interrupt and
           * enable the TXCOMP interrupt
           */

          i2c_putrel(priv, SAM_I2C_IDR_OFFSET, I2C_INT_RXRDY);
          i2c_putrel(priv, SAM_I2C_IER_OFFSET, I2C_INT_TXCOMP);
        }

      /* Not yet complete, but will the next be the last byte? */

      else if (priv->xfrd == (msg->length - 1))
        {
          /* Yes, set the stop signal */

          i2c_putrel(priv, SAM_I2C_CR_OFFSET, I2C_CR_STOP);
        }
    }

  /* Byte sent*/

  else if ((pending & I2C_INT_TXRDY) != 0)
    {
      /* Transfer finished? */

      if (priv->xfrd >= msg->length)
        {
          /* The transfer is complete.  Disable the TXRDY interrupt and
           * enable the TXCOMP interrupt
           */

          i2c_putrel(priv, SAM_I2C_IDR_OFFSET, I2C_INT_TXRDY);
          i2c_putrel(priv, SAM_I2C_IER_OFFSET, I2C_INT_TXCOMP);

          /* Send the STOP condition */

          regval  = i2c_getrel(priv, SAM_I2C_CR_OFFSET);
          regval |= I2C_CR_STOP;
          i2c_putrel(priv, SAM_I2C_CR_OFFSET, regval);
        }

      /* No, there are more bytes remaining to be sent */

      else
        {
          i2c_putrel(priv, SAM_I2C_THR_OFFSET, msg->buffer[priv->xfrd]);
          priv->xfrd++;
        }
    }

  /* Transfer complete */

  else if ((pending & I2C_INT_TXCOMP) != 0)
    {
      i2c_putrel(priv, SAM_I2C_IDR_OFFSET, I2C_INT_TXCOMP);

      /* Is there another message to send? */

      if (priv->msgc > 1)
        {
          /* Yes... start the next message */

          priv->msg++;
          priv->msgc--;
          i2c_startmessage(priv, priv->msg);
        }
      else
        {
          /* No.. we made it to the end of the message list with no errors.
           * Cancel any timeout and wake up the waiting thread with a
           * success indication.
           */

          i2c_wakeup(priv, OK);
        }
    }

  /* Check for errors */

  else if ((pending & I2C_INT_ERRORS) != 0)
    {
      /* Wake up the thread with an I/O error indication */

      i2clldbg("ERROR: I2C%d pending: %08x\n", priv->attr->i2c, pending);
      i2c_wakeup(priv, -EIO);
    }

  return OK;
}

#ifdef CONFIG_SAM_I2C0
static int i2c0_interrupt(int irq, FAR void *context)
{
  return i2c_interrupt(&g_i2c0);
}
#endif

#ifdef CONFIG_SAM_I2C1
static int i2c1_interrupt(int irq, FAR void *context)
{
  return i2c_interrupt(&g_i2c1);
}
#endif

#ifdef CONFIG_SAM_I2C2
static int i2c2_interrupt(int irq, FAR void *context)
{
  return i2c_interrupt(&g_i2c2);
}
#endif

#ifdef CONFIG_SAM_I2C3
static int i2c3_interrupt(int irq, FAR void *context)
{
  return i2c_interrupt(&g_i2c3);
}
#endif

#ifdef CONFIG_SAM_I2C4
static int i2c4_interrupt(int irq, FAR void *context)
{
  return i2c_interrupt(&g_i2c4);
}
#endif

#ifdef CONFIG_SAM_I2C5
static int i2c5_interrupt(int irq, FAR void *context)
{
  return i2c_interrupt(&g_i2c5);
}
#endif

/*******************************************************************************
 * Name: i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 * Assumptions:
 *   Called from the timer interrupt handler with interrupts disabled.
 *
 *******************************************************************************/

static void i2c_timeout(int argc, uint32_t arg, ...)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *)arg;

  i2clldbg("ERROR: I2C%d Timeout!\n", priv->attr->i2c);
  i2c_wakeup(priv, -ETIMEDOUT);
}

/*******************************************************************************
 * Name: i2c_startread
 *
 * Description:
 *   Start the next read message
 *
 *******************************************************************************/

static void i2c_startread(struct i2c_dev_s *priv, struct i2c_msg_s *msg)
{
  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd   = 0;

  /* Set STOP signal if only one byte is sent*/

  if (msg->length == 1)
    {
      i2c_putrel(priv, SAM_I2C_CR_OFFSET, I2C_CR_STOP);
    }

  /* Set slave address and number of internal address bytes. */

  i2c_putrel(priv, SAM_I2C_MMR_OFFSET, 0);
  i2c_putrel(priv, SAM_I2C_MMR_OFFSET, I2C_MMR_IADRSZ_NONE | I2C_MMR_MREAD |
                   I2C_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  i2c_putrel(priv, SAM_I2C_IADR_OFFSET, 0);

  /* Enable read interrupt and send the START condition */

  i2c_putrel(priv, SAM_I2C_IER_OFFSET, I2C_INT_RXRDY | I2C_INT_ERRORS);
  i2c_putrel(priv, SAM_I2C_CR_OFFSET, I2C_CR_START);
}

/*******************************************************************************
 * Name: i2c_startwrite
 *
 * Description:
 *   Start the next write message
 *
 *******************************************************************************/

static void i2c_startwrite(struct i2c_dev_s *priv, struct i2c_msg_s *msg)
{
  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd   = 0;

  /* Set slave address and number of internal address bytes. */

  i2c_putrel(priv, SAM_I2C_MMR_OFFSET, 0);
  i2c_putrel(priv, SAM_I2C_MMR_OFFSET, I2C_MMR_IADRSZ_NONE | I2C_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  i2c_putrel(priv, SAM_I2C_IADR_OFFSET, 0);

  /* Write first byte to send.*/

  i2c_putrel(priv, SAM_I2C_THR_OFFSET, msg->buffer[priv->xfrd++]);

  /* Enable write interrupt */

  i2c_putrel(priv, SAM_I2C_IER_OFFSET, I2C_INT_TXRDY | I2C_INT_ERRORS);
}

/*******************************************************************************
 * Name: i2c_startmessage
 *
 * Description:
 *   Start the next write message
 *
 *******************************************************************************/

static void i2c_startmessage(struct i2c_dev_s *priv, struct i2c_msg_s *msg)
{
  if ((msg->flags & I2C_M_READ) != 0)
    {
      i2c_startread(priv, msg);
    }
  else
    {
      i2c_startwrite(priv, msg);
    }
}

/*******************************************************************************
 * I2C device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 *******************************************************************************/

static uint32_t i2c_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *)dev;
  uint32_t actual;

  DEBUGASSERT(dev);

  i2cvdbg("sercom=%d frequency=%d\n", priv->sercom, frequency);

  /* Get exclusive access to the device */

  i2c_takesem(&priv->exclsem);

  /* And setup the clock frequency */

  actual = i2c_hw_setfrequency(priv, frequency);
  i2c_givesem(&priv->exclsem);
  return actual;
}

/*******************************************************************************
 * Name: i2c_setaddress
 *
 * Description:
 *   Set the I2C slave address for a subsequent read/write
 *
 *******************************************************************************/

static int i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *) dev;

  i2cvdbg("I2C%d address: %02x nbits: %d\n", priv->attr->i2c, addr, nbits);
  DEBUGASSERT(dev != NULL && nbits == 7);

  /* Get exclusive access to the device */

  i2c_takesem(&priv->exclsem);

  /* Remember 7- or 10-bit address */

  priv->address = addr;
  priv->flags   = (nbits == 10) ? I2C_M_TEN : 0;

  i2c_givesem(&priv->exclsem);
  return OK;
}

/*******************************************************************************
 * Name: i2c_write
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 *******************************************************************************/

static int i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *wbuffer, int wbuflen)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *) dev;
  irqstate_t flags;
  int ret;

  struct i2c_msg_s msg =
  {
    .addr   = priv->address,
    .flags  = priv->flags,
    .buffer = (uint8_t *)wbuffer,  /* Override const */
    .length = wbuflen
  };

  i2cvdbg("I2C%d buflen: %d\n", priv->attr->i2c, wbuflen);
  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the device */

  i2c_takesem(&priv->exclsem);

  /* Initiate the write */

  priv->msg  = &msg;
  priv->msgc = 1;

  /* Initiate the write operation.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = irqsave();
  i2c_startwrite(priv, &msg);

  /* And wait for the write to complete.  Interrupts will be re-enabled while
   * we are waiting.
   */

  ret = i2c_wait(priv, wbuflen);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  irqrestore(flags);
  i2c_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: i2c_read
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 *******************************************************************************/

static int i2c_read(FAR struct i2c_dev_s *dev, uint8_t *rbuffer, int rbuflen)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *)dev;
  irqstate_t flags;
  int ret;

  struct i2c_msg_s msg =
  {
    .addr   = priv->address,
    .flags  = priv->flags | I2C_M_READ,
    .buffer = rbuffer,
    .length = rbuflen
  };

  DEBUGASSERT(dev != NULL);
  i2cvdbg("I2C%d rbuflen: %d\n", priv->attr->i2c, rbuflen);

  /* Get exclusive access to the device */

  i2c_takesem(&priv->exclsem);

  /* Initiate the read */

  priv->msg  = &msg;
  priv->msgc = 1;

  /* Initiate the read operation.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = irqsave();
  i2c_startread(priv, &msg);

  /* And wait for the read to complete.  Interrupts will be re-enabled while
   * we are waiting.
   */

  ret = i2c_wait(priv, rbuflen);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  irqrestore(flags);
  i2c_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: i2c_writeread
 *
 * Description:
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_WRITEREAD
static int i2c_writeread(FAR struct i2c_dev_s *dev, const uint8_t *wbuffer,
                         int wbuflen, uint8_t *rbuffer, int rbuflen)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *)dev;
  struct i2c_msg_s msgv[2];
  irqstate_t flags;
  int ret;

  DEBUGASSERT(dev != NULL);
  i2cvdbg("I2C%d wbuflen: %d rbuflen: %d\n", priv->attr->i2c, wbuflen, rbuflen);

  /* Format two messages: The first is a write */

  msgv[0].addr   = priv->address;
  msgv[0].flags  = priv->flags;
  msgv[0].buffer = (uint8_t *)wbuffer;  /* Override const */
  msgv[0].length = wbuflen;

  /* The second is either a read (rbuflen > 0) or a write (rbuflen < 0) with
   * no restart.
   */

  if (rbuflen > 0)
    {
      msgv[1].flags = (priv->flags | I2C_M_READ);
    }
  else
    {
      msgv[1].flags = (priv->flags | I2C_M_NORESTART),
      rbuflen = -rbuflen;
    }

  msgv[1].addr   = priv->address;
  msgv[1].buffer = rbuffer;
  msgv[1].length = rbuflen;

  /* Get exclusive access to the device */

  i2c_takesem(&priv->exclsem);

  /* Initiate the read */

  priv->msg  = msgv;
  priv->msgc = 2;

  /* Initiate the write operation.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = irqsave();
  i2c_startwrite(priv, msgv);

  /* And wait for the write/read to complete.  Interrupts will be re-enabled
   * while we are waiting.
   */

  ret = i2c_wait(priv, wbuflen + rbuflen);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  irqrestore(flags);
  i2c_givesem(&priv->exclsem);
  return ret;
}
#endif

/*******************************************************************************
 * Name: i2c_setownaddress
 *
 * Description:
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_SLAVE
static int i2c_setownaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
#error Not implemented
  return -ENOSYS;
}
#endif

/*******************************************************************************
 * Name: i2c_registercallback
 *
 * Description:
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_SLAVE
static int i2c_registercallback((FAR struct i2c_dev_s *dev,
                                int (*callback)(FAR void *arg), FAR void *arg)
{
#error Not implemented
  return -ENOSYS;
}
#endif

/*******************************************************************************
 * Name: i2c_transfer
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value on failure.
 *
 *******************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
static int i2c_transfer(FAR struct i2c_dev_s *dev,
                        FAR struct i2c_msg_s *msgs, int count)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *)dev;
  irqstate_t flags;
  unsigned int size;
  int i;
  int ret;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);
  i2cvdbg("I2C%d count: %d\n", priv->attr->i2c, count);

  /* Calculate the total transfer size so that we can calculate a reasonable
   * timeout value.
   */

  size = 0;
  for (i = 0; i < count; i++)
    {
      size += msgs[i].length;
    }

  DEBUGASSERT(size > 0);

  /* Get exclusive access to the device */

  i2c_takesem(&priv->exclsem);

  /* Initiate the message transfer */

  priv->msg  = msgs;
  priv->msgc = count;

  /* Initiate the transfer.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = irqsave();
  i2c_startmessage(priv, msgs);

  /* And wait for the transfers to complete.  Interrupts will be re-enabled
   * while we are waiting.
   */

  ret = i2c_wait(priv, size);
  if (ret < 0)
    {
      i2cdbg("ERROR: Transfer failed: %d\n", ret);
    }

  irqrestore(flags);
  i2c_givesem(&priv->exclsem);
  return ret;
}
#endif

/*******************************************************************************
 * Initialization
 *******************************************************************************/

/****************************************************************************
 * Name: i2c_enableclk
 *
 * Description:
 *   Enable clocking on the selected I2C
 *
 ****************************************************************************/

static void i2c_enableclk(struct sam_i2c_dev_s *priv)
{
  int pid;

  /* Get the peripheral ID associated with the I2C device port and enable
   * clocking to the I2C block.
   */

  pid = priv->attr->pid;
  if (pid < 32)
    {
      sam_enableperiph0(pid);
    }
  else
    {
      sam_enableperiph1(pid);
    }
}

/*******************************************************************************
 * Name: i2c_hw_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 *******************************************************************************/

static uint32_t i2c_hw_setfrequency(struct sam_i2c_dev_s *priv, uint32_t frequency)
{
  uint32_t maxfreq;
  uint32_t actual;
  uint32_t baud;
  uint32_t ctrla;

  i2cvdbg("sercom=%d frequency=%d\n", priv->attr->sercom, frequency);

  /* Check if the configured BAUD is within the valid range */

  maxfreq = (priv->attr->srcfreq >> 1);
  if (frequency > maxfreq)
    {
      /* Set the frequency to the maximum */

      spidbg("ERROR: Cannot realize frequency: %ld\n", (long)frequency);
      frequency = maxfreq;
    }

  /* Check if the requested frequency is the same as the frequency selection */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }


#ifdef CONFIG_I2C_RESET
  priv->frequency = frequency;
#endif
  return actual;
}

/*******************************************************************************
 * Name: i2c_hw_initialize
 *
 * Description:
 *   Initialize/Re-initialize the I2C peripheral.  This logic performs only
 *   repeatable initialization after either (1) the one-time initialization, or
 *   (2) after each bus reset.
 *
 *******************************************************************************/

static void i2c_hw_initialize(struct sam_i2c_dev_s *priv, uint32_t frequency)
{
  irqstate_t flags;
  uint32_t regval;
  uint32_t mck;

  i2cvdbg("I2C%d Initializing\n", priv->attr->i2c);

  /* Enable clocking to the SERCOM module in PM */

  flags = irqsave();
  sercom_enable(priv->sercom);

  /* Configure the GCLKs for the SERCOM module */

  sercom_coreclk_configure(priv->sercom, priv->gclkgen, false);
  sercom_slowclk_configure(priv->sercom, priv->slowgen);

  /* Check if module is enabled */
  regval = i2c_getreg32(priv, SAM_I2C_CTRLA_OFFSET);
  if (regval & I2C_CTRLA_ENABLE)
    {
      i2cvdbg("ERROR: Cannot initialize I2C because it is already initialized!\n");
      return -EPERM;
    } 

  /* Check if reset is in progress */
  regval = i2c_getreg32(priv, SAM_I2C_CTRLA_OFFSET);
  if (regval & I2C_CTRLA_SWRST)
    {
      i2cvdbg("ERROR: Module is in RESET process!\n");
      return -EBUSY;
    }

  /* Set the SERCOM in I2C master mode */

  regval  = i2c_getreg32(priv, SAM_I2C_CTRLA_OFFSET);
  regval &= ~I2C_CTRLA_MODE_MASK;
  regval |= (I2C_CTRLA_MODE_MASTER);
  spi_putreg32(priv, regval, SAM_I2C_CTRLA_OFFSET);

  /* Configure pads */

  i2c_pad_configure(priv);

  /* Set an initial baud value. */

  (void)i2c_hw_setfrequency(priv, 100000);

  /* Enable I2C */

  regval  = i2c_getreg32(priv, SAM_I2C_CTRLA_OFFSET);
  regval |= I2C_CTRLA_ENABLE;
  i2c_putreg32(priv, regval, SAM_I2C_CTRLA_OFFSET);
  i2c_wait_synchronization(priv);

  /* Enable SERCOM interrupts at the NVIC */

  up_enable_irq(priv->attr->irq);
  irqrestore(flags);
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize a I2C device for I2C operation
 *
 *******************************************************************************/

struct i2c_dev_s *up_i2cinitialize(int bus)
{
  struct sam_i2c_dev_s *priv;
  uint32_t frequency;
  irqstate_t flags;
  int ret;

  i2cvdbg("Initializing I2C%d\n", bus);

#ifdef CONFIG_SAM_I2C0
  if (bus == 0)
    {
      /* Select up I2C0 and setup invariant attributes */

      priv       = &g_i2c0;
      priv->attr = &g_i2c0attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C0_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAM_I2C1
  if (bus == 1)
    {
      /* Select up I2C1 and setup invariant attributes */

      priv       = &g_i2c1;
      priv->attr = &g_i2c1attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C1_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAM_I2C2
  if (bus == 2)
    {
      /* Select up I2C2 and setup invariant attributes */

      priv       = &g_i2c2;
      priv->attr = &g_i2c2attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C2_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAM_I2C3
  if (bus == 3)
    {
      /* Select up I2C3 and setup invariant attributes */

      priv       = &g_i2c3;
      priv->attr = &g_i2c3attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C3_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAM_I2C4
  if (bus == 4)
    {
      /* Select up I2C4 and setup invariant attributes */

      priv       = &g_i2c4;
      priv->attr = &g_i2c4attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C4_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAM_I2C5
  if (bus == 5)
    {
      /* Select up I2C5 and setup invariant attributes */

      priv       = &g_i2c5;
      priv->attr = &g_i2c5attr;

      /* Select the (initial) I2C frequency */

      frequency = CONFIG_SAM_I2C5_FREQUENCY;
    }
  else
#endif
    {
      i2cdbg("ERROR: Unsupported bus: I2C%d\n", bus);
      return NULL;
    }

  /* Perform one-time I2C initialization */

  flags = irqsave();

  /* Allocate a watchdog timer */

  priv->timeout = wd_create();
  if (priv->timeout == NULL)
    {
      idbg("ERROR: Failed to allocate a timer\n");
      goto errout_with_irq;
    }

  /* Attach Interrupt Handler */

  ret = irq_attach(priv->attr->irq, priv->attr->handler);
  if (ret < 0)
    {
      idbg("ERROR: Failed to attach irq %d\n", priv->attr->irq);
      goto errout_with_wdog;
    }

  /* Initialize the I2C driver structure */

  priv->dev.ops = &g_i2cops;
  priv->address = 0;
  priv->flags   = 0;

  (void)sem_init(&priv->exclsem, 0, 1);
  (void)sem_init(&priv->waitsem, 0, 0);

  /* Perform repeatable I2C hardware initialization */

  i2c_hw_initialize(priv, frequency);
  irqrestore(flags);
  return &priv->dev;

errout_with_wdog:
  wd_delete(priv->timeout);
  priv->timeout = NULL;

errout_with_irq:
  irqrestore(flags);
  return NULL;
}

/*******************************************************************************
 * Name: up_i2cuninitalize
 *
 * Description:
 *   Uninitialize an I2C device
 *
 *******************************************************************************/

int up_i2cuninitialize(FAR struct i2c_dev_s *dev)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *) dev;

  i2cvdbg("I2C%d Un-initializing\n", priv->attr->i2c);

  /* Disable I2C interrupts */

  up_disable_irq(priv->attr->irq);

  /* Reset data structures */

  sem_destroy(&priv->exclsem);
  sem_destroy(&priv->waitsem);

  /* Free the watchdog timer */

  wd_delete(priv->timeout);
  priv->timeout = NULL;

  /* Detach Interrupt Handler */

  (void)irq_detach(priv->attr->irq);
  return OK;
}

/************************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
int up_i2creset(FAR struct i2c_dev_s *dev)
{
  struct i2c_dev_s *priv = (struct i2c_dev_s *)dev;
  unsigned int clockcnt;
  unsigned int stretchcnt;
  uint32_t sclpin;
  uint32_t sdapin;
  int ret;

  ASSERT(priv);

  /* Get exclusive access to the I2C device */

  i2c_takesem(&priv->exclsem);

  /* Disable I2C interrupts */

  up_disable_irq(priv->attr->irq);

  /* Use PIO configuration to un-wedge the bus.
   *
   * Reconfigure both pins as open drain outputs with initial output value
   * "high" (i.e., floating since these are open-drain outputs).
   */

  sclpin = MKI2C_OUTPUT(priv->attr->sclcfg);
  sdapin = MKI2C_OUTPUT(priv->attr->sdacfg);

  sam_configpio(sclpin);
  sam_configpio(sdapin);

  /* Peripheral clocking must be enabled in order to read valid data from
   * the output pin (clocking is enabled automatically for pins configured
   * as inputs).
   */

  sam_pio_forceclk(sclpin, true);
  sam_pio_forceclk(sdapin, true);

  /* Clock the bus until any slaves currently driving it low let it float.
   * Reading from the output will return the actual sensed level on the
   * SDA pin (not the level that we wrote).
   */

  clockcnt = 0;
  while (sam_pioread(sdapin) == false)
    {
      /* Give up if we have tried too hard */

      if (clockcnt++ > 10)
        {
          ret = -ETIMEDOUT;
          goto errout_with_lock;
        }

      /* Sniff to make sure that clock stretching has finished.  SCL should
       * be floating high here unless something is driving it low.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretchcnt = 0;
      while (sam_pioread(sclpin) == false)
        {
          /* Give up if we have tried too hard */

          if (stretchcnt++ > 10)
            {
              ret = -EAGAIN;
              goto errout_with_lock;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      sam_piowrite(sclpin, false);
      up_udelay(10);

      /* Drive SCL high (floating) again */

      sam_piowrite(sclpin, true);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  sam_piowrite(sdapin, false);
  up_udelay(10);
  sam_piowrite(sclpin, false);
  up_udelay(10);

  sam_piowrite(sclpin, true);
  up_udelay(10);
  sam_piowrite(sdapin, true);
  up_udelay(10);

  /* Clocking is no longer forced */

  sam_pio_forceclk(sclpin, false);
  sam_pio_forceclk(sdapin, false);

  /* Re-initialize the port hardware */

  i2c_hw_initialize(priv, priv->frequency);
  ret = OK;

errout_with_lock:

  /* Release our lock on the bus */

  i2c_givesem(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_I2C_RESET */
#endif /* CONFIG_SAM_I2C0 || ... || CONFIG_SAM_I2C3 */
