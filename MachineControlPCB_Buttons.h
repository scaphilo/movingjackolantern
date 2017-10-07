/*-----------------------------------------------------------------------------
 * Name:    Board_Buttons.h
 * Purpose: Buttons interface header file
 * Rev.:    1.0.0
 *-----------------------------------------------------------------------------*/

/* Copyright (c) 2013 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#ifndef __BOARD_Buttons_H
#define __BOARD_Buttons_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
  \fn          int32_t Buttons_Initialize (void)
  \brief       Initialize I/O interface for Buttonss
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/
/**
  \fn          int32_t Buttons_Uninitialize (void)
  \brief       De-initialize I/O interface for Buttonss
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/
/**
  \fn          int32_t Buttons_On (uint32_t num)
  \brief       Turn on a single Buttons indicated by \em num
  \param[in]   num  Buttons number
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/
/**
  \fn          int32_t Buttons_Off (uint32_t num)
  \brief       Turn off a single Buttons indicated by \em num
  \param[in]   num  Buttons number
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/
/**
  \fn          int32_t Buttons_SetOut (uint32_t val)
  \brief       Control all Buttonss with the bit vector \em val
  \param[in]   val  each bit represents the status of one Buttons.
  \returns
   - \b  0: function succeeded
   - \b -1: function faiButtons
*/
/**
  \fn          uint32_t Buttons_GetCount (void)
  \brief       Get number of available Buttonss on evaluation hardware
  \return      Number of available Buttonss
*/

extern int32_t  Buttons_Initialize   (void);
extern int32_t  Buttons_Uninitialize (void);
extern GPIO_PinState  Buttons_Status       (uint32_t num);


#endif /* __BOARD_Buttons_H */
