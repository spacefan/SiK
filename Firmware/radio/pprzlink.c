// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
// Copyright (c) 2017 Michal Podhradsky
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

///
/// @file	pprzlink.c
///
/// mavlink reporting code
///

#include <stdarg.h>
#include "radio.h"
#include "packet.h"
#include "timer.h"

extern __xdata uint8_t pbuf[MAX_PACKET_LENGTH];

// new RADIO_STATUS common message
#define PPRZ_MSG_ID_RSSI 28
#define MAVLINK_RADIO_STATUS_CRC_EXTRA 185

// use '3D' for 3DRadio
#define RADIO_SOURCE_SYSTEM '3'
#define RADIO_SOURCE_COMPONENT 'D'

/*
 * Calculates the MAVLink checksum on a packet in pbuf[] 
 * and append it after the data
 */
static void mavlink_crc(register uint8_t crc_extra)
{
	register uint8_t length = pbuf[1];
	__xdata uint16_t sum = 0xFFFF;
	__xdata uint8_t i, stoplen;

	stoplen = length + 6;

	// MAVLink 1.0 has an extra CRC seed
	pbuf[length+6] = crc_extra;
	stoplen++;

	i = 1;
	while (i<stoplen) {
		register uint8_t tmp;
		tmp = pbuf[i] ^ (uint8_t)(sum&0xff);
		tmp ^= (tmp<<4);
		sum = (sum>>8) ^ (tmp<<8) ^ (tmp<<3) ^ (tmp>>4);
		i++;
	}

	pbuf[length+6] = sum&0xFF;
	pbuf[length+7] = sum>>8;
}


/*
we use a hand-crafted PPRZLINK packet based on the following
message definition

  <message name="RSSI" id="28">
    <field name="rssi" type="uint8" unit="dB"/>
    <field name="tx_power" type="uint8" unit="dB"/>
  </message>
*/
struct pprzlink_rssi {
	uint8_t rssi;
	uint8_t remrssi;
};

/// send a pprzlink status report packet
/// TODO: what about the AC_ID? That should be picked from the passing messages.
void MAVLink_report(void)
{
	struct mavlink_RADIO_v10 *m = (struct mavlink_RADIO_v10 *)&pbuf[6];
	pbuf[0] = MAVLINK10_STX;
	pbuf[1] = sizeof(struct mavlink_RADIO_v10);
	pbuf[2] = seqnum++;
	pbuf[3] = RADIO_SOURCE_SYSTEM;
	pbuf[4] = RADIO_SOURCE_COMPONENT;
	pbuf[5] = MAVLINK_MSG_ID_RADIO_STATUS;

    m->rssi     = statistics.average_rssi;
    m->remrssi  = remote_statistics.average_rssi;
	mavlink_crc(MAVLINK_RADIO_STATUS_CRC_EXTRA);

	if (serial_write_space() < sizeof(struct mavlink_RADIO_v10)+8) {
		// don't cause an overflow
		return;
	}

	serial_write_buf(pbuf, sizeof(struct mavlink_RADIO_v10)+8);
}
