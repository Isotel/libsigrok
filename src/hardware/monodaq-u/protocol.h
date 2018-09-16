/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2018 staslock <staslock@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_MONODAQ_U_PROTOCOL_H
#define LIBSIGROK_HARDWARE_MONODAQ_U_PROTOCOL_H

#include <config.h>
#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "monodaq-u"

#define MONODAQU_MAX_MESSAGE_ID 	127
#define MONODAQU_MAX_MESSAGE_SIZE 	64

#define MONODAQ_U_COMMAND_EP_OUT 	0x01
#define MONODAQ_U_COMMAND_EP_IN  	(0x080 | 0x02)
#define MONODAQ_U_DATA_EP_IN 	 	(0x080 | 0x03)
#define MONODAQ_U_INPUT_EP_COUNT 	2

#define MONODAQ_U_DIGITAL_BUFFER_SIZE	8192
#define MONODAQ_U_DIGITAL_PACKET_SIZE	48
#define MONODAQ_U_DIGITAL_BLOCKS		100000

typedef struct {
	GCond cond;
	gint64 time;
	size_t sz;
	uint8_t data[MONODAQU_MAX_MESSAGE_SIZE];
} msg_t;

struct monodaq_u_drv_context {
	struct drv_context drvc;
	/**/
	libusb_context* libusb_ctx;
	GThread* libusb_event_thread;
	volatile uint8_t libusb_event_thread_active;
};

typedef enum {
	M_STATE_STOPPED,
	M_STATE_SAMPLING,
} dev_state_t;

struct dev_context {
	dev_state_t state;

	/** The currently configured samplerate of the device. */
	uint64_t sample_rate;

	/** Maximum number of samples to capture, if nonzero. */
	uint64_t limit_samples;

	GMutex message_mutex;
	msg_t message_table[MONODAQU_MAX_MESSAGE_ID + 1];

	uint8_t input_eps[MONODAQ_U_INPUT_EP_COUNT];
	struct libusb_transfer* input_transfers [MONODAQ_U_INPUT_EP_COUNT];
	uint8_t input_buffers[MONODAQ_U_INPUT_EP_COUNT][MONODAQU_MAX_MESSAGE_SIZE];

	uint8_t digital_buffer[MONODAQ_U_DIGITAL_BUFFER_SIZE];

	GMutex digital_mutex;
	size_t digital_samples_per_block;
	uint8_t digital_blocks[MONODAQ_U_DIGITAL_BLOCKS][MONODAQ_U_DIGITAL_PACKET_SIZE];
	volatile size_t digital_bl_in_idx;
	volatile size_t digital_bl_out_idx;
};

SR_PRIV int monodaq_u_init_device(const struct sr_dev_inst *sdi);
SR_PRIV int monodaq_u_start_acquire(const struct sr_dev_inst *sdi);
SR_PRIV int monodaq_u_stop_acquire(const struct sr_dev_inst *sdi);
SR_PRIV int monodaq_u_send_data(int fd, int revents, void *cb_data);
#endif
