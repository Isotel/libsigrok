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

#include <config.h>
#include <stdint.h>
#include <string.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"


#define INS_CHS 14

typedef struct {
	uint8_t protocol;
	uint8_t message_num;
} __attribute__((packed)) isn_proto_t;

typedef struct {
	isn_proto_t header;
	uint8_t state;
	uint8_t actions;
	uint8_t locked;
	uint8_t name[8];
} __attribute__((packed)) ins_cpyst_t;

typedef struct {
	uint16_t select;
	uint16_t opts;
} __attribute__((packed)) ins_ch_sel_t;

typedef struct {
	isn_proto_t header;
	ins_ch_sel_t selections[INS_CHS];
} __attribute__((packed)) ins_function_t;

typedef struct {
	isn_proto_t header;
	uint32_t rates[INS_CHS];
} __attribute__((packed)) ins_rates_t;

typedef struct {
	isn_proto_t header;
	uint16_t smp_rate;        ///< sample rate of A/D in kHz/2
	uint32_t mux_N;            ///< number of mux channels
	uint8_t dec_N;                ///< decimation factor per channel
	uint8_t range;                ///< A/D voltage reference
	uint8_t a_samples;            ///< samples count in analog input (device -> host) packet
	uint8_t d_samples;            ///< samples count in digital input (device -> host) packet
	uint16_t d_a_ratio;            ///< number of digital samples per analog sample
	uint32_t pckt_cnt_ain;    ///< number of analog in packets sent
	uint32_t pckt_cnt_din;    ///< number of digital in packets sent
	uint32_t pckt_cnt_aout;    ///< number of analog out packets received
	uint32_t pckt_cnt_dout;    ///< number of digital out packets received
} __attribute__((packed)) daq_ad_t;

typedef struct {
	isn_proto_t header;
	uint32_t timed;     ///< start of timed trigger (doesn't exist in SCPI)
	uint32_t acq_N;     ///< number of samples to acquire, 0 means infinite
} __attribute__((packed)) daq_trigger_t;

typedef struct {
	uint8_t protocol_id;
	uint8_t port;
	uint16_t packet_counter; /**< sequntial packet counter */
	uint8_t short_protocol;      // frame_id = 0x80 + (len-1)
	uint8_t user_protocol_id;
	uint16_t alignment_counter; /**< plugin packet counter, resetted to 0 on each measurement start */
} __attribute__((packed)) isn_proto_long_t;


#define ISN_PROTO_HEADER(MESSAGE_NUM) .header = { 0x07f, (MESSAGE_NUM)}

#define ISN_CPYST_CLEARED       0
#define ISN_CPYST_UPDATED       1
#define ISN_CPYST_UNDONE        2
#define ISN_CYPST_STORED        3
#define ISN_CYPST_LOADED        4
#define ISN_CPYST_ERROR         5
#define ISN_CPYST_ERROR_HASH    6
#define ISN_CPYST_ERROR_IO      7

#define ISN_CPYST_CLEAR     8
#define ISN_CPYST_UNDO      16
#define ISN_CYPST_STORE     32
#define ISN_CYPST_LOAD      64

#define INS_CH_FUNC_NA              0
#define INS_CH_FUNC_POWER           0x0001
#define INS_CH_FUNC_CIN             0x0002
#define INS_CH_FUNC_VIN             0x0004
#define INS_CH_FUNC_STRAIN          0x0008
#define INS_CH_FUNC_TC              0x0010
#define INS_CH_FUNC_RTD             0x0020
#define INS_CH_FUNC_VOUT            0x0040
#define INS_CH_FUNC_DIN             0x0080
#define INS_CH_FUNC_DOUT            0x0100
#define INS_CH_FUNC_EXC             0x0200
#define INS_CH_FUNC_PWM             0x0400
#define INS_CH_FUNC_1WIRE           0x0800
#define INS_CH_FUNC_I2C             0x1000
#define INS_CH_FUNC_ENCODER         0x2000
#define INS_CH_FUNC_BIASED          0x4000
#define INS_CH_FUNC_SELECT          0x8000

#define MONDAQ_U_MSG_DAQ_AD         19
#define MONDAQ_U_MSG_DAQ_TRIGGER    23
#define MONDAQ_U_MSG_CONFIG         68
#define MONDAQ_U_MSG_FUNCTION       41
#define MONDAQ_U_MSG_RATE           57

#define MONDAQ_U_ADC_PROTOCOL_ID            0x07d
#define MONDAQ_U_ADC_CHANNEL_ID            1
#define MONDAQ_U_DIGITAL_CHANNEL_ID        2
#define MONDAQ_U_ANALOG_USER_PROTOCOL_ID    1
#define MONDAQ_U_DIGITAL_USER_PROTOCOL_ID    2


static int receive_message(const struct sr_dev_inst *sdi, uint8_t msg_num,
						   uint8_t *data, size_t len)
{
	struct dev_context *devc = sdi->priv;
	gint64 start_time, end_time;
	int ret;

	g_mutex_lock(&devc->message_mutex);
	start_time = g_get_monotonic_time();
	end_time = start_time + 500 * G_TIME_SPAN_MILLISECOND;
	while (devc->message_table[msg_num].time < start_time) {
		if (!g_cond_wait_until(&devc->message_table[msg_num].cond, &devc->message_mutex, end_time)) {
			ret = SR_ERR_TIMEOUT;
			goto exit;
		}
	}
	if (len != devc->message_table[msg_num].sz) {
		ret = SR_ERR_DATA;
		goto exit;
	}
	memcpy(data, devc->message_table[msg_num].data, len);
	ret = SR_OK;

	exit:
	g_mutex_unlock(&devc->message_mutex);
	return ret;
}

static int xfer_command(const struct sr_dev_inst *sdi,
						uint8_t *command, uint8_t in_len, uint8_t out_len)
{
	struct sr_usb_dev_inst *usb;
	int ret, xfer;

	usb = sdi->conn;

	ret = libusb_bulk_transfer(usb->devhdl, MONODAQ_U_COMMAND_EP_OUT,
							   command, in_len, &xfer, 1000);
	if (ret != 0) {
		sr_dbg("Failed to send command %3d: %s.",
			   command[1], libusb_error_name(ret));
		return SR_ERR;
	}
	if (xfer != in_len) {
		sr_dbg("Failed to send command %3d: incorrect length "
			   "%d != %d.", command[1], xfer, in_len);
		return SR_ERR;
	}

	ret = receive_message(sdi, command[1], command, out_len);

	if (ret != SR_OK) {
		sr_dbg("Failed to receive reply to command %3d: %d.",
			   command[1], ret);
		return SR_ERR;
	}

	return SR_OK;
}


static int do_command(const struct sr_dev_inst *sdi,
					  uint8_t *command, uint8_t cmd_len)
{
	return xfer_command(sdi, command, cmd_len, cmd_len);
}

static int do_request(const struct sr_dev_inst *sdi,
					  uint8_t *command, uint8_t cmd_len)
{
	return xfer_command(sdi, command, sizeof(isn_proto_t), cmd_len);
}


static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi = transfer->user_data;
	struct dev_context *devc = sdi->priv;
	uint8_t *data;
	uint8_t msg_num;
	isn_proto_long_t *lp;

	switch (transfer->status) {
		case LIBUSB_TRANSFER_COMPLETED:
			data = transfer->buffer;
			if (transfer->actual_length >= sizeof(isn_proto_t)) {
				if (((isn_proto_t *)data)->protocol == 0x7F) {
					msg_num = ((isn_proto_t *)data)->message_num;
					sr_dbg("receive_transfer: received message %d, sz %d", msg_num, transfer->actual_length);

					g_mutex_lock(&devc->message_mutex);
					devc->message_table[msg_num].sz = (size_t)transfer->actual_length;
					devc->message_table[msg_num].time = g_get_monotonic_time();
					memcpy(devc->message_table[msg_num].data,
						   data, (size_t)transfer->actual_length);

					g_cond_broadcast(&devc->message_table[msg_num].cond);
					g_mutex_unlock(&devc->message_mutex);
				} else if (devc->state == M_STATE_SAMPLING
						   && ((isn_proto_t *)data)->protocol == MONDAQ_U_ADC_PROTOCOL_ID
						   && transfer->actual_length >= sizeof(isn_proto_long_t)) {
					lp = (isn_proto_long_t *)data;
					if (lp->port == MONDAQ_U_DIGITAL_CHANNEL_ID) {
						g_mutex_lock(&devc->digital_mutex);

						size_t idx = devc->digital_bl_in_idx + 1;
						idx = (idx < MONODAQ_U_DIGITAL_BLOCKS) ? idx : idx - MONODAQ_U_DIGITAL_BLOCKS;

						if (idx == devc->digital_bl_out_idx) {
							sr_warn("receive_transfer: digital input queue full, packet dropped");
						} else {
/*
							sr_dbg("receive_transfer: saving digital input packet, idx %zd, sz: %d", idx,
								   (lp->short_protocol & 0x03fu) - sizeof(uint16_t));
*/
							memcpy(devc->digital_blocks[idx], &data[sizeof(isn_proto_long_t)],
								   (lp->short_protocol & 0x03fu) - sizeof(uint16_t));
							devc->digital_bl_in_idx = idx;
						}

						g_mutex_unlock(&devc->digital_mutex);
					}
				}
			}

		case LIBUSB_TRANSFER_TIMED_OUT:
			libusb_submit_transfer(transfer);
			break;
		default:
			break;
	}
}

int monodaq_u_init_device(const struct sr_dev_inst *sdi)
{
	int ret;
	size_t sz;

	ins_cpyst_t reset_command = {
			ISN_PROTO_HEADER(MONDAQ_U_MSG_CONFIG),
			.state = ISN_CPYST_CLEAR,
			.actions = 0,
			.locked = 0,
			.name = {0}
	};
	ins_function_t function_command = {
			ISN_PROTO_HEADER(MONDAQ_U_MSG_FUNCTION),
	};

	struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;

	devc->state = M_STATE_STOPPED;

	devc->input_eps[0] = MONODAQ_U_COMMAND_EP_IN;
	devc->input_eps[1] = MONODAQ_U_DATA_EP_IN;

	g_mutex_init(&devc->message_mutex);
	g_mutex_init(&devc->digital_mutex);

	for (sz = 0; sz < ARRAY_SIZE(devc->message_table); ++sz) {
		g_cond_init(&devc->message_table[sz].cond);
		devc->message_table[sz].time = 0;
	}
	for (sz = 0; sz < ARRAY_SIZE(devc->input_transfers); ++sz) {
		devc->input_transfers[sz] = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(devc->input_transfers[sz], usb->devhdl,
								  devc->input_eps[sz], (unsigned char *)&devc->input_buffers[sz],
								  MONODAQU_MAX_MESSAGE_SIZE,
								  receive_transfer, (void *)sdi, 0);
		ret = libusb_submit_transfer(devc->input_transfers[sz]);
	}

	if ((ret = do_command(sdi, (uint8_t *)&reset_command, sizeof(reset_command))) != SR_OK) {
		return ret;
	}
	if ((ret = do_request(sdi, (uint8_t *)&function_command, sizeof(function_command))) != SR_OK) {
		return ret;
	}
	for (sz = 0; sz < 8; ++sz) {
		function_command.selections[sz].select = INS_CH_FUNC_DIN;
	}
	if ((ret = do_command(sdi, (uint8_t *)&function_command, sizeof(function_command))) != SR_OK) {
		return ret;
	}

	return ret;
}

int monodaq_u_send_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	uint8_t *p_data;
	size_t idx, digital_samples_per_block;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;
	digital_samples_per_block = devc->digital_samples_per_block;

	packet.type = SR_DF_LOGIC;
	packet.payload = &logic;
	logic.data = p_data = devc->digital_buffer;
	logic.length = 0;
	logic.unitsize = 1;

	g_mutex_lock(&devc->digital_mutex);
	for (idx = devc->digital_bl_out_idx; idx != devc->digital_bl_in_idx;) {
		if(MONODAQ_U_DIGITAL_BUFFER_SIZE - logic.length < digital_samples_per_block) {
			sr_session_send(sdi, &packet);
			logic.length = 0;
			p_data = devc->digital_buffer;

		} else {
			memcpy(p_data, devc->digital_blocks[idx], digital_samples_per_block);
			p_data += digital_samples_per_block;
			logic.length += digital_samples_per_block;
		}
		++idx;
		idx = devc->digital_bl_out_idx = idx < MONODAQ_U_DIGITAL_BLOCKS ? idx : idx - MONODAQ_U_DIGITAL_BLOCKS;
	}

	g_mutex_unlock(&devc->digital_mutex);

	if(logic.length) {
		sr_session_send(sdi, &packet);
	}

	return G_SOURCE_CONTINUE;
}

int monodaq_u_start_acquire(const struct sr_dev_inst *sdi)
{
	ins_rates_t rates = {
			ISN_PROTO_HEADER(MONDAQ_U_MSG_RATE),
	};
	daq_ad_t daq_ad = {
			ISN_PROTO_HEADER(MONDAQ_U_MSG_DAQ_AD),
	};
	daq_trigger_t daq_trigger = {
			ISN_PROTO_HEADER(MONDAQ_U_MSG_DAQ_TRIGGER),
	};


	struct dev_context *devc;
	int ret;

	devc = sdi->priv;

	sr_dbg("monodaq_u_start_acquire: requesting sample rates");
	if ((ret = do_request(sdi, (uint8_t *)&rates, sizeof(rates))) != SR_OK) {
		return ret;
	}
	sr_dbg("monodaq_u_start_acquire: setting sample rate to %ldHz", devc->sample_rate);
	rates.rates[0] = (uint32_t)devc->sample_rate;
	sr_dbg("monodaq_u_start_acquire: sending sample rates");
	if ((ret = do_command(sdi, (uint8_t *)&rates, sizeof(rates))) != SR_OK) {
		return ret;
	}

	sr_dbg("monodaq_u_start_acquire: requesting daq params");
	if ((ret = do_request(sdi, (uint8_t *)&daq_ad, sizeof(daq_ad))) != SR_OK) {
		return ret;
	}
	devc->digital_samples_per_block = daq_ad.d_samples;
	sr_dbg("monodaq_u_start_acquire: digital %d samples per block", devc->digital_samples_per_block);

	sr_dbg("monodaq_u_start_acquire: starting acquire");
	devc->digital_bl_in_idx = devc->digital_bl_out_idx = 0;
	devc->state = M_STATE_SAMPLING;
	daq_trigger.acq_N = 0;
	daq_trigger.timed = 1;
	if ((ret = do_command(sdi, (uint8_t *)&daq_trigger, sizeof(daq_trigger))) != SR_OK) {
		return ret;
	}

	return SR_OK;
}

int monodaq_u_stop_acquire(const struct sr_dev_inst *sdi)
{
	daq_trigger_t daq_trigger = {
			ISN_PROTO_HEADER(MONDAQ_U_MSG_DAQ_TRIGGER),
	};
	struct dev_context *devc;
	int ret;

	devc = sdi->priv;

	sr_dbg("monodaq_u_start_acquire: stopping acquire");
	devc->state = M_STATE_STOPPED;
	daq_trigger.acq_N = 0;
	daq_trigger.timed = 0;
	if ((ret = do_command(sdi, (uint8_t *)&daq_trigger, sizeof(daq_trigger))) != SR_OK) {
		return ret;
	}

	return SR_OK;
}
