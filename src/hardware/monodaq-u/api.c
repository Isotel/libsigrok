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
#include <glib.h>
#include <libusb.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

#define MONODAQ_U_VID 0x5726
#define MONODAQ_U_PID 0x1502

#define USB_INTERFACE		0
#define USB_CONFIGURATION	1

static const uint32_t scanopts[] = {
		SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
		SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
		SR_CONF_CONTINUOUS,
		SR_CONF_LIMIT_SAMPLES | SR_CONF_SET,
		SR_CONF_CONN | SR_CONF_GET,
		SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static const char *channel_names[] = {
		"DI1", "DI2", "DI3", "DI4", "DI5", "DI6", "DI7",
};


static const uint64_t samplerates[] = {
/*
		SR_HZ(50),
		SR_HZ(100),
		SR_HZ(200),
		SR_HZ(500),
		SR_KHZ(1),
		SR_KHZ(2),
		SR_KHZ(5),
		SR_KHZ(10),
		SR_KHZ(20),
		SR_KHZ(50),
*/
		SR_KHZ(100),
};


static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct sr_config *src;
	GSList *l, *devices, *conn_devices;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	unsigned int i;
	const char *conn;
	char connection_id[64];

	devices = NULL;
	drvc = di->context;
	drvc->instances = NULL;

	conn = NULL;
	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
			case SR_CONF_CONN:
				conn = g_variant_get_string(src->data, NULL);
				break;
			default:
				break;
		}
	}
	if (conn)
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	else
		conn_devices = NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	for (i = 0; devlist[i]; i++) {
		if (conn) {
			for (l = conn_devices; l; l = l->next) {
				usb = l->data;
				if (usb->bus == libusb_get_bus_number(devlist[i])
					&& usb->address == libusb_get_device_address(devlist[i]))
					break;
			}
			if (!l)
				/* This device matched none of the ones that
				 * matched the conn specification. */
				continue;
		}

		libusb_get_device_descriptor(devlist[i], &des);

		if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
			continue;

		if (des.idVendor != MONODAQ_U_VID || des.idProduct != MONODAQ_U_PID)
			continue;

		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->vendor = g_strdup("ISOTEL");
		sdi->model = g_strdup("MonoDAQ-U");
		sdi->connection_id = g_strdup(connection_id);

		//TODO fill channels
		for (i = 0; i < ARRAY_SIZE(channel_names); i++) {
			sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE,
						   channel_names[i]);
		}

		devc = g_malloc0(sizeof(struct dev_context));
		sdi->priv = devc;
		devices = g_slist_append(devices, sdi);

		sr_dbg("Found a MonoDAQ_U device.");
		sdi->status = SR_ST_INACTIVE;
		sdi->inst_type = SR_INST_USB;
		sdi->conn = sr_usb_dev_inst_new(
				libusb_get_bus_number(devlist[i]),
				libusb_get_device_address(devlist[i]), NULL);
		sr_dbg("Found a MonoDAQ_U device, bus number %02X, address %02X",
				libusb_get_bus_number(devlist[i]), libusb_get_device_address(devlist[i]));
	}

	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;
	libusb_device **devlist;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct monodaq_u_drv_context* m_drvc;
	int ret = SR_ERR, i, device_count;
	char connection_id[64];

	di = sdi->driver;
	m_drvc = di->context;
	usb = sdi->conn;

	device_count = (int) libusb_get_device_list(m_drvc->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
			   libusb_error_name(device_count));
		return SR_ERR;
	}

	for (i = 0; i < device_count; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != MONODAQ_U_VID || des.idProduct != MONODAQ_U_PID)
			continue;

		if ((sdi->status == SR_ST_INITIALIZING) ||
			(sdi->status == SR_ST_INACTIVE)) {
			/*
			 * Check device by its physical USB bus/port address.
			 */
			if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
				continue;

			if (strcmp(sdi->connection_id, connection_id))
				/* This is not the one. */
				continue;
		}

		if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
			usb->address = libusb_get_device_address(devlist[i]);
		} else {
			sr_err("Failed to open device: %s.",
				   libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE);
		if (ret == LIBUSB_ERROR_BUSY) {
			sr_err("Unable to claim USB interface. Another "
				   "program or driver has already claimed it.");
			ret = SR_ERR;
			break;
		} else if (ret == LIBUSB_ERROR_NO_DEVICE) {
			sr_err("Device has been disconnected.");
			ret = SR_ERR;
			break;
		} else if (ret != 0) {
			sr_err("Unable to claim interface: %s.",
				   libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		if ((ret = monodaq_u_init_device(sdi)) != SR_OK) {
			sr_err("Failed to init device.");
			break;
		}

		sr_info("Opened device on %d.%d (logical) / %s (physical), interface %d.",
				usb->bus, usb->address, sdi->connection_id, USB_INTERFACE);

		ret = SR_OK;

		break;
	}

	libusb_free_device_list(devlist, 1);

	if (ret != SR_OK) {
		if (usb->devhdl) {
			libusb_release_interface(usb->devhdl, USB_INTERFACE);
			libusb_close(usb->devhdl);
			usb->devhdl = NULL;
		}
		return SR_ERR;
	}

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;

	usb = sdi->conn;

	if (!usb->devhdl)
		return SR_ERR_BUG;

	sr_info("Closing device on %d.%d (logical) / %s (physical) interface %d.",
			usb->bus, usb->address, sdi->connection_id, USB_INTERFACE);
	libusb_release_interface(usb->devhdl, USB_INTERFACE);
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;

	return SR_OK;
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	(void)cg;

	switch (key) {
		case SR_CONF_CONN:
			if (!sdi || !sdi->conn)
				return SR_ERR_ARG;
			usb = sdi->conn;
			*data = g_variant_new_printf("%d.%d", usb->bus, usb->address);
			break;
		case SR_CONF_SAMPLERATE:
			if (!sdi)
				return SR_ERR;
			devc = sdi->priv;
			*data = g_variant_new_uint64(devc->sample_rate);
			break;
		default:
			return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;

	(void)cg;

	devc = sdi->priv;

	switch (key) {
		case SR_CONF_SAMPLERATE:
			devc->sample_rate = g_variant_get_uint64(data);
			break;
		case SR_CONF_LIMIT_SAMPLES:
			devc->limit_samples = g_variant_get_uint64(data);
			break;
		default:
			return SR_ERR_NA;
	}

	return SR_OK;}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	switch (key) {
		case SR_CONF_SCAN_OPTIONS:
		case SR_CONF_DEVICE_OPTIONS:
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
		case SR_CONF_SAMPLERATE:
			*data = std_gvar_samplerates(ARRAY_AND_SIZE(samplerates));
			break;
		default:
			return SR_ERR_NA;
	}

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	int ret;

	ret = monodaq_u_start_acquire(sdi);
	if(ret != SR_OK) {
		return ret;
	}

	sr_session_source_add(sdi->session, -1, 0, 30,
						  monodaq_u_send_data, (struct sr_dev_inst *)sdi);
	std_session_send_df_header(sdi);
	std_session_send_frame_begin(sdi);


	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	int ret;

	ret = monodaq_u_stop_acquire(sdi);
	if(ret != SR_OK) {
		return ret;
	}

	sr_session_source_remove(sdi->session, -1);
	std_session_send_frame_end(sdi);
	std_session_send_df_end(sdi);

	return SR_OK;
}

/*
 *
 */

static gpointer usb_event_thread(gpointer data)
{
	struct monodaq_u_drv_context *m_drvc = data;

	while (m_drvc->libusb_event_thread_active) {
		struct timeval tv = { 0, 10000 };
		libusb_handle_events_timeout_completed(m_drvc->libusb_ctx, &tv, NULL);
	}
	return NULL;
}

static int monodaq_u_init(struct sr_dev_driver *di, struct sr_context *sr_ctx)
{
	struct monodaq_u_drv_context *m_drvc;
	struct drv_context *drvc;
	int ret;

	if (!di) {
		sr_err("%s: Invalid argument.", __func__);
		return SR_ERR_ARG;
	}

	m_drvc = g_malloc0(sizeof(struct monodaq_u_drv_context));
	drvc = &m_drvc->drvc;
	drvc->sr_ctx = sr_ctx;
	drvc->instances = NULL;
	di->context = m_drvc;

	sr_dbg("initializing libusb context");
	ret = libusb_init(&m_drvc->libusb_ctx);
	if (ret != LIBUSB_SUCCESS) {
		sr_err("Failed to init libusb context %s.", libusb_error_name(ret));
		goto err;
	}

	sr_dbg("starting libusb event thread");
	m_drvc->libusb_event_thread_active = 1;
	m_drvc->libusb_event_thread = g_thread_new(
			"monodaq_u libusb thread",
			usb_event_thread,
			m_drvc);
	sr_dbg("started libusb event thread");

	return SR_OK;

err:
	g_free(m_drvc);
	return SR_ERR;
}

static int monodaq_u_cleanup(const struct sr_dev_driver *di)
{
	int ret;
	struct monodaq_u_drv_context *m_drvc;

	if (!di) {
		sr_err("%s: Invalid argument.", __func__);
		return SR_ERR_ARG;
	}

	m_drvc = di->context;

	sr_dbg("stopping libusb event thread");
	m_drvc->libusb_event_thread_active = 0;
	g_thread_join(m_drvc->libusb_event_thread);
	sr_dbg("stopped libusb event thread");

	ret = sr_dev_clear(di);
	g_free(di->context);

	return ret;
}



SR_PRIV struct sr_dev_driver monodaq_u_driver_info = {
	.name = "monodaq-u",
	.longname = "MonoDAQ-U",
	.api_version = 1,
	.init = monodaq_u_init,
	.cleanup = monodaq_u_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};

SR_REGISTER_DEV_DRIVER(monodaq_u_driver_info);
