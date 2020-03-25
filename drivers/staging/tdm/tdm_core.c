/**********************************************************************
 * Author: Michail Kurachkin
 *
 * Contact: michail.kurachkin@promwad.com
 *
 * Copyright (c) 2013 Promwad Inc.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 *
 * This file is distributed in the hope that it will be useful, but
 * AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 * NONINFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * or visit http://www.gnu.org/licenses/.
**********************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include "tdm.h"

#include <linux/cache.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>

static void tdmdev_release(struct device *dev)
{
        struct tdm_device *tdm_dev = to_tdm_device(dev);

        /* tdm controllers may cleanup for released devices */
        if (tdm_dev->controller->cleanup)
                tdm_dev->controller->cleanup(tdm_dev);

        put_device(&tdm_dev->controller->dev);
        kfree(tdm_dev);
}

static int tdm_match_device(struct device *dev, struct device_driver *drv)
{
        const struct tdm_device *tdm_dev = to_tdm_device(dev);

        return strcmp(tdm_dev->modalias, drv->name) == 0;
}


static int tdm_uevent(struct device *dev, struct kobj_uevent_env *env)
{
        const struct tdm_device *tdm_dev = to_tdm_device(dev);

        add_uevent_var(env, "MODALIAS=%s%s", "tdm:", tdm_dev->modalias);
        return 0;
}


static void tdm_release(struct device *dev)
{
        struct tdm_device *tdm_dev;

        tdm_dev = to_tdm_device(dev);

        kfree(tdm_dev);
}


static struct class tdm_class = {
                .name           = "tdm",
                .owner          = THIS_MODULE,
                .dev_release    = tdm_release,
        };



/**
 * Struct for TDM bus
 */
struct bus_type tdm_bus_type = {
        .name           = "tdm",
        .dev_attrs      = NULL,
        .match          = tdm_match_device,
        .uevent         = tdm_uevent,
        .suspend        = NULL,
        .resume         = NULL,
};
EXPORT_SYMBOL_GPL(tdm_bus_type);



static int tdm_drv_probe(struct device *dev)
{
        const struct tdm_driver *tdm_drv = to_tdm_driver(dev->driver);

        return tdm_drv->probe(to_tdm_device(dev));
}

static int tdm_drv_remove(struct device *dev)
{
        const struct tdm_driver *tdm_drv = to_tdm_driver(dev->driver);

        return tdm_drv->remove(to_tdm_device(dev));
}

static void tdm_drv_shutdown(struct device *dev)
{
        const struct tdm_driver *tdm_drv = to_tdm_driver(dev->driver);

        tdm_drv->shutdown(to_tdm_device(dev));
}


/**
 * tdm_register_driver - register a TDM driver
 * @sdrv: the driver to register
 * Context: can sleep
 */
int tdm_register_driver(struct tdm_driver *tdm_drv)
{
        tdm_drv->driver.bus = &tdm_bus_type;

        if (tdm_drv->probe)
                tdm_drv->driver.probe = tdm_drv_probe;

        if (tdm_drv->remove)
                tdm_drv->driver.remove = tdm_drv_remove;

        if (tdm_drv->shutdown)
                tdm_drv->driver.shutdown = tdm_drv_shutdown;

        return driver_register(&tdm_drv->driver);
}
EXPORT_SYMBOL_GPL(tdm_register_driver);


/**
 * tdm_unregister_driver - reverse effect of tdm_register_driver
 * @sdrv: the driver to unregister
 * Context: can sleep
 */
void tdm_unregister_driver(struct tdm_driver *tdm_dev)
{
        if (tdm_dev)
                driver_unregister(&tdm_dev->driver);
}
EXPORT_SYMBOL_GPL(tdm_unregister_driver);



/**
 * Request and init unused voice channel
 * @param tdm_dev - TDM device requested voice channel
 * @return pointer to voice channel
 */
struct tdm_voice_channel *request_voice_channel(struct tdm_device *tdm_dev) {
	struct tdm_controller *tdm = tdm_dev->controller;
	struct tdm_voice_channel *ch;
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&tdm->lock, flags);

	list_for_each_entry(ch, &tdm->voice_channels, list)
                if (ch->dev == NULL) {
                        ch->dev = &tdm_dev->dev;
                        ch->mode_wideband = tdm_dev->mode_wideband;
                        ch->tdm_channel = tdm_dev->tdm_channel_num;
                        tdm_dev->ch = ch;
                        spin_unlock_irqrestore(&tdm->lock, flags);
                        rc = tdm->init_voice_channel(tdm_dev, ch);
                        if (rc)
                        {
                            dev_err(&tdm_dev->dev, "Can't init TDM voice channel\n");
                        	return NULL;
                        }

                        return ch;
                }

	spin_unlock_irqrestore(&tdm->lock, flags);
	return NULL;
}

/**
 * Release requested early voice channel
 * @param TDM device requested early voice channel
 * @return 0 - OK
 */
int release_voice_channel(struct tdm_device *tdm_dev)
{
	struct tdm_controller *tdm = tdm_dev->controller;
	struct tdm_voice_channel *ch;
	unsigned long flags;

	spin_lock_irqsave(&tdm->lock, flags);

	tdm_dev->ch = NULL;
	list_for_each_entry(ch, &tdm->voice_channels, list)
			if (ch->dev == &tdm_dev->dev) {
					tdm->release_voice_channel(tdm_dev);
					ch->dev = NULL;
					spin_unlock_irqrestore(&tdm->lock, flags);
					return 0;
			}

	spin_unlock_irqrestore(&tdm->lock, flags);

	return -ENODEV;
}



/*
 * Container for union board info of all TDM devices
 */
struct tdm_board_devices {
        struct list_head        list;
        struct tdm_board_info   bi; /* Board specific info */
};

/* List of all board specific TDM devices */
static LIST_HEAD(tdm_devices_list);

/* List of all registred TDM controllers */
static LIST_HEAD(tdm_controller_list);


/*
 * Used to protect add/del opertion for board_info list and
 * tdm_controller list, and their matching process
 */
static DEFINE_MUTEX(board_lock);


static void tdm_match_controller_to_boardinfo(struct tdm_controller *tdm,
        struct tdm_board_info *bi)
{
        struct tdm_device *tdm_dev;

        if (tdm->bus_num != bi->bus_num)
                return;

        tdm_dev = tdm_new_device(tdm, bi);
        if (!tdm_dev)
                dev_err(tdm->dev.parent, "can't create new device for %s\n",
                        bi->modalias);
}



/**
 * Allocate memory for TDM controller device
 * @dev: the controller, possibly using the platform_bus
 * @size: how much zeroed driver-private data to allocate; the pointer to this
 *      memory is in the driver_data field of the returned device,
 *      accessible with tdm_controller_get_devdata().
 * Context: can sleep
 */
struct tdm_controller *tdm_alloc_controller(struct device *dev, unsigned size) {
        struct tdm_controller   *tdm;

        if (!dev)
                return NULL;

        tdm = kzalloc(sizeof *tdm + size, GFP_KERNEL);
        if (!tdm)
                return NULL;

        device_initialize(&tdm->dev);
        tdm->dev.class = &tdm_class;
        tdm->dev.parent = get_device(dev);
        spin_lock_init(&tdm->lock);
        INIT_LIST_HEAD(&tdm->list);
        INIT_LIST_HEAD(&tdm->voice_channels);

        dev_set_drvdata(&tdm->dev, tdm + 1);

        return tdm;
}
EXPORT_SYMBOL_GPL(tdm_alloc_controller);


/**
 * Free memory for TDM controller device, allocated by tdm_alloc_controller
 * @dev: the controller, possibly using the platform_bus
 * Context: can sleep
 */
int tdm_free_controller(struct tdm_controller *tdm)
{
        if (!tdm)
                return -EINVAL;

        dev_set_drvdata(&tdm->dev, NULL);
        put_device(&tdm->dev);
        kzfree(tdm);

        return 0;
}
EXPORT_SYMBOL_GPL(tdm_free_controller);


/**
 * Allocate memory for voice channel
 * @return object voice_channel or NULL if error
 */
struct tdm_voice_channel *tdm_alloc_voice_channel(void)
{
        struct tdm_voice_channel *ch;

        ch = kzalloc(sizeof *ch, GFP_KERNEL);
        if (!ch)
                return NULL;

        init_waitqueue_head(&ch->tx_queue);
        init_waitqueue_head(&ch->rx_queue);

        return ch;
}
EXPORT_SYMBOL_GPL(tdm_alloc_voice_channel);



/**
 * Add allocated voice channel in tdm controller
 * @param tdm - tdm controller
 * @param ch - allocated voice channel
 * @param driver_private - private driver structure
 * @return 0 - ok
 */
int
tdm_register_new_voice_channel(struct tdm_controller *tdm,
    struct tdm_voice_channel *ch,
    void *driver_private)
{
	struct tdm_voice_channel *c;
	int last_num;

	last_num = -1;
	list_for_each_entry(c, &tdm->voice_channels, list)
		if (c->channel_num > last_num)
			last_num = c->channel_num;

	ch->channel_num = last_num + 1;
	ch->private_data = driver_private;
	list_add_tail(&ch->list, &tdm->voice_channels);

	return 0;
}
EXPORT_SYMBOL_GPL(tdm_register_new_voice_channel);

/**
 * free memory for voice channels allocated by tdm_alloc_voice_channels
 * @param tdm - TDM controller
 * @param count_channels - count supported voice channels
 * @return
 */
int tdm_free_voice_channels(struct tdm_controller *tdm)
{
	struct tdm_voice_channel *ch, *ch_tmp;
        if (!tdm)
        	return -EINVAL;

	list_for_each_entry_safe(ch, ch_tmp, &tdm->voice_channels, list)
        {
        	list_del(&ch->list);
        	kzfree(ch);
        }

        return 0;
}
EXPORT_SYMBOL_GPL(tdm_free_voice_channels);


/**
 * get voice_channel object by voice number
 * @param tdm - tdm_controller contained voice channels
 * @param num - voice channel number
 * @return voice_channel object or NULL
 */
struct tdm_voice_channel *
get_voice_channel_by_num(struct tdm_controller *tdm, int num)
{
	struct tdm_voice_channel *ch;
	list_for_each_entry(ch, &tdm->voice_channels, list)
		if (ch->channel_num == num)
			return ch;

	return NULL;
}
EXPORT_SYMBOL_GPL(get_voice_channel_by_num);


/**
 * tdm_controller_register - register TDM controller
 * @master: initialized master, originally from spi_alloc_master()
 * Context: can sleep
 *
 * This must be called from context that can sleep.  It returns zero on
 * success, else a negative error code (dropping the master's refcount).
 * After a successful return, the caller is responsible for calling
 * tdm_controller_unregister().
 */
int tdm_controller_register(struct tdm_controller *tdm)
{
        static atomic_t         dyn_bus_id = ATOMIC_INIT((1<<15) - 1);
        struct device           *dev = tdm->dev.parent;
        struct tdm_board_devices        *bi;
        int                     status = -ENODEV;
        int                     dynamic = 0;

        if (!dev) {
                dev_err(dev, "parent device not exist\n");
                return -ENODEV;
        }

        /* convention:  dynamically assigned bus IDs count down from the max */
        if (tdm->bus_num < 0) {
                tdm->bus_num = atomic_dec_return(&dyn_bus_id);
                dynamic = 1;
        }

        /* register the device, then userspace will see it.
         * registration fails if the bus ID is in use.
         */
        dev_set_name(&tdm->dev, "tdm%u", tdm->bus_num);
        status = device_add(&tdm->dev);
        if (status < 0) {
                dev_err(dev, "cannot added controller device, %d\n", status);
                goto done;
        }
        dev_dbg(dev, "registered controller %s%s\n", dev_name(&tdm->dev),
                dynamic ? " (dynamic)" : "");

        mutex_lock(&board_lock);
        list_add_tail(&tdm->list, &tdm_controller_list);
        list_for_each_entry(bi, &tdm_devices_list, list)
                tdm_match_controller_to_boardinfo(tdm, &bi->bi);

        mutex_unlock(&board_lock);

        status = 0;

done:
        return status;
}
EXPORT_SYMBOL_GPL(tdm_controller_register);


static int __unregister(struct device *dev, void *null)
{
        device_unregister(dev);
        return 0;
}


/**
 * tdm_controller_unregister - unregister TDM controller
 * @tdm: the controller being unregistered
 * Context: can sleep
 *
 * This call is used only by TDM controller drivers, which are the
 * only ones directly touching chip registers.
 *
 * This must be called from context that can sleep.
 */
void tdm_controller_unregister(struct tdm_controller *tdm)
{
        int dummy;

        mutex_lock(&board_lock);
        list_del(&tdm->list);
        mutex_unlock(&board_lock);

        dummy = device_for_each_child(&tdm->dev, NULL, __unregister);
        device_unregister(&tdm->dev);
}
EXPORT_SYMBOL_GPL(tdm_controller_unregister);


/**
 * tdm_new_device - instantiate one new TDM device
 * @tdm: TDM Controller to which device is connected
 * @chip: Describes the TDM device
 * Context: can sleep
 *
 * Returns the new device, or NULL.
 */
struct tdm_device *tdm_new_device(struct tdm_controller *tdm,
                                  struct tdm_board_info *chip) {
        struct tdm_device       *tdm_dev;
        int                     status;

        if (!tdm_controller_get(tdm))
                return NULL;

        tdm_dev = kzalloc(sizeof *tdm_dev, GFP_KERNEL);
        if (!tdm_dev) {
                dev_err(tdm->dev.parent, "cannot alloc for TDM device\n");
                tdm_controller_put(tdm);
                return NULL;
        }

        tdm_dev->controller = tdm;
        tdm_dev->dev.parent = tdm->dev.parent;
        tdm_dev->dev.bus = &tdm_bus_type;
        tdm_dev->dev.release = tdmdev_release;
        device_initialize(&tdm_dev->dev);

        WARN_ON(strlen(chip->modalias) >= sizeof(tdm_dev->modalias));

        tdm_dev->tdm_channel_num = chip->tdm_channel_num;
        tdm_dev->mode_wideband = chip->mode_wideband;
        tdm_dev->buffer_sample_count = chip->buffer_sample_count;
        strlcpy(tdm_dev->modalias, chip->modalias, sizeof(tdm_dev->modalias));

        status = tdm_add_device(tdm_dev);
        if (status < 0) {
                put_device(&tdm_dev->dev);
                return NULL;
        }

        return tdm_dev;
}
EXPORT_SYMBOL_GPL(tdm_new_device);



/**
 * tdm_add_device - Add tdm_device on TDM bus
 * @tdm_dev: TDM device to register
 *
 * Returns 0 on success; negative errno on failure
 */
int tdm_add_device(struct tdm_device *tdm_dev)
{
        static DEFINE_MUTEX(tdm_add_lock);
        struct tdm_controller *tdm = tdm_dev->controller;
        struct tdm_controller_hw_settings *hw = tdm->settings;
        struct device *dev = tdm->dev.parent;
        struct device *d;
        struct tdm_voice_channel *ch;
        int status;
        u8 count_tdm_channels;


        if (tdm_dev->mode_wideband) {
                dev_err(dev, "mode_wideband is not supported by this driver\n");
                status = -EINVAL;
                goto done;
        }

        count_tdm_channels = hw->count_time_slots / hw->channel_size;

        if (tdm_dev->tdm_channel_num >= count_tdm_channels) {
                dev_err(dev, "Incorrect requested TDM channel.\n"
                        "Requested %d TDM channel, %d TDM channels available.\n",
                        tdm_dev->tdm_channel_num, count_tdm_channels);

                status = -EINVAL;
                goto done;
        }

        if (tdm_dev->mode_wideband &&
            (tdm_dev->tdm_channel_num > count_tdm_channels / 2)) {
                dev_err(dev, "Incorrect requested TDM channel in wideband mode.\n"
                        "Requested %d TDM channel, %d TDM channels available\n"
                        "in wideband mode\n",
                        tdm_dev->tdm_channel_num, count_tdm_channels / 2);

                status = -EINVAL;
                goto done;
        }


        /* Set the bus ID string */
        dev_set_name(&tdm_dev->dev, "%s.%u", dev_name(&tdm_dev->controller->dev),
                     tdm_dev->tdm_channel_num);

        /* We need to make sure there's no other device with this
         * chipselect **BEFORE** we call setup(), else we'll trash
         * its configuration.  Lock against concurrent add() calls.
         */
        mutex_lock(&tdm_add_lock);

        d = bus_find_device_by_name(&tdm_bus_type, NULL, dev_name(&tdm_dev->dev));
        if (d != NULL) {
                dev_err(dev, "TDM channel %d already in use\n",
                        tdm_dev->tdm_channel_num);
                put_device(d);
                status = -EBUSY;
                goto done;
        }

        ch = request_voice_channel(tdm_dev);
        if (ch == NULL) {
                dev_err(dev, "Can't request TDM voice channel. All voice channels is busy\n");
                status = -EBUSY;
                goto done;
        }

        /* Device may be bound to an active driver when this returns */
        status = device_add(&tdm_dev->dev);
        if (status < 0)
                dev_err(dev, "can't add %s, status %d\n",
                        dev_name(&tdm_dev->dev), status);
        else
                dev_dbg(dev, "registered child %s\n", dev_name(&tdm_dev->dev));

done:
        mutex_unlock(&tdm_add_lock);
        return status;
}
EXPORT_SYMBOL_GPL(tdm_add_device);



/**
 * Receive audio-data from tdm device.
 * @param tdm_dev - tdm device registered on TDM bus
 * @param data - pointer to receive block data.
 *              Allocated data size must be equal value
 *              returned by tdm_get_voice_block_size();
 * @return 0 - success
 */
int tdm_recv(struct tdm_device *tdm_dev, u8 *data)
{
        struct tdm_controller *tdm = tdm_dev->controller;

        if (tdm_dev->ch == NULL)
                return -ENODEV;

        return tdm->recv(tdm_dev->ch, data);
}
EXPORT_SYMBOL_GPL(tdm_recv);


/**
 * Transmit audio-data from tdm device.
 * @param tdm_dev - tdm device registered on TDM bus
 * @param data - pointer to transmit block data.
 *              Transmit data size must be equal value
 *              returned by tdm_get_voice_block_size();
 * @return 0 - success
 */
int tdm_send(struct tdm_device *tdm_dev, u8 *data)
{
        struct tdm_controller *tdm = tdm_dev->controller;

        if (tdm_dev->ch == NULL)
                return -ENODEV;

        return tdm->send(tdm_dev->ch, data);
}
EXPORT_SYMBOL_GPL(tdm_send);


/**
 * Enable audio transport
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - success
 */
int tdm_run_audio(struct tdm_device *tdm_dev)
{
        struct tdm_controller *tdm = tdm_dev->controller;

        if (!tdm_dev->ch) {
                dev_err(&tdm_dev->dev, "Can't run audio because not allocated tdm voice channel\n");
                return -ENODEV;
        }

        return tdm->run_audio(tdm_dev);
}
EXPORT_SYMBOL_GPL(tdm_run_audio);


/**
 * Disable audio transport
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - success
 */
int tdm_stop_audio(struct tdm_device *tdm_dev)
{
        struct tdm_controller *tdm = tdm_dev->controller;

        if (!tdm_dev->ch) {
                dev_err(&tdm_dev->dev, "Can't stop audio because not allocated tdm voice channel\n");
                return -ENODEV;
        }

        return tdm->stop_audio(tdm_dev);
}
EXPORT_SYMBOL_GPL(tdm_stop_audio);



/**
 * Check rx audio buffer for exist new data
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - not enought data, 1 - data exist
 */
int tdm_poll_rx(struct tdm_device *tdm_dev)
{
        struct tdm_controller *tdm = tdm_dev->controller;

        return tdm->poll_rx(tdm_dev);
}
EXPORT_SYMBOL_GPL(tdm_poll_rx);


/**
 * Check tx audio buffer for free space
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - not enought free space, 1 - exist free space
 */
int tdm_poll_tx(struct tdm_device *tdm_dev)
{
        struct tdm_controller *tdm = tdm_dev->controller;

        return tdm->poll_tx(tdm_dev);
}
EXPORT_SYMBOL_GPL(tdm_poll_tx);


/**
 * Get voice block size for transmit or receive operations
 * @param tdm_dev - tdm device registered on TDM bus
 * @return voice block size, or error if returned value less 0
 */
int tdm_get_voice_block_size(struct tdm_device *tdm_dev)
{
        struct tdm_voice_channel *ch;

        ch = tdm_dev->ch;
        if (ch == NULL)
                return -ENODEV;

        return ch->buffer_len;
}
EXPORT_SYMBOL_GPL(tdm_get_voice_block_size);



/**
 * tdm_register_board_info - register TDM devices for a given board
 * @info: array of chip descriptors
 * @n: how many descriptors are provided
 * Context: can sleep
 */
int __init
tdm_register_board_info(struct tdm_board_info const *info, unsigned n)
{
        struct tdm_board_devices *bi;
        int i;

        bi = kzalloc(n * sizeof(*bi), GFP_KERNEL);
        if (!bi)
                return -ENOMEM;

        for (i = 0; i < n; i++, bi++, info++) {
                struct tdm_controller *tdm;

                memcpy(&bi->bi, info, sizeof(*info));
                mutex_lock(&board_lock);

                list_add_tail(&bi->list, &tdm_devices_list);
                list_for_each_entry(tdm, &tdm_controller_list, list)
                        tdm_match_controller_to_boardinfo(tdm, &bi->bi);

                mutex_unlock(&board_lock);
        }

        return 0;
}



static int __init tdm_core_init(void)
{
        int status;

        status = bus_register(&tdm_bus_type);
        if (status < 0)
                goto err0;

        status = class_register(&tdm_class);
        if (status < 0)
                goto err1;
        return 0;

err1:
        bus_unregister(&tdm_bus_type);
err0:
        return status;
}

postcore_initcall(tdm_core_init);

