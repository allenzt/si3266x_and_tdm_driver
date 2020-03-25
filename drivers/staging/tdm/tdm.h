/*
 * tdm_base.h
 *
 *  Created on: 20.01.2012
 *      Author: Michail Kurochkin
 */

#ifndef TDM_BASE_H_
#define TDM_BASE_H_

#include <linux/device.h>

extern struct bus_type tdm_bus_type;



/**
 * General hardware TDM settings
 */
struct tdm_controller_hw_settings {
	u8 fs_freq; /*  Frequency of discretization for audio transport. Normally 8KHz. */
	u8 count_time_slots; /*  Total count of time slots in frame. One time slot = 8bit */
	u8 channel_size; /*  Sample size (in time slots). 1 or 2 time slots. */

	/*  Controller generate PCLK clock - TDM_CLOCK_OUTPUT, */
	/*  or clock generate remote device - TDM_CLOCK_INPUT */
#define TDM_CLOCK_INPUT 0
#define TDM_CLOCK_OUTPUT 1
	u8 clock_direction; /*  Drirection for PCLK */
	u8 fs_clock_direction;  /*  Drirection for FS */

	/*  FS and data polarity. Detection on rise or fall */
#define TDM_POLAR_NEGATIVE 0
#define TDM_POLAR_POSITIV 1
	u8 fs_polarity;
	u8 data_polarity;

	struct mbus_dram_target_info	*dram;
};


/*
 * Voice channel
 */
struct tdm_voice_channel {
	u8 channel_num; /*  Hardware channel number */
	u8 mode_wideband; /*  1 - support wideband mode for current voice channel */
	u8 tdm_channel; /*  TDM channel on registered voice channel */
	u8 buffer_len; /*  Length of transmit and receive buffers */
	void *private_data; /*  hardware dependency channel private data */

	/*  wait queue for transmit and receive operations */
	wait_queue_head_t tx_queue;
	wait_queue_head_t rx_queue;

	struct list_head list; /*  Union all tdm voice channels by one controller */

	struct device *dev; /*  device requested voice channel */
};


/**
 * Remote device connected to TDM bus
 */
struct tdm_device {
	struct device dev; /*  device connected to TDM bus */
	struct tdm_controller *controller; /*  controller attendant TDM bus */
	u8 tdm_channel_num; /*  requested TDM channel number on TDM frame */
	u16 buffer_sample_count; /*  count samples for tx and rx buffers */
	u8 mode_wideband; /*  quality mode 1 or 0. Wideband mode demand is 16bit tdm channel size */
	struct tdm_voice_channel *ch; /*  requested voice channel */
	char modalias[32];
};


/**
 * Driver remote device connected to TDM bus
 */
struct tdm_driver {
	int			(*probe)(struct tdm_device *tdm_dev);
	int			(*remove)(struct tdm_device *tdm_dev);
	void			(*shutdown)(struct tdm_device *tdm_dev);
	int			(*suspend)(struct tdm_device *tdm_dev, pm_message_t mesg);
	int			(*resume)(struct tdm_device *tdm_dev);
	struct device_driver	driver;
};


/**
 * TDM controller device
 */
struct tdm_controller {
	struct device	dev;
	spinlock_t		lock;

	struct list_head list; /*  Union all TDM controllers */

	s16 bus_num; /*  Number of controller, use -1 for auto numeration */

/* 	struct tdm_voice_channel *voice_channels; // Hardware or software voice channels transport */
/* 	u8 count_voice_channels; // count voice channels supported by current TDM controller */

	/* List of voice channels */
	struct list_head voice_channels;

	/*  TDM hardware settings */
	struct tdm_controller_hw_settings *settings;

	int (*init_voice_channel)(struct tdm_device *tdm_dev, struct tdm_voice_channel* ch);
	int (*release_voice_channel)(struct tdm_device *tdm_dev);
	int (*send)(struct tdm_voice_channel *ch, u8 *data);
	int (*recv)(struct tdm_voice_channel *ch, u8 *data);
	int (*run_audio)(struct tdm_device *tdm_dev);
	int (*stop_audio)(struct tdm_device *tdm_dev);
	int (*poll_rx)(struct tdm_device *tdm_dev);
	int (*poll_tx)(struct tdm_device *tdm_dev);

	/*  called on release() for TDM device to free memory provided by tdm_controller */
	void	(*cleanup)(struct tdm_device *tdm);
};



/*
 * Board specific information for requested TDM channel
 */
struct tdm_board_info {
	/* the device name and module name are coupled, like platform_bus;
	 * "modalias" is normally the driver name.
	 *
	 * platform_data goes to tdm_controller.dev.platform_data,
	 * controller_data goes to tdm_device.controller_data,
	 * irq is copied too
	 */
	char		modalias[32];

	/* bus_num is board specific and matches the bus_num of some
	 * tdm_controller that will probably be registered later.
	 */
	u16		bus_num;

	const void	*platform_data;
	void		*controller_data;

	u8		tdm_channel_num; /*  Number TDM channel for connected device */
	u8 		buffer_sample_count; /*  Size for transmit and receive buffer */
	u8		mode_wideband; /*  1 - Enable wideband mode for requested TDM */

	struct list_head	list; /*  Entry for union all board info */
};




/**
 * Search address tdm_controller structure contained device stucture
 * @param dev - device
 * @return pointer to tdm_device
 */
static inline struct tdm_controller *to_tdm_controller(struct device *dev) {
	return dev ? container_of(dev, struct tdm_controller, dev) : NULL;
}


/**
 * Search address tdm_device structure contained device stucture
 * @param dev - device
 * @return pointer to tdm_device
 */
static inline struct tdm_device *to_tdm_device(struct device *dev) {
	return dev ? container_of(dev, struct tdm_device, dev) : NULL;
}

/**
 * Search address tdm_driver structure contained device stucture
 * @param dev - device
 * @return pointer to tdm_driver
 */
static inline struct tdm_driver *to_tdm_driver(struct device_driver *drv) {
	return drv ? container_of(drv, struct tdm_driver, driver) : NULL;
}


/**
 * Get private driver tdm controller data
 * @param tdm
 */
static inline void *tdm_controller_get_devdata(struct tdm_controller *tdm)
{
	return dev_get_drvdata(&tdm->dev);
}


/**
 * decrement pointer counter to tdm_controller
 * @param tdm - tdm_controller
 */
static inline void tdm_controller_put(struct tdm_controller *tdm)
{
	if (tdm)
		put_device(&tdm->dev);
}


/**
 * Increment pointer counter to tdm_controller
 * @param tdm - tdm_controller
 */
static inline struct tdm_controller *tdm_controller_get(struct tdm_controller *tdm) {
	if (!tdm || !get_device(&tdm->dev))
		return NULL;
	return tdm;
}


/**
 * Store private data for tdm device
 * @param tdm_dev - tdm device
 * @param data - private data
 */
static inline void tdm_set_drvdata(struct tdm_device *tdm_dev, void *data)
{
	dev_set_drvdata(&tdm_dev->dev, data);
}


/**
 * Get stored early private data for tdm device
 * @param tdm_dev - tdm device
 */
static inline void *tdm_get_drvdata(struct tdm_device *tdm_dev)
{
	return dev_get_drvdata(&tdm_dev->dev);
}


int tdm_register_driver(struct tdm_driver *tdm_drv);

void tdm_unregister_driver(struct tdm_driver *tdm_dev);

struct tdm_controller *tdm_alloc_controller(struct device *dev, unsigned size);

struct tdm_voice_channel *tdm_alloc_voice_channel(void);

int tdm_free_controller(struct tdm_controller *tdm);

int tdm_controller_register(struct tdm_controller *tdm);

void tdm_controller_unregister(struct tdm_controller *tdm);

struct tdm_device *tdm_new_device(struct tdm_controller *tdm,
                                  struct tdm_board_info *chip);

int tdm_add_device(struct tdm_device *tdm_dev);

int tdm_recv(struct tdm_device *tdm_dev, u8 *data);

int tdm_send(struct tdm_device *tdm_dev, u8 *data);

int tdm_run_audio(struct tdm_device *tdm_dev);

int tdm_stop_audio(struct tdm_device *tdm_dev);

int tdm_poll_rx(struct tdm_device *tdm_dev);

int tdm_poll_tx(struct tdm_device *tdm_dev);

int tdm_get_voice_block_size(struct tdm_device *tdm_dev);

int
tdm_register_new_voice_channel(struct tdm_controller *tdm,
    struct tdm_voice_channel *ch,
    void *driver_private);

int tdm_free_voice_channels(struct tdm_controller *tdm);

struct tdm_voice_channel *
get_voice_channel_by_num(struct tdm_controller *tdm, int num);


#ifdef CONFIG_TDM
int __init
tdm_register_board_info(struct tdm_board_info const *info, unsigned n);
#else
/* board init code may ignore whether TDM is configured or not */
static inline int __init
tdm_register_board_info(struct tdm_board_info const *info, unsigned n)
{
	return 0;
}
#endif

#endif /* TDM_BASE_H_ */
