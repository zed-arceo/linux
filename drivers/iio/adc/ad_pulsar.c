// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices PulSAR ADC family driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/regulator/consumer.h>

#define AD_PULSAR_NS_TO_HZ(T)   div_u64(NSEC_PER_SEC,T)

#define ad_pulsar_CHANNEL(info)                                                 \
        {                                                                       \
                .type = IIO_VOLTAGE,                                            \
                .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),        \
		.scan_type = {						        \
			.sign = info.input_type == SINGLE_ENDED ? 'u' : 's',    \
			.storagebits = 32,			                \
			.realbits = info.resolution,				\
		},                                                              \
        }

enum {
        ID_AD7988_5 = 0,
        ID_AD7988_1,
        ID_AD7984,
        ID_AD7983,
        ID_AD7982,
        ID_AD7980,
        ID_AD7946,
        ID_AD7942,
        ID_AD7693,
        ID_AD7691,
        ID_AD7690,
        ID_AD7688,
        ID_AD7687,
        ID_AD7686,
        ID_AD7685
};

enum ad_pulsar_input_type {
        SINGLE_ENDED = 0,
        DIFFERENTIAL
};

struct ad_pulsar_chip_info {
        enum ad_pulsar_input_type input_type;
        int max_rate;
        int sclk_rate;
        int resolution;
};

static const struct ad_pulsar_chip_info ad_pulsar_chip_infos[] = {
        [ID_AD7988_5]   = { 
                .input_type = SINGLE_ENDED, 
                .max_rate = 500000, 
                .resolution = 16, 
                .sclk_rate = 40000000
                },
        [ID_AD7988_1]   = { 
                .input_type = SINGLE_ENDED, 
                .max_rate = 100000, 
                .resolution = 16, 
                .sclk_rate = 40000000
                },
        [ID_AD7984]     = { 
                .input_type = DIFFERENTIAL, 
                .max_rate = 1333333, 
                .resolution = 18, 
                .sclk_rate = 80000000
                },
        [ID_AD7983]     = { 
                .input_type = SINGLE_ENDED, 
                .max_rate = 1333333, 
                .resolution = 16, 
                .sclk_rate = 80000000
                },
        [ID_AD7982]     = { 
                .input_type = DIFFERENTIAL, 
                .max_rate = 1000000, 
                .resolution = 18, 
                .sclk_rate = 40000000
                },
        [ID_AD7980]     = { 
                .input_type = SINGLE_ENDED, 
                .max_rate = 1000000, 
                .resolution = 16, 
                .sclk_rate = 1
                },
        [ID_AD7946]     = { 
                .input_type = SINGLE_ENDED, 
                .max_rate = 500000, 
                .resolution = 14, 
                .sclk_rate = 40000000
                },
        [ID_AD7942]     = { 
                .input_type = SINGLE_ENDED, 
                .max_rate = 250000, 
                .resolution = 14, 
                .sclk_rate = 40000000
                },
        [ID_AD7693]     = { 
                .input_type = DIFFERENTIAL, 
                .max_rate = 500000, 
                .resolution = 16, 
                .sclk_rate = 40000000
                },
        [ID_AD7691]     = { 
                .input_type = DIFFERENTIAL, 
                .max_rate = 250000, 
                .resolution = 18, 
                .sclk_rate = 40000000
                },
        [ID_AD7690]     = { 
                .input_type = DIFFERENTIAL, 
                .max_rate = 400000, 
                .resolution = 18, 
                .sclk_rate = 40000000
                },
        [ID_AD7688]     = { 
                .input_type = DIFFERENTIAL, 
                .max_rate = 500000, 
                .resolution = 16, 
                .sclk_rate = 40000000
                },
        [ID_AD7687]     = { 
                .input_type = DIFFERENTIAL, 
                .max_rate = 250000, 
                .resolution = 16, 
                .sclk_rate = 40000000
                },
        [ID_AD7686]     = { 
                .input_type = SINGLE_ENDED, 
                .max_rate = 500000, 
                .resolution = 16, 
                .sclk_rate = 40000000
                },
        [ID_AD7685]     = { 
                .input_type = SINGLE_ENDED, 
                .max_rate = 250000, 
                .resolution = 16, 
                .sclk_rate = 40000000
                }
};

static const struct iio_chan_spec ad_pulsar_channels[] = {
        [ID_AD7988_5]   = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7988_5]),
        [ID_AD7988_1]   = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7988_1]),
        [ID_AD7984]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7984]  ),
        [ID_AD7983]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7983]  ),
        [ID_AD7982]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7982]  ),
        [ID_AD7980]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7980]  ),
        [ID_AD7946]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7946]  ),
        [ID_AD7942]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7942]  ),
        [ID_AD7693]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7693]  ),
        [ID_AD7691]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7691]  ),
        [ID_AD7690]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7690]  ),
        [ID_AD7688]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7688]  ),
        [ID_AD7687]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7687]  ),
        [ID_AD7686]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7686]  ),
        [ID_AD7685]     = ad_pulsar_CHANNEL(ad_pulsar_chip_infos[ID_AD7685]  )
        
};

struct ad_pulsar_adc {
        const struct ad_pulsar_chip_info *info;
        struct pwm_device *cnv;
	struct spi_device *spi;
	struct regulator *vref;
	struct regulator *vio;
        int spi_speed_hz;
        int samp_freq;
};

static ssize_t ad_pulsar_samp_freq_avail(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
        int step, freq, len = 0;

        step = adc->info->max_rate / 10;
        step = step - (step % 10);
        for (freq = step; freq <= adc->info->max_rate; freq += step)
                len += scnprintf(buf + len, PAGE_SIZE - len, "%d ", freq);
        buf[len - 1] = '\n';

        return len;
}
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(ad_pulsar_samp_freq_avail);

static int ad_pulsar_set_samp_freq(struct ad_pulsar_adc *adc, int freq)
{
        /* Compute the period using a miltiple of 10 ns */
        int period = DIV_ROUND_UP(100000000UL, freq);
        struct pwm_state cnv_state;
	int ret;

        period /= 10;
        cnv_state.period = period;
        cnv_state.duty_cycle = 10;
        cnv_state.offset = 0;
	ret = pwm_apply_state(adc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

        adc->samp_freq = freq;

        return 0;
}

static int ad_pulsar_read_raw(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
                              int *val, int *val2, long info)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
        int ret;

        switch (info) {
        case IIO_CHAN_INFO_SAMP_FREQ:
                *val = adc->samp_freq;
                
                return IIO_VAL_INT;
        case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(adc->vref);
		if (ret < 0)
			return ret;
		*val = ret / 1000;
		*val2 = adc->info->resolution;
        
		return IIO_VAL_FRACTIONAL_LOG2;
        default:
                return -EINVAL;
        }
}

static int ad_pulsar_write_raw(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
                               int val, int val2, long info)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);

        switch (info){
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad_pulsar_set_samp_freq(adc, val);
	default:
		return -EINVAL;
        }
}

static int ad_pulsar_set_conversion(struct ad_pulsar_adc *adc, bool enabled)
{
        struct pwm_state cnv_state;

	pwm_get_state(adc->cnv, &cnv_state);
	cnv_state.enabled = enabled;

	return pwm_apply_state(adc->cnv, &cnv_state);
}

static void ad_pulsar_reg_disable(void *data)
{
	regulator_disable(data);
}

static void ad_pulsar_cnv_diasble(void *data)
{
	pwm_disable(data);
}

static int ad_pulsar_buffer_preenable(struct iio_dev *indio_dev)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
        unsigned int spi_rx_data[2];
        unsigned int spi_tx_data[2];
	struct spi_transfer xfer = {
                .tx_buf = spi_tx_data,
                .rx_buf = spi_rx_data,
		.len = 1,
		.bits_per_word = adc->info->resolution,
                .speed_hz = adc->info->sclk_rate,
	};
	
	struct spi_message msg;
	int ret;

        ad_pulsar_set_conversion(adc, true);
        
        spi_bus_lock(adc->spi->master);
        spi_message_init_with_transfers(&msg, &xfer, 1);
        ret = spi_engine_offload_load_msg(adc->spi, &msg);
        if (ret < 0)
                return ret;
        spi_engine_offload_enable(adc->spi, true);

	return ret;
}

static int ad_pulsar_buffer_postdisable(struct iio_dev *indio_dev)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);

        spi_engine_offload_enable(adc->spi, false);
        spi_bus_unlock(adc->spi->master);

	return ad_pulsar_set_conversion(adc, false);
}

static int ad_pulsar_dma_submit(struct iio_dma_buffer_queue *queue,
                                struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops ad_pulsar_dma_buffer_ops = {
	.submit = ad_pulsar_dma_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ad_pulsar_buffer_ops = {
	.preenable = &ad_pulsar_buffer_preenable,
	.postdisable = &ad_pulsar_buffer_postdisable,
};

static struct attribute *ad_pulsar_attrs[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad_pulsar_attrs_group = {
        .attrs = ad_pulsar_attrs,
};

static const struct iio_info ad_pulsar_info = {
        .attrs = &ad_pulsar_attrs_group,
        .read_raw = ad_pulsar_read_raw,
        .write_raw = ad_pulsar_write_raw,
};

static int ad_pulsar_setup(struct iio_dev *indio_dev)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
        int ret, voltage_mv, i;

        ret = regulator_get_voltage(adc->vio);
        if (ret < 0)
                return ret;
        voltage_mv = ret / 1000;

        // if (adc->info->sclk_rate) {
        //         for (i = 0; i < ARRAY_SIZE(ad_pulsar_vio_ranges_mv); i++) {
        //                 if (voltage_mv > ad_pulsar_vio_ranges_mv[i]) {

        //                 }
        //         }
        // }
}

static int ad_pulsar_probe(struct spi_device *spi)
{
        struct ad_pulsar_adc *adc;
	struct iio_buffer *buffer;
        struct iio_dev *indio_dev;
        int device_id;
        int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;

	adc->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->vref))
		return PTR_ERR(adc->vref);
	ret = regulator_enable(adc->vref);
	if (ret) {
                dev_err(&spi->dev, "Failed to enable VREF regulator");
		return ret;
        }
	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_reg_disable, adc->vref);
	if (ret)
		return ret;
        
	adc->vio = devm_regulator_get(&spi->dev, "vio");
	if (IS_ERR(adc->vio))
		return PTR_ERR(adc->vio);
	ret = regulator_enable(adc->vio);
	if (ret) {
                dev_err(&spi->dev, "Failed to enable VIO regulator");
		return ret;
        }
	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_reg_disable, adc->vio);
	if (ret)
		return ret;

        adc->cnv = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(adc->cnv))
		return PTR_ERR(adc->cnv);
	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_cnv_diasble, adc->cnv);

        device_id = spi_get_device_id(spi)->driver_data;

        adc->info = &ad_pulsar_chip_infos[device_id];

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->dev.parent = &spi->dev;
        indio_dev->channels = &ad_pulsar_channels[device_id];
	indio_dev->num_channels = 1;
        indio_dev->info = &ad_pulsar_info;
        indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad_pulsar_buffer_ops;

        buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
					         "rx",
					         &ad_pulsar_dma_buffer_ops,
					         indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

        return iio_device_register(indio_dev);
}

static const struct spi_device_id ad_pulsar_id_table[] = {
	{ "ad7988-5" , ID_AD7988_5 },
        { "ad7988-1" , ID_AD7988_1 },
        { "ad7984"   , ID_AD7984   },
        { "ad7983"   , ID_AD7983   },
        { "ad7982"   , ID_AD7982   },
        { "ad7980"   , ID_AD7980   },
        { "ad7946"   , ID_AD7946   },
        { "ad7942"   , ID_AD7942   },
        { "ad7693"   , ID_AD7693   },
        { "ad7691"   , ID_AD7691   },
        { "ad7690"   , ID_AD7690   },
        { "ad7688"   , ID_AD7688   },
        { "ad7687"   , ID_AD7687   },
        { "ad7686"   , ID_AD7686   },
        { "ad7685"   , ID_AD7685   },
	{}
};
MODULE_DEVICE_TABLE(spi, ad_pulsar_id_table);

static const struct of_device_id ad_pulsar_of_match[] = {
        { .compatible = "ad7988-5" },
        { .compatible = "ad7988-1" },
        { .compatible = "ad7984"   },
        { .compatible = "ad7983"   },
        { .compatible = "ad7982"   },
        { .compatible = "ad7980"   },
        { .compatible = "ad7946"   },
        { .compatible = "ad7942"   },
        { .compatible = "ad7693"   },
        { .compatible = "ad7691"   },
        { .compatible = "ad7690"   },
        { .compatible = "ad7688"   },
        { .compatible = "ad7687"   },
        { .compatible = "ad7686"   },
        { .compatible = "ad7685"   },
	{ },
};
MODULE_DEVICE_TABLE(of, ad_pulsar_of_match);

static struct spi_driver ad_pulsar_driver = {
	.driver = {
		.name = "ad_pulsar",
		.of_match_table = ad_pulsar_of_match,
	},
	.probe = ad_pulsar_probe,
	.id_table = ad_pulsar_id_table,
};
module_spi_driver(ad_pulsar_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices PulSAR ADC family driver");
MODULE_LICENSE("GPL v2");