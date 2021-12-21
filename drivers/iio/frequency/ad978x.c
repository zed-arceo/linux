/* SPDX-License-Identifier: GPL-2.0
/*
 * AD978X family device driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "cf_axi_dds.h"

/* AD978X Registers */
#define AD978X_REG_SPI_CONTROL          0x00
#define AD978X_REG_DATA_CONTROL         0x02
#define AD978X_REG_POWER_DOWN           0x03
#define AD978X_REG_SETUP_AND_HOLD       0x04
#define AD978X_REG_TIMING_ADJUST        0x05
#define AD978X_REG_SEEK                 0x06
#define AD978X_REG_MIX_MODE             0x0A
#define AD978X_REG_DAC1_FSC             0x0B
#define AD978X_REG_DAC1_FSC_MSBS        0x0C
#define AD978X_REG_AUXDAC1              0x0D
#define AD978X_REG_AUXDAC1_MSB          0x0E
#define AD978X_REG_DAC2_FSC             0x0F
#define AD978X_REG_DAC2_FSC_MSBS        0x10
#define AD978X_REG_AUXDAC2              0x11
#define AD978X_REG_AUXDAC2_MSB          0x12
#define AD978X_REG_BIST_CONTROL         0x1A
#define AD978X_REG_BIST_RESULT_1_LOW    0x1B
#define AD978X_REG_BIST_RESULT_1_HIGH   0x1C
#define AD978X_REG_BIST_RESULT_1_LOW    0x1D
#define AD978X_REG_BIST_RESULT_2_HIGH   0x1E
#define AD978X_REG_HARDWARE_REVISION    0x1F

/* AD978X_REG_SPI_CONTROL */
#define AD978X_SDIO_DIR                 BIT(7)
#define AD978X_LSBFIRST                 BIT(6)
#define AD978X_RESET                    BIT(5)
/* AD978X_REG_DATA_CONTROL */
#define AD978X_DATA                     BIT(7)
#define AD978X_INVDCO                   BIT(4)
/* AD978X_REG_POWER_DOWN */
#define AD978X_PD_DCO                   BIT(7)
#define AD978X_PD_INPT                  BIT(6)
#define AD978X_PD_AUX2                  BIT(5)
#define AD978X_PD_AUX1                  BIT(4)
#define AD978X_PD_BIAS                  BIT(3)
#define AD978X_PD_CLK                   BIT(2)
#define AD978X_PD_DAC2                  BIT(1)
#define AD978X_PD_DAC1                  BIT(0)
/* AD978X_REG_SETUP_AND_HOLD */
#define AD978X_SET                      GENMASK(7,4)
#define AD978X_HLD                      GENMASK(3,0)
/* AD978X_REG_TIMING_ADJUST */
#define AD978X_SAMP_DLY                 GENMASK(4,0)
/* AD978X_REG_SEEK */
#define AD978X_LVDS_LOW                 BIT(2)
#define AD978X_LVDS_HIGH                BIT(1)
#define AD978X_SEEK                     BIT(0)
/* AD978X_REG_MIX_MODE */
#define AD978X_DAC1_MIX                 GENMASK(3,2)
#define AD978X_DAC2_MIX                 GENMASK(1,0)
#define AD978X_DAC_MIX_MODE_MSK(dac)    GENMASK((1 + (2 * dac), (2 * dac))
#define AD978X_DAC_MIX_MODE(dac, mode)  (mode << (2 * dac))
/* AD978X_REG_DAC1_FSC_MSBS */
#define AD978X_DAC1FSC_MSB              GENMASK(1,0)
/* AD978X_REG_DAC2_FSC_MSBS */
#define AD978X_DAC2FSC_MSB              GENMASK(1,0)
/* AD978X_REG_BIST_CONTROL */
#define AD978X_BISTEN                   BIT(7)
#define AD978X_BISTRD                   BIT(6)
#define AD978X_BISTCLR                  BIT(5)
/* AD978X_REG_HARDWARE_REVISION */
#define AD978X_VERSION                  GENMASK(7,4)
#define AD978X_DEVICE                   GENMASK(3,0)
#define AD978X_HARDWARE_VERSION         0x1F
/* AD978X_AUX_DAC */
#define AD978X_REG_AUXDAC_MSB(dac)      (AD978X_REG_AUXDAC1_MSB + (4 * dac))
#define AD978X_REG_AUXDAC(dac)          (AD978X_REG_AUXDAC1 + (4 * dac))
#define AD978X_AUXSGN_MSK               BIT(7)
#define AD978X_AUXDIR_MSK               BIT(6)
#define AD978X_AUXDAC_MSB_MSK           GENMASK(1,0)
#define AD978X_AUXDAC_AUX_P             0
#define AD978X_AUXDAC_AUX_N             BIT(7)
#define AD978X_AUXDAC_OFFSET_I_LSB_NA   1955

/* AD978X SPI ops */
#define AD978X_SPI_WRITE                0
#define AD978X_SPI_READ                 BIT(7)
#define AD978X_SPI_TRANSFER_NBYTES(x)   (((x - 1) << 4) & GENMASK(6,5))

/* AD978X timing defs */
#define SEEK                            0
#define SET                             1
#define HLD                             2

enum ad978x_mix_mode {
        NORMAL_MODE,
        MIX_MODE,
        RZ_MODE
};

enum ad978x_output_type {
        SOURCE = 0,
        SINK = BIT(6)
};

struct ad978x_chan {
        enum ad978x_output_type auxdac_output_type;
        enum ad978x_mix_mode    mix_mode;
        unsigned int            offset;
        unsigned int            gain;
};

struct ad978x_device {
        struct spi_device       *spi;
        struct regulator        *ref_io;
	struct clk		*sampl_clk;
	struct gpio_desc        *reset_gpio;
        struct ad978x_chan      ch[2];
        unsigned short          vref_mv;
        unsigned int            resolution;
        unsigned int            sampling_freq;
};

enum ad978x_device_ids {
        ID_AD9783,
        ID_AD9781,
        ID_AD9780,
};

static inline struct ad978x_device *indio_dev_to_phy(struct iio_dev *indio_dev)
{
        struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
        
        return conv->phy;
}

static int ad978x_spi_read(struct spi_device *spi, unsigned int reg)
{
        unsigned int tx;

	tx = reg | AD978X_SPI_READ | AD978X_SPI_TRANSFER_NBYTES(1);
	
        return spi_w8r8(spi, tx);
}

static int ad978x_spi_write(struct spi_device *spi, unsigned int reg, unsigned int val)
{
        unsigned char tx[2] = {reg, val};

        reg |= AD978X_SPI_WRITE | AD978X_SPI_TRANSFER_NBYTES(1);
	
        return spi_write(spi, tx, sizeof(tx));
}

static int ad978x_spi_write_mask(struct spi_device *spi, unsigned int reg, unsigned int mask,
                             unsigned int val)
{
	unsigned int reg_val;
	int ret;

        ret = ad978x_spi_read(spi, reg);
        if (ret < 0)
                return ret;

        return ad978x_spi_write(spi, reg, (reg_val & ~mask) | val);
}

static int ad978x_setup(struct cf_axi_converter *conv)
{
        int ret;

        if (phy->reset_gpio) {
                gpiod_set_value_cansleep(phy->reset_gpio, 1);
                // usleep_range(10, 15);
                gpiod_set_value_cansleep(phy->reset_gpio, 0);
                // usleep_range(10, 15);
        }

        ret = ad978x_spi_write(phy->spi, AD978X_REG_SPI_CONTROL, AD978X_RESET);
        if (ret < 0)
                return -ENXIO;

        return 0;
}

static int ad978x_set_sampling_freq(struct iio_dev *indio_dev, unsigned int freq)
{
        struct ad978x_device *phy = indio_dev_to_phy(indio_dev);
        int ret;
        
        ret = clk_round_rate(phy->sampl_clk, freq);
        if (ret < 0)
                return ret;

        phy->sampling_freq = ret;

        return 0;
}

static inline int ad978x_seek(struct ad978x_device *phy)
{
        int ret;

        ret = ad978x_spi_read(phy->spi, AD978X_REG_SEEK);
        if (ret < 0)
                return ret;
        else
                return ret & AD978X_SEEK;
}

static int ad978x_timing_table(struct ad978x_device *phy, unsigned char table[32][3])
{
        unsigned char smp, set, hld, num_seek, max_seek;
        int ret;

        for (smp = 0; smp < 32; smp++) {
                ret = ad978x_spi_write(phy->spi, AD978X_REG_SETUP_AND_HOLD, 0);
                if (ret < 0)
                        return ret;
                ret = ad978x_spi_write(phy->spi, AD978X_REG_TIMING_ADJUST, smp);
                if (ret < 0)
                        return ret;
                ret = ad978x_seek(phy);
                if (ret < 0)
                        return ret;
                table[smp][SEEK] = ret;

                set = hld = 0;
                do {
                        hld++;
                        ad978x_spi_write_mask(phy->spi, AD978X_REG_SETUP_AND_HOLD, AD978X_HLD, hld);
                        ret = ad978x_seek(phy);
                        if (ret < 0)
                                return ret;
                }
                while(!ret && hld < 16);
                table[smp][HLD] = hld;

                smp = hld = 0;
                ad978x_spi_write(phy->spi, AD978X_REG_SETUP_AND_HOLD, 0);
                do {
                        ret++;
                        ad978x_spi_write_mask(phy->spi, AD978X_REG_SETUP_AND_HOLD, AD978X_SET, set);
                        ret = ad978x_seek(phy);
                        if (ret < 0)
                                return ret;
                }
                while(!ret && hld < ret);
                table[smp][SET] = set;
        }
}

static int ad978x_timing_adjust(struct ad978x_device *phy)
{
        int table[32][3], res[3], ret, smp, min = 16;

        // ret = ad978x_timing_table(phy, &table);
        if (ret < 0)
                return ret;

        for (smp = 0; smp < 32; smp++) {
                if (table[smp][SEEK] == 1 && (table[smp][SET] < table[smp][HLD])) {
                        ret = abs((table[smp][HLD] - table[smp][SET]));
                        if (ret <= min) {
                                min = ret;
                                // res = table[smp];
                                if ((table[smp - 1][SEEK] == table[smp + 1][SEEK] == 1) &&
                                    (table[smp][HLD] + table[smp][SET]) < 8)
                                    break;
                        }
                }
        }
        if (min != 16) {
                // dev_err(phy->spi, "Could not find and optimal value for the parallel port timing");
                return -EINVAL;
        }
        if (!(table[smp - 1][SEEK] == table[smp + 1][SEEK] == 1) && 
            !((table[smp][HLD] + table[smp][SET]) < 8)) {
                // dev_warn(phy->spi, "Please check for excessive on the input clock line.
                //         This device may not operate at the maximum sampling rate");
        }
}

static int ad978x_get_data_clk(struct cf_axi_converter *conv)
{
        return clk_get_rate(conv->clk[CLK_DAC]);
}

static int ad978x_parse_dt(struct device *dev)
{
	// struct iio_dev *indio_dev = iio_priv_to_dev(dev->conv);
	// struct fwnode_handle *fwnode;
	// int ret;

	// fwnode = dev_fwnode(indio_dev->dev.parent);
        return 0;
}

static int ad978x_set_chan_gain(struct iio_dev *indio_dev, int ch, int gain)
{
        struct ad978x_device *phy = indio_dev_to_phy(indio_dev);
        int ret;

        /* i_fs = (86.6 + (0.220 * gain)) * 1000 / R */
        /* gain = ((i_fs * R / 1000) - 86.6) / 0.220 */
        return 0;
}

static int ad978x_set_chan_offset(struct iio_dev *indio_dev, int ch, int gain_int, int gain_frac)
{
        struct ad978x_device *phy = indio_dev_to_phy(indio_dev);
        unsigned int i_gain;
        int ret;

        i_gain = gain_int * 1000 + gain_frac;
        i_gain = DIV_ROUND_CLOSEST(i_gain, AD978X_AUXDAC_OFFSET_I_LSB_NA);
        if (i_gain > 0x3FF)
                return -EINVAL;
        
        ret = ad978x_spi_write_mask(phy->spi, AD978X_REG_AUXDAC_MSB(ch),
                                    AD978X_AUXDAC_MSB_MSK, (i_gain >> 8) & 0x03);
        if (ret < 0)
                return ret;
        ret = ad978x_spi_write(phy->spi, AD978X_REG_AUXDAC(ch), i_gain);
        if (ret < 0)
                return ret;

        phy->ch[ch].gain = i_gain;

        return 0;
}

static int ad978x_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
        struct ad978x_device *phy = indio_dev_to_phy(indio_dev);
        int ret;

        switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:

		return ad978x_set_sampling_freq(indio_dev, val);
        // case IIO_CHAN_INFO_HARDWAREGAIN:

        //         return;
        case IIO_CHAN_INFO_OFFSET:
                
                return ad978x_set_chan_offset(indio_dev, chan->channel, val, val2);
	default:

		return -EINVAL;
	}
}

static int ad978x_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
        struct ad978x_device *phy = indio_dev_to_phy(indio_dev);
        int ret;

        switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = phy->sampling_freq;
                
		return IIO_VAL_INT;
        // case IIO_CHAN_INFO_HARDWAREGAIN:

        //         return;
        case IIO_CHAN_INFO_OFFSET:
                *val = phy->ch[chan->channel].offset / 1000;
                *val2 = phy->ch[chan->channel].offset % 1000;

                return IIO_VAL_FRACTIONAL;
	default:

		return -EINVAL;
	}
}

static int ad978x_set_mix_mode(struct iio_dev *dev, const struct iio_chan_spec *chan,
			       unsigned int mode)
{
        struct ad978x_device *phy = indio_dev_to_phy(dev);
        int ret;

        // ret = ad978x_spi_write_mask(phy->spi, AD978X_REG_MIX_MODE, 
        //                             AD978X_DAC_MIX_MODE_MSK(chan->channel),
        //                             AD978X_DAC_MIX_MODE(chan->channel, mode));
        if (ret < 0)
                return ret;
        
        phy->ch[chan->channel].mix_mode = mode;

        return 0;
}

static int ad978x_get_mix_mode(struct iio_dev *dev, const struct iio_chan_spec *chan)
{
        struct ad978x_device *phy = indio_dev_to_phy(dev);

        return phy->ch[chan->channel].mix_mode;
}

// static int ad978x_set_active_output(struct iio_dev *dev, const struct iio_chan_spec *chan,
// 			            unsigned int index)
// {
//         struct ad978x_device *phy = indio_dev_to_phy(indio_dev);
//         int ret;

//         ret = ad978x_spi_write_mask(phy->spi, AD978X_REG_AUXDAC_MSB(chan->channel),
//                                     AD978X_AUXSGN_MSK, index);
//         if (ret < 0)
//                 return ret;
        
//         phy->ch[chan->channel].output_pin = index;

//         return 0;
// }

static int ad978x_set_output_type(struct iio_dev *dev, const struct iio_chan_spec *chan,
			          unsigned int type)
{
        struct ad978x_device *phy = indio_dev_to_phy(dev);
        int ret;

        ret = ad978x_spi_write_mask(phy->spi, AD978X_REG_AUXDAC_MSB(chan->channel),
                                    AD978X_AUXDIR_MSK, type);
        if (ret < 0)
                return ret;

        phy->ch[chan->channel].auxdac_output_type = type;

        return 0;
}

static int ad978x_get_output_type(struct iio_dev *dev, const struct iio_chan_spec *chan)
{
        struct ad978x_device *phy = indio_dev_to_phy(dev);

        return phy->ch[chan->channel].auxdac_output_type;
}

static const char *const ad978x_mix_mode_iio_enum[] = {
        [NORMAL_MODE] = "NORMAL_MODE",
        [MIX_MODE] = "MIX_MODE",
        [RZ_MODE] = "RZ_MODE",
};

static const char *const ad978x_output_type_iio_enum[] = {
        [SOURCE] = "SOURCE",
        [SINK] = "SINK",
};

static const struct iio_enum ad978x_iio_enums[] = {
        {
                .items = ad978x_mix_mode_iio_enum,
                .num_items = ARRAY_SIZE(ad978x_mix_mode_iio_enum),
                .set = ad978x_set_mix_mode,
                .get = ad978x_get_mix_mode,
        },
        {
                .items = ad978x_output_type_iio_enum,
                .num_items = ARRAY_SIZE(ad978x_output_type_iio_enum),
                .set = ad978x_set_output_type,
                .get = ad978x_get_output_type,
        }
};

static struct iio_chan_spec_ext_info ad978x_ext_info[] = {
        IIO_ENUM("mix_mode", IIO_SEPARATE, &ad978x_iio_enums[0]),
        IIO_ENUM("output_type", IIO_SEPARATE, &ad978x_iio_enums[1]),
	{ },
};

// static struct atribute *ad978x_attributes[] = {
//         // &iio_dev_attr_sampling_frequency_available,
//         NULL
// };

// static const struct attribute_group ad978x_attribute_group = {
//         .attrs = ad978x_attributes,
// };

#define AD978X_CHANNEL(_name, _index, _resolution)                              \
        {                                                                       \
                .type = IIO_VOLTAGE,                                            \
                .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |   \
                                            BIT(IIO_CHAN_INFO_OFFSET),          \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),        \
                .indexed = 1,                                                   \
                .output = 1,                                                    \
                .channel = _index,                                              \
                .scan_index = _index,                                           \
		.extend_name = _name,                                           \
                .ext_info = ad978x_ext_info,                                    \
                .scan_type = {                                                  \
                        .sign = 'u',                                            \
                        .realbits = _resolution,                                \
                        .storagebits = 64,                                      \
                },                                                              \
        }

static const struct iio_chan_spec ad978x_channels[][2] = {
        [ID_AD9783] = {
                AD978X_CHANNEL("I_DAC", 0, 16),
                AD978X_CHANNEL("Q_DAC", 1, 16),
        }
};

static int ad978x_probe(struct spi_device *spi)
{
	struct cf_axi_converter *conv;
        struct ad978x_device *phy;
        int device_id;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;
        
        phy->sampl_clk = devm_clk_get(&spi->dev, "clk");
	if (IS_ERR(phy->sampl_clk))
		return PTR_ERR(phy->sampl_clk);
        
        phy->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(phy->reset_gpio))
		return PTR_ERR(phy->reset_gpio);

        // phy->pdata = spi->dev.platform_data;

        device_id = spi_get_device_id(spi)->driver_data;

        conv->clk[CLK_DAC] = phy->sampl_clk;

        conv->phy = phy;
        conv->write = ad978x_spi_write;
        conv->read = ad978x_spi_read;
	conv->setup = ad978x_setup;

	conv->get_data_clk = ad978x_get_data_clk;
	conv->write_raw = ad978x_write_raw;
	conv->read_raw = ad978x_read_raw;
	// conv->attrs = &ad978x_attribute_group;
	conv->spi = spi;
	conv->id = device_id;

}

static const struct of_device_id ad978x_of_match[] = {
        { .compatible = "ad9780"},
        { .compatible = "ad9781"},
        { .compatible = "ad9783"},
        {}
};

static const struct spi_device_id ad978x_id_table[] = {
        { "ad9780", ID_AD9780 },
	{ "ad9781", ID_AD9781 },
        { "ad9783", ID_AD9783 },
	{}
};

static struct spi_driver ad978x_driver = {
        .driver = {
                .name = "ad978x",
                .owner = THIS_MODULE,
                .of_match_table = ad978x_of_match,
        },
        .probe = ad978x_probe,
        .id_table = ad978x_id_table,
};
module_spi_driver(ad978x_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD978X family device driver");
MODULE_LICENSE("GPL v2");