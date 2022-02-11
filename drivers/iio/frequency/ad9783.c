// SPDX-License-Identifier: GPL-2.0
//
// AD9783 family device driver
//
// Copyright 2022 Analog Devices Inc.

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>

#include "cf_axi_dds.h"

/* AD9783 Registers */
#define AD9783_REG_SPI_CONTROL          0x00
#define AD9783_REG_DATA_CONTROL         0x02
#define AD9783_REG_POWER_DOWN           0x03
#define AD9783_REG_SETUP_AND_HOLD       0x04
#define AD9783_REG_TIMING_ADJUST        0x05
#define AD9783_REG_SEEK                 0x06
#define AD9783_REG_MIX_MODE             0x0A
#define AD9783_REG_DAC1_FSC             0x0B
#define AD9783_REG_DAC1_FSC_MSBS        0x0C
#define AD9783_REG_AUXDAC1              0x0D
#define AD9783_REG_AUXDAC1_MSB          0x0E
#define AD9783_REG_DAC2_FSC             0x0F
#define AD9783_REG_DAC2_FSC_MSBS        0x10
#define AD9783_REG_AUXDAC2              0x11
#define AD9783_REG_AUXDAC2_MSB          0x12
#define AD9783_REG_BIST_CONTROL         0x1A
#define AD9783_REG_BIST_RESULT_1_LOW    0x1B
#define AD9783_REG_BIST_RESULT_1_HIGH   0x1C
#define AD9783_REG_BIST_RESULT_2_LOW    0x1D
#define AD9783_REG_BIST_RESULT_2_HIGH   0x1E
#define AD9783_REG_HARDWARE_REVISION    0x1F

/* AD9783_REG_SPI_CONTROL */
#define AD9783_SDIO_DIR                 BIT(7)
#define AD9783_LSBFIRST                 BIT(6)
#define AD9783_RESET                    BIT(5)
/* AD9783_REG_DATA_CONTROL */
#define AD9783_DATA                     BIT(7)
#define AD9783_DATA_UNSIGNED            BIT(7)
#define AD9783_DATA_TWOS_COMPLEMENT     0x00
#define AD9783_INVDCO                   BIT(4)
/* AD9783_REG_POWER_DOWN */
#define AD9783_PD_DCO                   BIT(7)
#define AD9783_PD_INPT                  BIT(6)
#define AD9783_PD_AUX2                  BIT(5)
#define AD9783_PD_AUX1                  BIT(4)
#define AD9783_PD_BIAS                  BIT(3)
#define AD9783_PD_CLK                   BIT(2)
#define AD9783_PD_DAC2                  BIT(1)
#define AD9783_PD_DAC1                  BIT(0)
/* AD9783_REG_SETUP_AND_HOLD */
#define AD9783_SET                      GENMASK(7, 4)
#define AD9783_HLD                      GENMASK(3, 0)
#define AD9783_SH_RESET                 0x00
#define AD9783_SH_SET(x)                FIELD_PREP(AD9783_SET, x)
/* AD9783_REG_TIMING_ADJUST */
#define AD9783_SAMP_DLY                 GENMASK(4, 0)
/* AD9783_REG_SEEK */
#define AD9783_LVDS_LOW                 BIT(2)
#define AD9783_LVDS_HIGH                BIT(1)
#define AD9783_SEEK                     BIT(0)
/* AD9783_REG_MIX_MODE */
#define AD9783_DAC1_MIX                 GENMASK(3, 2)
#define AD9783_DAC2_MIX                 GENMASK(1, 0)
#define AD9783_DAC_MIX_MODE_MSK(dac)    GENMASK(1 + (2 * (dac)), (2 * (dac)))
#define AD9783_DAC_MIX_MODE(dac, mode)  ((mode) << (2 * (dac)))
/* AD9783_REG_DAC1_FSC_MSBS */
#define AD9783_DAC1FSC_MSB              GENMASK(1, 0)
/* AD9783_REG_DAC2_FSC_MSBS */
#define AD9783_DAC2FSC_MSB              GENMASK(1, 0)
/* AD9783_REG_BIST_CONTROL */
#define AD9783_BISTEN                   BIT(7)
#define AD9783_BISTRD                   BIT(6)
#define AD9783_BISTCLR                  BIT(5)
#define AD9783_BIST_INIT                0x00
/* AD9783_REG_HARDWARE_REVISION */
#define AD9783_VERSION                  GENMASK(7, 4)
#define AD9783_DEVICE                   GENMASK(3, 0)
#define AD9783_HARDWARE_VERSION         0x1F
/* AD9783_AUX_DAC */
#define AD9783_REG_AUXDAC_MSB(dac)      (AD9783_REG_AUXDAC1_MSB + (4 * (dac)))
#define AD9783_REG_AUXDAC(dac)          (AD9783_REG_AUXDAC1 + (4 * (dac)))
#define AD9783_AUXSGN_MSK               BIT(7)
#define AD9783_AUXDIR_MSK               BIT(6)
#define AD9783_AUXDAC_MSB_MSK           GENMASK(1, 0)
#define AD9783_AUXDAC_AUX_P             0
#define AD9783_AUXDAC_AUX_N             BIT(7)
#define AD9783_AUXDAC_OFFSET_I_LSB_NA   1955
/* AD9783_DAC */
#define AD9783_REG_DAC_MSB(dac)         (AD9783_REG_DAC1_FSC_MSBS + (4 * (dac)))
#define AD9783_REG_DACB(dac)            (AD9783_REG_DAC1_FSC + (4 * (dac)))
#define AD9783_DAC_MSB_MSK              GENMASK(1, 0)
/* AD9783_BIST */
#define AD9783_REG_BIST_RESULT          AD9783_REG_BIST_RESULT_1_LOW
/* AD9783 SPI ops */
#define AD9783_SPI_WRITE                0
#define AD9783_SPI_READ                 BIT(7)
#define AD9783_SPI_TRANSFER_NBYTES(x)   FIELD_PREP(GENMASK(6, 5), ((x) - 1))
#define AD9783_SPI_RESET                0

/* AD9783 timing defs */
#define SEEK                            0
#define SET                             1
#define HLD                             2

enum ad9783_mix_mode {
	NORMAL_MODE,
	MIX_MODE,
	RZ_MODE
};

enum ad9783_output {
	AUXP = 0,
	AUXN = BIT(7)
};

enum ad9783_output_type {
	SOURCE = 0,
	SINK = BIT(6)
};

struct ad9783_ch_dac {
	enum ad9783_mix_mode    mix_mode;
	unsigned int            gain;
};

struct ad9783_ch_auxdac {
	enum ad9783_output_type output_type;
	enum ad9783_output      active_output;
	unsigned int            offset;
};

struct ad9783_phy {
	struct iio_dev		*indio_dev;
	struct spi_device       *spi;
	/* Lock for write operations */
	struct mutex            lock;
	struct cf_axi_dds_state *dds;
	struct regulator        *ref_io;
	struct clk		*sampl_clk;
	struct gpio_desc        *reset_gpio;
	struct ad9783_ch_dac    dac[2];
	struct ad9783_ch_auxdac auxdac[2];
	int                     device_id;
};

enum ad9783_device_ids {
	ID_DEV_AD9780,
	ID_DEV_AD9781,
	ID_DEV_AD9783,
};

static inline struct ad9783_phy *indio_dev_to_phy(struct iio_dev *indio_dev)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	return conv->phy;
}

static int ad9783_spi_read(struct spi_device *spi, unsigned int reg)
{
	unsigned int tx;

	tx = reg | AD9783_SPI_READ | AD9783_SPI_TRANSFER_NBYTES(1);

	return spi_w8r8(spi, tx);
}

static int ad9783_spi_write(struct spi_device *spi, unsigned int reg,
			    unsigned int val)
{
	unsigned char tx[2] = {reg, val};

	tx[0] |= AD9783_SPI_WRITE | AD9783_SPI_TRANSFER_NBYTES(1);

	return spi_write(spi, tx, sizeof(tx));
}

static int ad9783_spi_write_mask(struct spi_device *spi, unsigned int reg,
				 unsigned int mask, unsigned int val)
{
	unsigned int reg_val;
	int ret;

	ret = ad9783_spi_read(spi, reg);
	if (ret < 0)
		return ret;

	return ad9783_spi_write(spi, reg, (reg_val & ~mask) | val);
}

static inline int ad9783_seek(struct ad9783_phy *phy)
{
	int ret;

	ret = ad9783_spi_read(phy->spi, AD9783_REG_SEEK);
	if (ret < 0)
		return ret;
	else
		return ret & AD9783_SEEK;
}

static int ad9783_timing_adjust(struct ad9783_phy *phy)
{
	int ret, i, smp, set, hld, min = 16;
	unsigned char table[32][3];

	for (smp = 0; smp < 32; smp++) {
		ret = ad9783_spi_write(phy->spi, AD9783_REG_SETUP_AND_HOLD,
				       AD9783_SH_RESET);
		if (ret < 0)
			return ret;
		ret = ad9783_spi_write(phy->spi, AD9783_REG_TIMING_ADJUST,
				       smp);
		if (ret < 0)
			return ret;
		ret = ad9783_seek(phy);
		if (ret < 0)
			return ret;
		table[smp][SEEK] = ret;

		set = 0;
		hld = 0;
		do {
			hld++;
			ad9783_spi_write_mask(phy->spi,
					      AD9783_REG_SETUP_AND_HOLD,
					      AD9783_HLD, hld);
			ret = ad9783_seek(phy);
			if (ret < 0)
				return ret;
		} while (ret > 0 && hld < 16);
		table[smp][HLD] = hld;

		hld = 0;
		ad9783_spi_write(phy->spi, AD9783_REG_SETUP_AND_HOLD,
				 AD9783_SH_RESET);
		do {
			set++;
			ad9783_spi_write_mask(phy->spi,
					      AD9783_REG_SETUP_AND_HOLD,
					      AD9783_SET, AD9783_SH_SET(set));
			ret = ad9783_seek(phy);
			if (ret < 0)
				return ret;
		} while (ret > 0 && set < 16);
		table[smp][SET] = set;
	}

	for (i = 0; i < 32; i++) {
		if (table[i][SEEK] == 1 && table[i][SET] < table[i][HLD]) {
			ret = abs((table[i][HLD] - table[i][SET]));
			if (ret <= min) {
				min = ret;
				set = table[i][SET];
				hld = table[i][HLD];
				smp = i;
			}
		}
	}

	if (min == 16) {
		dev_err(&phy->spi->dev,
			"Could not find and optimal value for the port timing");
		return -EINVAL;
	}
	if (!(table[smp - 1][SEEK] == table[smp + 1][SEEK] &&
	   table[smp - 1][SEEK] == 1) &&
	   !((table[smp][HLD] + table[smp][SET]) > 8)) {
		dev_warn(&phy->spi->dev,
			 "Please check for excessive on the input clock line.");
	}

	ret = ad9783_spi_write(phy->spi, AD9783_REG_TIMING_ADJUST, smp);
	if (ret < 0)
		return ret;

	ret = ad9783_spi_write_mask(phy->spi, AD9783_REG_SETUP_AND_HOLD,
				    AD9783_SET, AD9783_SH_SET(set));
	if (ret < 0)
		return ret;

	return ad9783_spi_write_mask(phy->spi, AD9783_REG_SETUP_AND_HOLD,
				     AD9783_HLD, hld);
}

static int ad9783_selftest(struct ad9783_phy *phy, struct cf_axi_dds_state *dds)
{
	int ret;

	ret = ad9783_spi_write_mask(phy->spi, AD9783_REG_DATA_CONTROL,
				    AD9783_DATA, AD9783_DATA_UNSIGNED);
	if (ret < 0)
		return ret;

	ret = ad9783_spi_write(phy->spi, AD9783_REG_BIST_CONTROL,
			       AD9783_BIST_INIT);
	if (ret < 0)
		return ret;

	usleep_range(5000, 5005);
	cf_axi_dds_datasel(dds, 0, DATA_SEL_ZERO);
	cf_axi_dds_datasel(dds, 1, DATA_SEL_ZERO);
	usleep_range(5000, 5005);

	ret = ad9783_spi_write(phy->spi, AD9783_REG_BIST_CONTROL,
			       AD9783_BISTCLR);
	if (ret < 0)
		return ret;

	ret = ad9783_spi_write(phy->spi, AD9783_REG_BIST_CONTROL,
			       AD9783_BIST_INIT);
	if (ret < 0)
		return ret;

	ret = ad9783_spi_write(phy->spi, AD9783_REG_BIST_CONTROL,
			       AD9783_BISTEN);
	if (ret < 0)
		return ret;

	usleep_range(5000, 5005);
	cf_axi_dds_datasel(dds, 0, DATA_SEL_PNXX);
	cf_axi_dds_datasel(dds, 1, DATA_SEL_PNXX);
	usleep_range(5000, 5005);

	return 0;
}

static int ad9783_selftest_check(struct ad9783_phy *phy,
				 struct cf_axi_dds_state *dds)
{
	unsigned int bist_result[4];
	int ret, i;

	ret = ad9783_spi_write(phy->spi, AD9783_REG_BIST_CONTROL,
			       AD9783_BISTRD);
	if (ret < 0)
		return ret;

	for (i = 0; i < 4; i++) {
		bist_result[i] = ad9783_spi_read(phy->spi,
						 AD9783_REG_BIST_RESULT + i);
		if (bist_result[i] < 0)
			return bist_result[i];
	}

	usleep_range(5000, 5005);
	cf_axi_dds_datasel(dds, 0, DATA_SEL_DDS);
	cf_axi_dds_datasel(dds, 1, DATA_SEL_DDS);
	usleep_range(5000, 5005);

	if (bist_result[0] == bist_result[2] &&
	    bist_result[1] == bist_result[3])
		return 0;
	else
		return -EINVAL;
}

static int ad9783_interface_tune(struct ad9783_phy *phy)
{
	struct cf_axi_converter *conv = spi_get_drvdata(phy->spi);
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);
	int ret;

	ret = ad9783_timing_adjust(phy);
	if (ret < 0)
		return ret;

	ret = ad9783_selftest(phy, dds);
	if (ret < 0)
		return ret;

	if (ad9783_selftest_check(phy, dds) < 0)
		return -EINVAL;

	ret = ad9783_spi_write_mask(phy->spi, AD9783_REG_DATA_CONTROL,
				    AD9783_DATA, AD9783_DATA_TWOS_COMPLEMENT);
	if (ret < 0)
		return -ENXIO;

	return 0;
}

static int ad9783_set_chan_gain(struct iio_dev *indio_dev, int ch,
				int val_int, int val_frac)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	val = val_int * 1000 * 1000 + val_frac;
	val = clamp_t(typeof(val), val, 8660000, 31660000);
	val = (val * 10) - 86600000;
	val = DIV_ROUND_CLOSEST(val, 220000);

	ret = ad9783_spi_write_mask(phy->spi, AD9783_REG_DAC_MSB(ch),
				    AD9783_DAC_MSB_MSK, (val >> 8) & 0x03);
	if (ret < 0)
		return ret;

	ret = ad9783_spi_write(phy->spi, AD9783_REG_DACB(ch), val);
	if (ret < 0)
		return ret;

	phy->dac[ch].gain = DIV_ROUND_CLOSEST(((val * 220000) + 86600000), 10);

	return 0;
}

static int ad9783_set_auxdac(struct iio_dev *indio_dev, int ch,
			     int val_int, int val_frac)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	val = val_int * 1000 * 1000 + val_frac;
	val = clamp_t(typeof(val), val, 0, 2000000);
	val = DIV_ROUND_CLOSEST(val, AD9783_AUXDAC_OFFSET_I_LSB_NA);

	ret = ad9783_spi_write_mask(phy->spi, AD9783_REG_AUXDAC_MSB(ch),
				    AD9783_AUXDAC_MSB_MSK, (val >> 8) & 0x03);
	if (ret < 0)
		return ret;

	ret = ad9783_spi_write(phy->spi, AD9783_REG_AUXDAC(ch), val);
	if (ret < 0)
		return ret;

	phy->auxdac[ch].offset = val * AD9783_AUXDAC_OFFSET_I_LSB_NA;

	return 0;
}

static int ad9783_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&phy->lock);
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ad9783_set_auxdac(indio_dev, chan->address % 2,
					val, val2);
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ad9783_set_chan_gain(indio_dev, chan->channel,
					   val, val2);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&phy->lock);

	return ret;
}

static int ad9783_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		*val = phy->auxdac[chan->address % 2].offset;
		*val2 = 1000000;

		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = phy->dac[chan->address].gain;
		*val2 = 1000000;

		return IIO_VAL_FRACTIONAL;

		return 0;
	default:

		return -EINVAL;
	}
}

static int ad9783_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&phy->lock);
	if (readval) {
		ret = ad9783_spi_read(phy->spi, reg);
		*readval = ret;
	} else {
		ret = ad9783_spi_write(phy->spi, reg, writeval);
	}
	mutex_unlock(&phy->lock);

	return ret;
}

static int ad9783_set_mix_mode(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       unsigned int mode)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);
	int ret;

	ret = ad9783_spi_write_mask(phy->spi, AD9783_REG_MIX_MODE,
				    AD9783_DAC_MIX_MODE_MSK(chan->channel),
				    AD9783_DAC_MIX_MODE(chan->channel, mode));
	if (ret < 0)
		return ret;

	phy->dac[chan->channel].mix_mode = mode;

	return 0;
}

static int ad9783_get_mix_mode(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);

	return phy->dac[chan->channel].mix_mode;
}

static int ad9783_set_active_output(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int index)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);
	int ret;

	ret = ad9783_spi_write_mask(phy->spi,
				    AD9783_REG_AUXDAC_MSB(chan->address % 2),
				    AD9783_AUXSGN_MSK, index);
	if (ret < 0)
		return ret;

	phy->auxdac[chan->address % 2].active_output = index;

	return 0;
}

static int ad9783_get_active_output(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);

	return phy->auxdac[chan->address % 2].active_output;
}

static int ad9783_set_output_type(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  unsigned int type)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);
	int ret;

	ret = ad9783_spi_write_mask(phy->spi,
				    AD9783_REG_AUXDAC_MSB(chan->channel),
				    AD9783_AUXDIR_MSK, type);
	if (ret < 0)
		return ret;

	phy->auxdac[chan->address % 2].output_type = type;

	return 0;
}

static int ad9783_get_output_type(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct ad9783_phy *phy = iio_priv(indio_dev);

	return phy->auxdac[chan->address % 2].output_type;
}

static void ad987x_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

unsigned long long ad9783_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DAC]);
}

static const char *const ad9783_mix_mode_iio_enum[] = {
	[NORMAL_MODE] = "NORMAL_MODE",
	[MIX_MODE] = "MIX_MODE",
	[RZ_MODE] = "RZ_MODE",
};

static const char *const ad9783_output_type_iio_enum[] = {
	[SOURCE] = "SOURCE",
	[SINK] = "SINK",
};

static const char *const ad9783_active_output_iio_enum[] = {
	[AUXP] = "AUXP",
	[AUXN] = "AUXN",
};

static const struct iio_enum ad9783_iio_enums[] = {
	{
		.items = ad9783_mix_mode_iio_enum,
		.num_items = ARRAY_SIZE(ad9783_mix_mode_iio_enum),
		.set = ad9783_set_mix_mode,
		.get = ad9783_get_mix_mode,
	},
	{
		.items = ad9783_output_type_iio_enum,
		.num_items = ARRAY_SIZE(ad9783_output_type_iio_enum),
		.set = ad9783_set_output_type,
		.get = ad9783_get_output_type,
	},
	{
		.items = ad9783_active_output_iio_enum,
		.num_items = ARRAY_SIZE(ad9783_active_output_iio_enum),
		.set = ad9783_set_active_output,
		.get = ad9783_get_active_output,
	},
};

static struct iio_chan_spec_ext_info ad9783_dac_ext_info[] = {
	IIO_ENUM("mix_mode", IIO_SEPARATE, &ad9783_iio_enums[0]),
	IIO_ENUM_AVAILABLE("mix_mode", &ad9783_iio_enums[0]),
	{ },
};

static struct iio_chan_spec_ext_info ad9783_auxdac_ext_info[] = {
	IIO_ENUM("output_type", IIO_SEPARATE, &ad9783_iio_enums[1]),
	IIO_ENUM_AVAILABLE("output_type", &ad9783_iio_enums[1]),
	IIO_ENUM("active_output", IIO_SEPARATE, &ad9783_iio_enums[2]),
	IIO_ENUM_AVAILABLE("active_output", &ad9783_iio_enums[2]),
	{ },
};

static const struct iio_info ad9783_info = {
	.write_raw = &ad9783_write_raw,
	.read_raw = &ad9783_read_raw,
	.debugfs_reg_access = &ad9783_reg_access,
};

#define AD9783_CHANNEL_DAC(_name, _index)                               \
	{                                                               \
		.type = IIO_VOLTAGE,                                    \
		.extend_name = _name,                                   \
		.output = 1,                                            \
		.indexed = 1,                                           \
		.channel = _index,                                      \
		.address = _index,                                      \
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),  \
		.scan_index = _index,                                   \
		.ext_info = ad9783_dac_ext_info,                        \
	}

#define AD9783_CHANNEL_AUXDAC(_name, _index)                            \
	{                                                               \
		.type = IIO_VOLTAGE,                                    \
		.extend_name = _name,                                   \
		.output = 1,                                            \
		.indexed = 1,                                           \
		.channel = _index,                                      \
		.address = _index,                                      \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),           \
		.ext_info = ad9783_auxdac_ext_info,                     \
	}

static const struct iio_chan_spec ad9783_channels[][4] = {
	[ID_DEV_AD9783] = {
		AD9783_CHANNEL_DAC("TX_I", 0),
		AD9783_CHANNEL_DAC("TX_Q", 1),
		AD9783_CHANNEL_AUXDAC("AUX1", 2),
		AD9783_CHANNEL_AUXDAC("AUX2", 3),
	}
};

static int ad9783_setup(struct cf_axi_converter *conv)
{
	struct ad9783_phy *phy = conv->phy;
	struct spi_device *spi = phy->spi;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = phy->indio_dev;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad9783_info;
	indio_dev->channels = ad9783_channels[phy->device_id];
	indio_dev->num_channels = ARRAY_SIZE(ad9783_channels[phy->device_id]);

	if (phy->reset_gpio) {
		gpiod_set_value_cansleep(phy->reset_gpio, 1);
		usleep_range(10, 15);
		gpiod_set_value_cansleep(phy->reset_gpio, 0);
		usleep_range(10, 15);
	}

	ret = ad9783_spi_write(phy->spi, AD9783_REG_SPI_CONTROL,
			       AD9783_SPI_RESET);
	if (ret < 0)
		return -ENXIO;

	if (ad9783_interface_tune(phy) < 0)
		dev_warn(&phy->spi->dev, "Selftest failed.");

	phy->dac[0].gain = 8660000;
	phy->dac[1].gain = 8660000;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static int ad9783_register_converter(struct ad9783_phy *phy)
{
	struct spi_device *spi = phy->spi;
	struct cf_axi_converter *conv;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	conv->clk[CLK_DAC] = phy->sampl_clk;
	conv->id = ID_DEV_AD9783;

	conv->phy = phy;
	conv->spi = phy->spi;
	conv->write = ad9783_spi_write;
	conv->read = ad9783_spi_read;
	conv->setup = ad9783_setup;
	conv->get_data_clk = ad9783_get_data_clk;

	spi_set_drvdata(phy->spi, conv);

	return 0;
}

static int ad9783_probe(struct spi_device *spi)
{
	struct ad9783_phy *phy;
	struct iio_dev *indio_dev;
	const int *id;
	int ret;

	id = of_device_get_match_data(&spi->dev);
	if (!id)
		return -EINVAL;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (!indio_dev)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;
	phy->spi = spi;
	phy->device_id = *id;

	mutex_init(&phy->lock);

	phy->sampl_clk = devm_clk_get(&spi->dev, "dac_clk");
	if (IS_ERR(phy->sampl_clk))
		return PTR_ERR(phy->sampl_clk);

	ret = clk_prepare_enable(phy->sampl_clk);
	if (ret < 0)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, ad987x_clk_disable,
				       phy->sampl_clk);
	if (ret)
		return ret;

	phy->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
						  GPIOD_OUT_LOW);
	if (IS_ERR(phy->reset_gpio))
		return PTR_ERR(phy->reset_gpio);

	return ad9783_register_converter(phy);
}

static const int ad9780_id = ID_DEV_AD9780;
static const int ad9781_id = ID_DEV_AD9781;
static const int ad9783_id = ID_DEV_AD9783;
static const struct of_device_id ad9783_of_match[] = {
	{ .compatible = "adi,ad9780", .data = &ad9780_id},
	{ .compatible = "adi,ad9781", .data = &ad9781_id},
	{ .compatible = "adi,ad9783", .data = &ad9783_id},
	{}
};

static struct spi_driver ad9783_driver = {
	.driver = {
		.name = "ad9783",
		.owner = THIS_MODULE,
		.of_match_table = ad9783_of_match,
	},
	.probe = ad9783_probe,
};
module_spi_driver(ad9783_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9783 family device driver");
MODULE_LICENSE("GPL v2");
