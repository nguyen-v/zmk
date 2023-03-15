/*
 * Copyright (c) 2020 Rohit Gujarathi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT   sharp_ls0xx_custom

#include <logging/log.h>
LOG_MODULE_REGISTER(ls0xx_custom, CONFIG_DISPLAY_LOG_LEVEL);

#include <string.h>
#include <device.h>
#include <drivers/display.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <sys/byteorder.h>

/* Supports LS012B7DD01, LS012B7DD06, LS013B7DH03, LS013B7DH05
 * LS013B7DH06, LS027B7DH01A, LS032B7DD02, LS044Q7DH01
 */

/* Note:
 * -> high/1 means white, low/0 means black
 * -> Display expects LSB first
 */

#define LS0XX_PANEL_WIDTH   DT_INST_PROP(0, width)
#define LS0XX_PANEL_HEIGHT  DT_INST_PROP(0, height)

#define LS0XX_PIXELS_PER_BYTE  8U
/* Adding 2 for the line number and dummy byte
 * line_buf format for each row.
 * +-------------------+-------------------+----------------+
 * | line num (8 bits) | data (WIDTH bits) | dummy (8 bits) |
 * +-------------------+-------------------+----------------+
 */
#define LS0XX_BYTES_PER_LINE  ((LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE) + 2)

#define LS0XX_BIT_WRITECMD    0x01
#define LS0XX_BIT_VCOM        0x02
#define LS0XX_BIT_CLEAR       0x04

struct ls0xx_data {
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	const struct device *disp_dev;
#endif
#if DT_INST_NODE_HAS_PROP(0, extcomin_gpios)
	const struct device *extcomin_dev;
#endif
};

struct ls0xx_config {
	struct spi_dt_spec bus;
};

#if DT_INST_NODE_HAS_PROP(0, extcomin_gpios)
/* Driver will handle VCOM toggling */
static void ls0xx_vcom_toggle(void *a, void *b, void *c)
{
	struct ls0xx_data *driver = (struct ls0xx_data *)a;

	while (1) {
		gpio_pin_toggle(driver->extcomin_dev,
				DT_INST_GPIO_PIN(0, extcomin_gpios));
		k_usleep(3);
		gpio_pin_toggle(driver->extcomin_dev,
				DT_INST_GPIO_PIN(0, extcomin_gpios));
		k_msleep(1000 / DT_INST_PROP(0, extcomin_frequency));
		LOG_DBG("Toggling VCOM");
	}
}

K_THREAD_STACK_DEFINE(vcom_toggle_stack, 256);
struct k_thread vcom_toggle_thread;
#endif

static int ls0xx_blanking_off(const struct device *dev)
{
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	struct ls0xx_data *driver = dev->data;

	return gpio_pin_set(driver->disp_dev,
			    DT_INST_GPIO_PIN(0, disp_en_gpios), 1);
#else
	LOG_WRN("Unsupported");
	return -ENOTSUP;
#endif
}

static int ls0xx_blanking_on(const struct device *dev)
{
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	struct ls0xx_data *driver = dev->data;

	return gpio_pin_set(driver->disp_dev,
			    DT_INST_GPIO_PIN(0, disp_en_gpios), 0);
#else
	LOG_WRN("Unsupported");
	return -ENOTSUP;
#endif
}

static int ls0xx_cmd(const struct device *dev, uint8_t *buf, uint8_t len)
{
	const struct ls0xx_config *config = dev->config;
	struct spi_buf cmd_buf = { .buf = buf, .len = len };
	struct spi_buf_set buf_set = { .buffers = &cmd_buf, .count = 1 };

	return spi_write_dt(&config->bus, &buf_set);
}

static int ls0xx_clear(const struct device *dev)
{
	const struct ls0xx_config *config = dev->config;
	uint8_t clear_cmd[2] = { LS0XX_BIT_CLEAR, 0 };
	int err;

	err = ls0xx_cmd(dev, clear_cmd, sizeof(clear_cmd));
	spi_release_dt(&config->bus);

	return err;
}

// lookup table to increase speed
unsigned char reverse_byte(unsigned char x)
{
    static const unsigned char table[] = {
        0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
        0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
        0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
        0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
        0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
        0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
        0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
        0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
        0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
        0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
        0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
        0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
        0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
        0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
        0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
        0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
        0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
        0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
        0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
        0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
        0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
        0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
        0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
        0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
        0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
        0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
        0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
        0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
        0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
        0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
        0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
        0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
    };
    return table[x];
}

void reformat_image(uint8_t *data, uint16_t num_lines) {
  uint16_t bytes_per_line = (LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE);
  uint16_t size = (num_lines) * bytes_per_line;
  uint8_t temp[size];
  uint8_t res = 0;
  for (int i = 0; i < num_lines; i++) {
	for (int j = 0; j < bytes_per_line; j++) {
		// uint8_t in = data[i*bytes_per_line + j];
		// uint8_t res = 0;
		// for (int k = 0; k < 8; k++) {
      	//   res = (res << 1) | (in & 1);
      	//   in >>= 1;
    	// }
		res = reverse_byte(data[i*bytes_per_line + j]);
		temp[size - ((num_lines-i-1)*bytes_per_line + j) - 1] = res;
	}
  }
	memcpy(data, temp, sizeof(temp));
}

static int ls0xx_update_display(const struct device *dev,
				uint16_t start_line,
				uint16_t num_lines,
				const uint8_t *data)
{
	const struct ls0xx_config *config = dev->config;
	uint8_t write_cmd[1] = { LS0XX_BIT_WRITECMD };

	#if IS_ENABLED(CONFIG_ZMK_DISPLAY_FLIP)
		int8_t ln = LS0XX_PANEL_HEIGHT - start_line + 1;
	#else
		uint8_t ln = start_line;
	#endif

	uint8_t dummy = 27;
	struct spi_buf line_buf[3] = {
		{
			.len = sizeof(ln),
			.buf = &ln,
		},
		{
			.len = LS0XX_BYTES_PER_LINE - 2,
		},
		{
			.len = sizeof(dummy),
			.buf = &dummy,
		},
	};
	struct spi_buf_set line_set = {
		.buffers = line_buf,
		.count = ARRAY_SIZE(line_buf),
	};
	int err;

	LOG_DBG("Lines %d to %d", start_line, start_line + num_lines - 1);
	err = ls0xx_cmd(dev, write_cmd, sizeof(write_cmd));

	/* Send each line to the screen including
	 * the line number and dummy bits
	 */
	#if IS_ENABLED(CONFIG_ZMK_DISPLAY_FLIP)
		uint16_t size = (num_lines) * (LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE);

		reformat_image(data, num_lines);

		for (; ln > LS0XX_PANEL_HEIGHT - start_line - num_lines + 1; ln--) {
			line_buf[1].buf = (uint8_t *)data;
			err |= spi_write_dt(&config->bus, &line_set);
			data += LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE;
		}
	#else
		for (; ln <= start_line + num_lines - 1; ln++) {
			line_buf[1].buf = (uint8_t *)data;
			err |= spi_write_dt(&config->bus, &line_set);
			data += LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE;
		}
	#endif
	
	/* Send another trailing 8 bits for the last line
	 * These can be any bits, it does not matter
	 * just reusing the write_cmd buffer
	 */
	err |= ls0xx_cmd(dev, write_cmd, sizeof(write_cmd));

	spi_release_dt(&config->bus);

	return err;
}

/* Buffer width should be equal to display width */
static int ls0xx_write(const struct device *dev, const uint16_t x,
		       const uint16_t y,
		       const struct display_buffer_descriptor *desc,
		       const void *buf)
{
	LOG_DBG("X: %d, Y: %d, W: %d, H: %d", x, y, desc->width, desc->height);

	if (buf == NULL) {
		LOG_WRN("Display buffer is not available");
		return -EINVAL;
	}

	if (desc->width != LS0XX_PANEL_WIDTH) {
		LOG_ERR("Width not a multiple of %d", LS0XX_PANEL_WIDTH);
		return -EINVAL;
	}

	if (desc->pitch != desc->width) {
		LOG_ERR("Unsupported mode");
		return -ENOTSUP;
	}

	if ((y + desc->height) > LS0XX_PANEL_HEIGHT) {
		LOG_ERR("Buffer out of bounds (height)");
		return -EINVAL;
	}

	if (x != 0) {
		LOG_ERR("X-coordinate has to be 0");
		return -EINVAL;
	}

	/* Adding 1 since line numbering on the display starts with 1 */
	return ls0xx_update_display(dev, y + 1, desc->height, buf);
}

static int ls0xx_read(const struct device *dev, const uint16_t x,
		      const uint16_t y,
		      const struct display_buffer_descriptor *desc,
		      void *buf)
{
	LOG_ERR("not supported");
	return -ENOTSUP;
}

static void *ls0xx_get_framebuffer(const struct device *dev)
{
	LOG_ERR("not supported");
	return NULL;
}

static int ls0xx_set_brightness(const struct device *dev,
				const uint8_t brightness)
{
	LOG_WRN("not supported");
	return -ENOTSUP;
}

static int ls0xx_set_contrast(const struct device *dev, uint8_t contrast)
{
	LOG_WRN("not supported");
	return -ENOTSUP;
}

static void ls0xx_get_capabilities(const struct device *dev,
				   struct display_capabilities *caps)
{
	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = LS0XX_PANEL_WIDTH;
	caps->y_resolution = LS0XX_PANEL_HEIGHT;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO01;
	caps->current_pixel_format = PIXEL_FORMAT_MONO01;
	caps->screen_info = SCREEN_INFO_X_ALIGNMENT_WIDTH;
}

static int ls0xx_set_orientation(const struct device *dev,
				 const enum display_orientation orientation)
{
	LOG_ERR("Unsupported");
	return -ENOTSUP;
}

static int ls0xx_set_pixel_format(const struct device *dev,
				  const enum display_pixel_format pf)
{
	if (pf == PIXEL_FORMAT_MONO01) {
		return 0;
	}

	LOG_ERR("not supported");
	return -ENOTSUP;
}

static int ls0xx_init(const struct device *dev)
{
	const struct ls0xx_config *config = dev->config;
	struct ls0xx_data *driver = dev->data;

	if (!spi_is_ready(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}
	LOG_INF("ls0xx initialized");

#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	driver->disp_dev = device_get_binding(
		DT_INST_GPIO_LABEL(0, disp_en_gpios));
	if (driver->disp_dev == NULL) {
		LOG_ERR("Could not get DISP pin port for LS0XX");
		return -EIO;
	}
	LOG_INF("Configuring DISP pin to OUTPUT_HIGH");
	gpio_pin_configure(driver->disp_dev,
			   DT_INST_GPIO_PIN(0, disp_en_gpios),
			   GPIO_OUTPUT_HIGH);
#endif

#if DT_INST_NODE_HAS_PROP(0, extcomin_gpios)
	driver->extcomin_dev = device_get_binding(
		DT_INST_GPIO_LABEL(0, extcomin_gpios));
	if (driver->extcomin_dev == NULL) {
		LOG_ERR("Could not get EXTCOMIN pin port for LS0XX");
		return -EIO;
	}
	LOG_INF("Configuring EXTCOMIN pin");
	gpio_pin_configure(driver->extcomin_dev,
			   DT_INST_GPIO_PIN(0, extcomin_gpios),
			   GPIO_OUTPUT_LOW);

	/* Start thread for toggling VCOM */
	k_tid_t vcom_toggle_tid = k_thread_create(&vcom_toggle_thread,
						  vcom_toggle_stack,
						  K_THREAD_STACK_SIZEOF(vcom_toggle_stack),
						  ls0xx_vcom_toggle,
						  driver, NULL, NULL,
						  3, 0, K_NO_WAIT);
	k_thread_name_set(vcom_toggle_tid, "ls0xx_vcom");
#endif  /* DT_INST_NODE_HAS_PROP(0, extcomin_gpios) */

	/* Clear display else it shows random data */
	return ls0xx_clear(dev);
}

static struct ls0xx_data ls0xx_driver;

static const struct ls0xx_config ls0xx_config = {
	.bus = SPI_DT_SPEC_INST_GET(
		0, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		SPI_TRANSFER_LSB | SPI_CS_ACTIVE_HIGH |
		SPI_HOLD_ON_CS | SPI_LOCK_ON, 0)
};

static struct display_driver_api ls0xx_driver_api = {
	.blanking_on = ls0xx_blanking_on,
	.blanking_off = ls0xx_blanking_off,
	.write = ls0xx_write,
	.read = ls0xx_read,
	.get_framebuffer = ls0xx_get_framebuffer,
	.set_brightness = ls0xx_set_brightness,
	.set_contrast = ls0xx_set_contrast,
	.get_capabilities = ls0xx_get_capabilities,
	.set_pixel_format = ls0xx_set_pixel_format,
	.set_orientation = ls0xx_set_orientation,
};

DEVICE_DT_INST_DEFINE(0, ls0xx_init, NULL,
		      &ls0xx_driver, &ls0xx_config,
		      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,
		      &ls0xx_driver_api);
