/**
 * Library to control the AD7616 in Serial Mode 
 * Created by Katharine Lundblad on November 19, 2020
 */
#ifndef SerialPort_h_
#define SerialPort_h_

#include "Arduino.h"
#include "SPI.h"

/* Macros and Constants Definitions */
#define AD7616_REG_UP_IF_TYPE       0x40C
#define AD7616_REG_UP_CTRL          0x440
#define AD7616_REG_UP_CONV_RATE     0x444
#define AD7616_REG_UP_BURST_LENGTH  0x448
#define AD7616_REG_UP_READ_DATA     0x44C
#define AD7616_REG_UP_WRITE_DATA    0x450

/* Define pins */
#define VSPI_SDOA   19
#define VSPI_SDI    23
#define VSPI_SCLK   18
#define VSPI_SS     5
#define VSPI_BUSY   16
#define VSPI_CONVST 4
#define VSPI_RESET  15


/* AD7616_REG_UP_CTRL */
#define AD7616_CTRL_RESETN        (1 << 0)
#define AD7616_CTRL_CNVST_EN      (1 << 1)

#define AD7616_REG_CONFIG              0x02
#define AD7616_REG_CHANNEL             0x03
#define AD7616_REG_INPUT_RANGE_A1      0x04
#define AD7616_REG_INPUT_RANGE_A2      0x05
#define AD7616_REG_INPUT_RANGE_B1      0x06
#define AD7616_REG_INPUT_RANGE_B2      0x07
#define AD7616_REG_SEQUENCER_STACK(x) (0x20 + (x))

/* AD7616_REG_CONFIG */
#define AD7616_SDEF             (1 << 7)
#define AD7616_BURSTEN          (1 << 6)
#define AD7616_SEQEN            (1 << 5)
#define AD7616_OS(x)            (((x) & 0x7) << 2)
#define AD7616_STATUSEN         (1 << 1)
#define AD7616_CRCEN            (1 << 0)

/* AD7616_REG_INPUT_RANGE */
#define AD7616_INPUT_RANGE(ch, x)   (((x) & 0x3) << (((ch) & 0x3) * 2))
#define AD7616_SET_CHANNEL(ch)      ((0x3) << (((ch) & 0x3) * 2))

/* AD7616_REG_SEQUENCER_STACK(x) */
#define AD7616_ADDR(x)          (((x) & 0x7F) << 9)
#define AD7616_SSREN            (1 << 8)
#define AD7616_BSEL(x)          (((x) & 0xF) << 4)
#define AD7616_ASEL(x)          (((x) & 0xF) << 0)

/* AD7616_REG_STATUS */
#define AD7616_STATUS_A(x)        (((x) & 0xF) << 12)
#define AD7616_STATUS_B(x)        (((x) & 0xF) << 8)
#define AD7616_STATUS_CRC(x)      (((x) & 0xFF) << 0)

/* AD7616_REG_UP_CTRL */
#define AD7616_CTRL_RESETN        (1 << 0)
#define AD7616_CTRL_CNVST_EN      (1 << 1)

/*enable or disable debug */
#define DEBUG_ENABLED 0

/* delay for reading and writing registers */
#define READ_WRITE_DELAY 1

/* registers */
typedef enum {
  
} adc7616_register_t; 

/* TYPES DECLARATIONS */
typedef enum {
  AD7616_SW,
  AD7616_HW,
} ad7616_mode;

typedef enum {
  AD7616_SERIAL,
  AD7616_PARALLEL,
} ad7616_interface;

enum rngsel{
  h_rngsel = 0,
  h1_rngsel = 1
};

typedef enum {
  AD7616_VA0,
  AD7616_VA1,
  AD7616_VA2,
  AD7616_VA3,
  AD7616_VA4,
  AD7616_VA5,
  AD7616_VA6,
  AD7616_VA7,
  AD7616_VB0,
  AD7616_VB1,
  AD7616_VB2,
  AD7616_VB3,
  AD7616_VB4,
  AD7616_VB5,
  AD7616_VB6,
  AD7616_VB7,
  AD7616_VCC,
  AD7616_ALDO,
  AD7616_RESERVED,
  AD7616_SELF_TEST,
} ad7616_ch;

typedef enum {
  AD7616_2V5 = 1,
  AD7616_5V  = 2,
  AD7616_10V = 3,
} ad7616_range;

typedef enum {
  AD7616_OSR_0,
  AD7616_OSR_2,
  AD7616_OSR_4,
  AD7616_OSR_8,
  AD7616_OSR_16,
  AD7616_OSR_32,
  AD7616_OSR_64,
  AD7616_OSR_128,
} ad7616_osr;

typedef struct {
  
  /* GPIO */
  int8_t        gpio_hw_rngsel0;
  int8_t        gpio_hw_rngsel1;
  int8_t        gpio_reset;
  int8_t        gpio_os0;
  int8_t        gpio_os1;
  int8_t        gpio_os2;
  
  /* Device Settings */
  ad7616_interface  interface;
  ad7616_mode       mode;
  ad7616_range      va[8];
  ad7616_range      vb[8];
  ad7616_osr        osr;
} ad7616_dev;

typedef struct {
  /* SPI */
  uint8_t       spi_chip_select;
  uint32_t      spi_device_id;
  
  /* GPIO */
  uint32_t      gpio_device_id;
  int8_t        gpio_hw_rngsel0;
  int8_t        gpio_hw_rngsel1;
  int8_t        gpio_reset;
  int8_t        gpio_os0;
  int8_t        gpio_os1;
  int8_t        gpio_os2;
  
  /* Device Settings */
  ad7616_mode     mode;
  ad7616_range    va[8];
  ad7616_range    vb[8];
  ad7616_osr      osr;
} ad7616_init_param;



/* SPI read from device. */
int ad7616_read(uint8_t reg_addr, uint16_t *reg_data);

/* SPI write to device */
int ad7616_write(uint8_t reg_addr, uint16_t reg_data);

/* SPI read from device using a mask. */
int32_t ad7616_read_mask(uint8_t reg_addr, uint16_t mask, uint16_t *data);

/* SPI write to device using a mask. */
int32_t ad7616_write_mask(uint8_t reg_addr, uint16_t mask, uint16_t data);

/* Perform a full reset of the device. */
void ad7616_reset();

/* Set the analog input range for the selected analog input channel. */
int32_t ad7616_set_range(ad7616_ch ch, ad7616_range range);

/* Set the oversampling ratio. */
int32_t ad7616_set_oversampling_ratio(ad7616_osr osr);

/* Set the specific channel to be converted */
int ad7616_set_channel(byte channel);

/* Convert register reads and writes to streamable data*/
int ad7616_conversion(uint16_t *readVarA, uint16_t *readVarB);

/* Initialize the device. */
int32_t ad7616_setup(ad7616_init_param init_param);

class SerialPortClass {
public:
  /* constructor */
  SerialPortClass() {
    /* ... */ 
  }

  /* Initializes SPI connection with ADC */
  void init();

  /* cancels current transaction to resync the ADC */
  void sync();

  /* resets the ADC registers to the default state */
  void reset();

  void print_byte(byte values);
};

extern SerialPortClass SerialPort;

#endif
