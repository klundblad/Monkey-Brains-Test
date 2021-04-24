/*
 * Library to control the AD7616 in Serial Mode
 * Created by Katharine Lundblad on November 19, 2020
 */
#include "SerialPort.h"

SerialPortClass SerialPort;

SPIClass vspi(VSPI);

static const int spiClk = 1000000; // 1 MHz

void ad7616_reset() {
  pinMode(VSPI_SDI, OUTPUT);
  pinMode(VSPI_SDOA, INPUT); 
  pinMode(VSPI_RESET, OUTPUT);
  pinMode(VSPI_CONVST, OUTPUT);
  pinMode(VSPI_BUSY, INPUT);
  pinMode(VSPI_SCLK, OUTPUT);
  pinMode(VSPI_SS, OUTPUT);
  digitalWrite(VSPI_RESET, HIGH);
  digitalWrite(VSPI_SS, HIGH); 
  // digitalWrite(VSPI_BUSY, LOW); 
  digitalWrite(VSPI_CONVST, LOW); 
  digitalWrite(VSPI_RESET, LOW); 
  delay(1000);
  digitalWrite(VSPI_RESET, HIGH);
  delay(1000);
}

void SerialPortClass::init() {
  /* initiate SPI communication */
  vspi.begin();
  /* use SPI mode 3 (VSPI) */
  vspi.setDataMode(SPI_MODE2);
  /* allow the LDOs to power up */
  delay(10);
}

void SerialPortClass::reset() {
  /* sending at least 64 high bits returns ADC to default state */
  for (int i = 0; i < 8; i++) {
    vspi.transfer(0xFF);
  }
  /* allow the LDOs to power up */
  delay(10);
}

void SerialPortClass::sync() {
  /* toggle the chip select */
  digitalWrite(SS, HIGH);
  delay(10);
  digitalWrite(SS, LOW);
  /* allow the LDOs to power up */
  delay(10);
}

/**
 * SPI read from device.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success
 */
int ad7616_read(uint8_t reg_addr, uint16_t *reg_data) {
  vspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE2));
  digitalWrite(VSPI_SS, LOW); //pull SS slow to prep other end for transfer
  uint8_t buf[2];

  buf[0] = 0x00 | ((reg_addr & 0x3F) << 1);
  buf[1] = 0x00;
  /* send the desired amount of bytes */
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = vspi.transfer(buf[i]);
  }
  digitalWrite(VSPI_SS, HIGH);
  vspi.endTransaction();
  vspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE2));
  digitalWrite(VSPI_SS, LOW);

  /* SDOA containing Reg1 Data and Reg2 data not accessed till second read */
  buf[0] = 0x00 | ((reg_addr & 0x3F) << 1);
  buf[1] = 0x00; 
  /* send the desired amount of bytes */
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = vspi.transfer(buf[i]);
  }
  *reg_data = (((uint16_t)buf[0] & 0x01) << 8) | buf[1];
    
  digitalWrite(VSPI_SS, HIGH); //pull ss high to signify end of data transfer
  //delay(1);
  vspi.endTransaction();
  
  return 0; 
}

int ad7616_write(uint8_t reg_addr, uint16_t reg_data) {
  vspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE2));
  digitalWrite(VSPI_SS, LOW); // pull SS slow to prep other end for transfer
  
  uint8_t buf[2];
  
  buf[0] = 0x80 | ((reg_addr & 0x3F) << 1) | ((reg_data & 0x100) >> 8);
  buf[1] = (reg_data & 0xFF);
  /* send the desired amount of bytes */
  for (int i = 0; i < sizeof(buf); i++) {
    vspi.transfer(buf[i]); 
  }

  digitalWrite(VSPI_SS, HIGH); //pull ss high to signify end of data transfer
  vspi.endTransaction();
  return 0; 
}

/**
 * Read from device using a mask.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success
 */
int ad7616_read_mask(uint8_t reg_addr, uint16_t mask, uint16_t *data) {
  uint16_t reg_data;
  int ret;
  ret = ad7616_read(reg_addr, &reg_data);
  *data = (reg_data & mask);

  return ret;
}


/**
 * SPI write to device using a mask.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success
 */
int ad7616_write_mask(uint8_t reg_addr, uint16_t mask, uint16_t data) {
  uint16_t reg_data;
  int ret;

  ret = ad7616_read(reg_addr, &reg_data);
  reg_data &= ~mask;
  reg_data |= data;
  ret = ad7616_write(reg_addr, reg_data);
  
  return ret;
}

/*
 * Setting the channel to enable digital readout
 * 
 * @param channel the desired channel in which the channel select 
 * register can select to read and perform a conversion from 
 */
int ad7616_set_channel(byte channel) {
  uint8_t mask;
  uint8_t data;

  if (channel <= AD7616_VA7) {
    data = channel - AD7616_VA0; 
    mask = 0x0F;
  } else if (channel <= AD7616_VB7) {
    data = (channel - AD7616_VB0) << 4; 
    mask = 0xF0;
  } else {
    data = (channel - AD7616_VB0);
    data = data | data << 4; 
    mask = 0xFF;
  }
  return ad7616_write_mask(AD7616_REG_CHANNEL, mask, data);
}

/*
 * Performs a conversion in order to stream data via SPI
 * 
 * @param *readVarA a pointer to store the desired amount of 
 * bytes in order to perform the first SPI transfer
 * @param *readVarB a pointer to store the desired amount of 
 * bytes in order to perform the second SPI transfer
 */
int ad7616_conversion(uint16_t *readVarA, uint16_t *readVarB) {
  uint8_t buf[2];
  
  digitalWrite(VSPI_CONVST, HIGH); 
  int busyStatus = digitalRead(VSPI_BUSY); 
  digitalWrite(VSPI_CONVST, LOW); 
  busyStatus = digitalRead(VSPI_BUSY); 
  
  while (busyStatus != 0) { // waiting for VSPI_BUSY to go low
    busyStatus = digitalRead(VSPI_BUSY); 
    /* while waiting for busy to go low, SPI transfer is performed */
  }
  
  vspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE2));
  digitalWrite(VSPI_SS, LOW); 
  /* SDOA containing Reg1 Data and Reg2 data not accessed till second read */
  buf[0] = 0x00;
  buf[1] = 0x00;
  /* send the desired amount of bytes */
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = vspi.transfer(buf[i]);
  }
  *readVarA = ((uint16_t)buf[0] << 8) | buf[1];
  
  buf[0] = 0x00;
  buf[1] = 0x00;
  /* send the desired amount of bytes */
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = vspi.transfer(buf[i]);
  }
  *readVarB = ((uint16_t)buf[0] << 8) | buf[1];
  digitalWrite(VSPI_SS, HIGH); 
  vspi.endTransaction();
}
