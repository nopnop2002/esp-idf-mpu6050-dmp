// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// EFM32 stub port by Nicolas Baldeck <nicolas@pioupiou.fr>
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2015-01-02 - Initial release


/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2015 Jeff Rowberg, Nicolas Baldeck

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"

#include "I2Cdev.h"

#define I2C_NUM I2C_NUM_0
#define I2C_TICKS_TO_WAIT 100 // Maximum ticks to wait before issuing a timeout.

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); /*assert(0 && #x);*/} } while(0);

static i2c_master_bus_handle_t bus_handle;

/** Default constructor.
 */
I2Cdev::I2Cdev() {
}

/** Initialize i2c master driver
 */
void I2Cdev::initialize() {
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.i2c_port = I2C_NUM;
    i2c_mst_config.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
    i2c_mst_config.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
    i2c_mst_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
}

/** Add I2C master BUS device.
 * @param devAddr I2C slave device address
 * @param clkSpeed I2C SCL line frequency
 * @return I2C slave device handle
 */
i2c_master_dev_handle_t I2Cdev::addDevice(uint16_t devAddr, uint32_t clkSpeed) {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = devAddr;
    dev_cfg.scl_speed_hz = clkSpeed;

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    return dev_handle;
}

/** Default timeout value for read operations.
 */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

/** Scan I2C device.
 * @param target Desired I2C slave device address
 * @return Desired I2C slave device address was found or not
 */
bool I2Cdev::scanDevice(uint16_t target) {
    bool result = false;
    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(bus_handle, address, I2CDEV_DEFAULT_READ_TIMEOUT);
            if (ret == ESP_OK) {
                printf("%02x ", address);
                if (address == target) result = true;
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    return result;
}

/** Read a single bit from an 8-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout, void *wireObj) {
    uint8_t b;
    uint8_t count = readByte(devHandle, regAddr, &b, timeout, wireObj);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBitW(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout, void *wireObj) {
    uint16_t b;
    uint8_t count = readWord(devHandle, regAddr, &b, timeout, wireObj);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devHandle, regAddr, &b, timeout, wireObj)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t I2Cdev::readBitsW(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout, void *wireObj) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devHandle, regAddr, &w, timeout, wireObj)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t *data, uint16_t timeout, void *wireObj) {
    return readBytes(devHandle, regAddr, 1, data, timeout, wireObj);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t I2Cdev::readBytes(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj) {
    uint8_t out_buf[1];
    out_buf[0] = regAddr;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(devHandle, out_buf, sizeof(out_buf), data, length, I2C_TICKS_TO_WAIT));
    return length;
}

/** Write single word to a 16-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint16_t data, void *wireObj){
    uint8_t data1[] = {(uint8_t)(data>>8), (uint8_t)(data & 0xff)};
    writeBytes(devHandle, regAddr, 2, data1, wireObj);
    return true;
}

/** Write multiple words to a 16-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWords(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t length, uint16_t *data, void *wireObj) {
    for (int _index=0;_index<length;_index++) {
        uint8_t _regAddr = regAddr + (_index * 2);
	    uint8_t data1[] = {(uint8_t)(data[_index]>>8), (uint8_t)(data[_index] & 0xff)};
	    writeBytes(devHandle, _regAddr, 2, data1, wireObj);
    }
	return true;
}

/** write a single bit in an 8-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t bitNum, uint8_t data, void *wireObj) {
    uint8_t b;
    readByte(devHandle, regAddr, &b, I2Cdev::readTimeout, wireObj);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devHandle, regAddr, b, wireObj);
}

/** write a single bit in a 16-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitW(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t bitNum, uint16_t data, void *wireObj) {
    uint16_t w;
    readWord(devHandle, regAddr, &w, I2Cdev::readTimeout, wireObj);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devHandle, regAddr, w, wireObj);
}


/** Write multiple bits in an 8-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data, void *wireObj) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b = 0;
    if (readByte(devHandle, regAddr, &b, I2Cdev::readTimeout, wireObj) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devHandle, regAddr, b, wireObj);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitsW(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data, void *wireObj) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devHandle, regAddr, &w, I2Cdev::readTimeout, wireObj) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devHandle, regAddr, w, wireObj);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t data, void *wireObj) {
    uint8_t out_buf[2];
    out_buf[0] = regAddr;
    out_buf[1] = data;
    ESP_ERROR_CHECK(i2c_master_transmit(devHandle, out_buf, 2, I2C_TICKS_TO_WAIT));
    return true;
}

/** Write single byte to an 8-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register address to write to
 * @param length Number of bytes to write
 * @param data Array of bytes to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t length, uint8_t *data, void *wireObj){
    uint8_t *out_buf;
    out_buf = (uint8_t *)malloc(length+1);
    if (out_buf == NULL) {
        ESP_LOGE(__FUNCTION__, "malloc fail");
        return false;
    }
    out_buf[0] = regAddr;
    for(int i=0;i<length;i++) out_buf[i+1] = data[i];
    ESP_ERROR_CHECK(i2c_master_transmit(devHandle, out_buf, length+1, I2C_TICKS_TO_WAIT));
    free(out_buf);
    return true;
}


/** Read single word from a 16-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readWord(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint16_t *data, uint16_t timeout, void *wireObj){
    uint8_t msb[2] = {0,0};
    readBytes(devHandle, regAddr, 2, msb, timeout, wireObj);
    *data = (int16_t)((msb[0] << 8) | msb[1]);
    return 0;
}

/** Read multiple words from a 16-bit device register.
 * @param devHandle I2C slave device handle
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2Cdev::readWords(i2c_master_dev_handle_t devHandle, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout, void *wireObj) {
    uint8_t msb[2] = {0,0};
    for (int _index=0;_index<length;_index++) {
        uint8_t _regAddr = regAddr + (_index * 2);
	    readBytes(devHandle, _regAddr, 2, msb, timeout, wireObj);
   	    data[_index] = (int16_t)((msb[0] << 8) | msb[1]);
	}

	return length;
}
