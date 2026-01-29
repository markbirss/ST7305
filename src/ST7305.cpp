#include "ST7305.h"

ST7305::ST7305(int16_t w, int16_t h, SPIClass *spi, int8_t cs_pin, int8_t dc_pin, 
               int8_t rst_pin, int8_t te_pin) : 
    Adafruit_GFX(w, h),
    _spi(spi),
    _cs_pin(cs_pin),
    _dc_pin(dc_pin),
    _rst_pin(rst_pin),
    _te_pin(te_pin),    rotation(0) {  // Initialize rotation to 
    
    buffer = (uint8_t *)malloc(400 * 22);
    temp_buffer = (uint8_t *)malloc(200 * 18 * 3);
}

bool ST7305::begin() {
    // Initialize pins
    pinMode(_cs_pin, OUTPUT);
    pinMode(_dc_pin, OUTPUT);
    pinMode(_rst_pin, OUTPUT);
    if (_te_pin >= 0) {
        pinMode(_te_pin, INPUT);
    }
    
    digitalWrite(_cs_pin, HIGH);
    
    // Hardware reset
    digitalWrite(_rst_pin, HIGH);
    delay(50);
    digitalWrite(_rst_pin, LOW);
    delay(100);
    digitalWrite(_rst_pin, HIGH);
    
    // Initialize display
    initDisplay();
    
    clearDisplay();
    return true;
}

void ST7305::sendCommand(uint8_t command) {
    digitalWrite(_dc_pin, LOW);  // Command mode
    digitalWrite(_cs_pin, LOW);
    _spi->transfer(command);
    digitalWrite(_cs_pin, HIGH);
}

void ST7305::sendData(uint8_t data) {
    digitalWrite(_dc_pin, HIGH);  // Data mode
    digitalWrite(_cs_pin, LOW);
    _spi->transfer(data);
    digitalWrite(_cs_pin, HIGH);
}

void ST7305::sendData(uint8_t *data, size_t len) {
    digitalWrite(_dc_pin, HIGH);  // Data mode
    digitalWrite(_cs_pin, LOW);
    _spi->transfer(data, len);
    digitalWrite(_cs_pin, HIGH);
}

void ST7305::initDisplay() {
    sendCommand(0xD6); sendData(0x17); sendData(0x02);  // NVM Load Control
  
    sendCommand(0xD1); sendData(0x01);  // Booster Enable
  
    sendCommand(0xC0); sendData(0x12); sendData(0x0A);  // Gate Voltage Setting
    
    sendCommand(0xC1);  // VSHP Setting (4.8V)
    sendData(115); sendData(0x3E); sendData(0x3C); sendData(0x3C);
    
    sendCommand(0xC2);  // VSLP Setting (0.98V)
    sendData(0); sendData(0x21); sendData(0x23); sendData(0x23);
    
    sendCommand(0xC4);  // VSHN Setting (-3.6V)
    sendData(50); sendData(0x5C); sendData(0x5A); sendData(0x5A);
    
    sendCommand(0xC5);  // VSLN Setting (0.22V)
    sendData(50); sendData(0x35); sendData(0x37); sendData(0x37);
    
    sendCommand(0xB2); sendData(0x12);  // Frame Rate Control
    
    // Update Period Gate EQ Control in HPM
    sendCommand(0xB3);
    uint8_t b3_data[] = {0xE5, 0xF6, 0x17, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x71};
    sendData(b3_data, sizeof(b3_data));
    
    // Update Period Gate EQ Control in LPM
    sendCommand(0xB4);
    uint8_t b4_data[] = {0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45};
    sendData(b4_data, sizeof(b4_data));
    
    sendCommand(0x62); // Gate Timing Control
    uint8_t g62_data[] = {0x32, 0x03, 0x1F};
    sendData(g62_data, sizeof(g62_data));
    
    sendCommand(0xB7); sendData(0x13);  // Source EQ Enable
    sendCommand(0xB0); sendData(0x64);  // Gate Line Setting: 400 line
    
    sendCommand(0x11);  // Sleep out
    delay(120);
    
    sendCommand(0xC9); sendData(0x00);  // Source Voltage Select
    sendCommand(0x36); sendData(0x48);  // Memory Data Access Control
    sendCommand(0x3A); sendData(0x11);  // Data Format Select
    sendCommand(0xB9); sendData(0x20);  // Gamma Mode Setting
    sendCommand(0xB8); sendData(0x29);  // Panel Setting
    
    sendCommand(0x2A); sendData(0x05); sendData(0x36);  // Column Address Setting
    sendCommand(0x2B); sendData(0x00); sendData(0xC7);  // Row Address Setting
    
    if (_te_pin >= 0) {
        sendCommand(0x35); sendData(0x00);  // TE
    }
    
    sendCommand(0xD0); sendData(0xFF);  // Auto power down
    sendCommand(0x38);  // HPM:high Power Mode ON
    sendCommand(0x29);  // Display on

    sendCommand(0x20);  // Display Inversion Off
    sendCommand(0xBB);  // Enable Clear RAM
    sendData(0x4F);  // CLR=0 ; Enable Clear RAM,clear RAM to 0
    delay(10);

    sendCommand(0x2A); // Column Address Setting
    sendData(0x05);
    sendData(0x36);

    sendCommand(0x2B); // Row Address Setting 
    sendData(0x00);
    sendData(0xC7);

}

void ST7305::convertBuffer() {
    uint16_t k = 0;
    for (uint16_t i = 0; i < 400; i += 2) {
        // Convert 2 columns
        for (uint16_t j = 0; j < 22; j += 3) {
            for (uint8_t y = 0; y < 3; y++) {
                uint8_t b1 = buffer[(j + y) * 400 + i];
                uint8_t b2 = buffer[(j + y) * 400 + i + 1];
                
                // First 4 bits
                uint8_t mix = 0;
                mix |= ((b1 & 0x01) << 7);
                mix |= ((b2 & 0x01) << 6);
                mix |= ((b1 & 0x02) << 4);
                mix |= ((b2 & 0x02) << 3);
                mix |= ((b1 & 0x04) << 1);
                mix |= ((b2 & 0x04) << 0);
                mix |= ((b1 & 0x08) >> 2);
                mix |= ((b2 & 0x08) >> 3);
                temp_buffer[k++] = mix;
                
                // Second 4 bits
                b1 >>= 4;
                b2 >>= 4;
                mix = 0;
                mix |= ((b1 & 0x01) << 7);
                mix |= ((b2 & 0x01) << 6);
                mix |= ((b1 & 0x02) << 4);
                mix |= ((b2 & 0x02) << 3);
                mix |= ((b1 & 0x04) << 1);
                mix |= ((b2 & 0x04) << 0);
                mix |= ((b1 & 0x08) >> 2);
                mix |= ((b2 & 0x08) >> 3);
                temp_buffer[k++] = mix;
            }
        }
    }
}

void ST7305::display() {
    convertBuffer();
    
    // Set display window
    uint8_t caset[] = {0x17, 0x17 + 18 - 1};
    uint8_t raset[] = {0x00, 0x00 + 200 - 1};
    
    sendCommand(0x2A);
    sendData(caset, sizeof(caset));
    
    sendCommand(0x2B);
    sendData(raset, sizeof(raset));
    
    sendCommand(0x2C);
    sendData(temp_buffer, 200 * 18 * 3);
}

void ST7305::clearDisplay() {
    memset(buffer, 0, 400 * 22);
}

void ST7305::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= width() || y < 0 || y >= height()) {
        return;
    }

    int16_t new_x, new_y;
    
    switch (rotation) {
        case 1:  // 90 degrees
            new_x = height() - y - 1;
            new_y = x;
            break;
        case 2:  // 180 degrees
            new_x = width() - x - 1;
            new_y = height() - y - 1;
            break;
        case 3:  // 270 degrees
            new_x = y;
            new_y = width() - x - 1;
            break;
        default: // 0 degrees
            new_x = x;
            new_y = y;
            break;
    }
    
    // Calculate byte position and bit position within byte
    uint16_t byte_idx = (new_y >> 3) * 400 + new_x;
    uint8_t bit_pos = new_y & 0x07;
    
    if (color) {
        buffer[byte_idx] |= (1 << bit_pos);
    } else {
        buffer[byte_idx] &= ~(1 << bit_pos);
    }
}
void ST7305::setRotation(uint8_t m) {
    rotation = m % 4;  // Ensure rotation is within 0-3
}
