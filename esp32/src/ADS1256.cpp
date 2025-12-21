#include "ADS1256.h"

// ---- ADS1256 command & register definitions ----
enum {
    CMD_WAKEUP = 0x00,
    CMD_RDATA = 0x01,
    CMD_RDATAC = 0x03,
    CMD_SDATAC = 0x0F,
    CMD_RREG = 0x10,
    CMD_WREG = 0x50,
    CMD_SELFCAL = 0xF0,
    CMD_RESET = 0xFE
};

enum {
    REG_STATUS = 0x00,
    REG_MUX = 0x01,
    REG_ADCON = 0x02,
    REG_DRATE = 0x03,
};

Ads1256::Ads1256(int csPin, int drdyPin)
    : _cs(csPin), _drdy(drdyPin), _spi(nullptr) {}

bool Ads1256::begin(SPIClass& spi) {
    _spi = &spi;

    pinMode(_cs, OUTPUT);
    pinMode(_drdy, INPUT);
    csHigh();

    // Reset sequence
    sendCommand(CMD_SDATAC);
    delay(2);
    sendCommand(CMD_RESET);
    delay(5);
    sendCommand(CMD_SDATAC);
    delay(5);

    // Configure: 30 kSPS, PGA=1, differential mode
    writeRegister(REG_DRATE, 0xF0);  // 30,000 SPS
    writeRegister(REG_ADCON, 0x00);  // PGA=1, CLKOUT off
    writeRegister(REG_MUX, 0x23);    // AIN2-AIN3 (current channel)

    // Self-calibration
    sendCommand(CMD_SELFCAL);
    unsigned long t0 = millis();
    while (!drdyLow() && (millis() - t0) < 1000) {
        delay(1);
    }

    Serial.println("ADS1256: initialized at 30kSPS");
    return true;
}

void Ads1256::startContinuous(uint8_t ch) {
    if (!_spi) return;

    sendCommand(CMD_SDATAC);
    delayMicroseconds(5);

    // Start on current channel (AIN2-AIN3)
    writeRegister(REG_MUX, 0x23);
    delayMicroseconds(10);

    sendCommand(CMD_RDATAC);
}

void Ads1256::stopContinuous() {
    if (!_spi) return;
    sendCommand(CMD_SDATAC);
}

void Ads1256::switchToVoltageChannel() {
    if (!_spi) return;
    sendCommand(CMD_SDATAC);
    delayMicroseconds(5);
    writeRegister(REG_MUX, 0x01);  // AIN0-AIN1
    delayMicroseconds(5);
    sendCommand(CMD_RDATAC);
}

void Ads1256::switchToCurrentChannel() {
    if (!_spi) return;
    sendCommand(CMD_SDATAC);
    delayMicroseconds(5);
    writeRegister(REG_MUX, 0x23);  // AIN2-AIN3
    delayMicroseconds(5);
    sendCommand(CMD_RDATAC);
}

bool Ads1256::readVoltageFast(float& volts) {
    if (!_spi) return false;
    if (!drdyLow()) return false;

    long raw = readData();

    // ADC scaling
    const float VREF = 2.5f;
    const float FS = 8388607.0f;  // 2^23 - 1
    float v_adc = (float)raw * (VREF / FS);

    // AMC1311 specs: 8.2× gain, 1V offset at zero input
    // Voltage divider: (68k + 10k) / 10k = 7.8
    const float V_OFFSET = 1.0f;
    const float AMC_GAIN = 8.2f;
    const float DIVIDER_RATIO = 7.8f;

    float v_pack = ((v_adc - V_OFFSET) / AMC_GAIN) * DIVIDER_RATIO;

    volts = v_pack;
    return true;
}

bool Ads1256::readCurrentFast(float& amps) {
    if (!_spi) return false;
    if (!drdyLow()) return false;

    long raw = readData();

    // ADC scaling
    const float VREF = 2.5f;
    const float FS = 8388607.0f;  // 2^23 - 1
    float v_adc = (float)raw * (VREF / FS);

    // AMC1311 specs: 8.2× gain, 1V offset at zero current
    // Shunt: 50µΩ = 0.00005Ω
    // Current = (V_adc - 1.0V) / (8.2 × 0.00005)
    const float V_OFFSET = 1.0f;
    const float SHUNT_R = 0.00005f;  // 50µΩ
    const float AMC_GAIN = 8.2f;

    float current = (v_adc - V_OFFSET) / (AMC_GAIN * SHUNT_R);

    amps = current;
    return true;
}

// ---- Low‑level helpers ----

void Ads1256::sendCommand(uint8_t cmd) {
    if (!_spi) return;
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    csLow();
    _spi->transfer(cmd);
    csHigh();
    _spi->endTransaction();
}

void Ads1256::writeRegister(uint8_t reg, uint8_t value) {
    if (!_spi) return;
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    csLow();
    _spi->transfer(CMD_WREG | (reg & 0x0F));
    _spi->transfer(0x00);
    _spi->transfer(value);
    csHigh();
    _spi->endTransaction();
    delayMicroseconds(5);
}

uint8_t Ads1256::readRegister(uint8_t reg) {
    if (!_spi) return 0;
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    csLow();
    _spi->transfer(CMD_RREG | (reg & 0x0F));
    _spi->transfer(0x00);
    delayMicroseconds(5);
    uint8_t v = _spi->transfer(0xFF);
    csHigh();
    _spi->endTransaction();
    return v;
}

long Ads1256::readData() {
    if (!_spi) return 0;
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    csLow();
    uint8_t b0 = _spi->transfer(0xFF);
    uint8_t b1 = _spi->transfer(0xFF);
    uint8_t b2 = _spi->transfer(0xFF);
    csHigh();
    _spi->endTransaction();

    long raw = ((long)b0 << 16) | ((long)b1 << 8) | b2;

    // 24-bit sign extension
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }

    return raw;
}