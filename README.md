# Run with platform.io
```bash
pio run -t upload
```

# Increase exposure time
... to get a better low light performance and less horizontal lines in the piucture.
Pixel clock needs to be slowed down by setting ov2460 internal clock divider.

```c
int Sipeed_OV2640::ov2640_set_fdiv(int fdiv)
{
    int ret;
    uint8_t reg;
    ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENSOR);

    // set 5 bit pixel clock divider to fdiv
    // f_pixel = XVCLK / (fdiv + 1)
    if (fdiv >= 0) {
        ret |= cambus_readb(_slaveAddr, CLKRC, &reg);
        reg &= ~(0x1F);
        reg |= fdiv & 0x1F;
        ret |= cambus_writeb(_slaveAddr, CLKRC, reg);
    }

    return ret;
}
```

for troubleshooting, read raw exposure time register

```c
int Sipeed_OV2640::ov2640_get_exposure(unsigned *exposure)
{
    int ret;
    uint8_t aec_1510, aec_92, aec_10, reg;
    unsigned tmp;
    ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENSOR);
    ret |= cambus_readb(_slaveAddr, REG04, &aec_10);
    ret |= cambus_readb(_slaveAddr, AEC, &aec_92);
    ret |= cambus_readb(_slaveAddr, REG45, &aec_1510);
    if (ret == 0) {
        tmp  = (aec_1510 & 0x3F) << 10;
        tmp |= aec_92 << 2;
        tmp |= aec_10 & 0x3;
        *exposure = tmp;
    }
    return ret;
}
```

# Observations
  * `ov2640_set_auto_exposure(1, 0)` will enable auto regulated exposure time (limited by pixel clock)
  * `ov2640_set_auto_exposure(0, 10000)` will set exposure time to 10 ms, if pixel clock allows it
  * `ov2640_set_auto_gain(enable, gain, gain_ceil)` auto regulate gain when enable is true, will not go above gain_ceil.
    will use the fixed gain of value `gain` when enable is false.
