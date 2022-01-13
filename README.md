# htu21d-esp32-driver
A C language driver for ESP32 platform to interface with HTU21D digital temperature/humidity sensor

This driver allows you dynamically select I2C prot and SDA/SCL pin.

Driver features:
- Initialisation with dynamic selection I2C port and SDA/SCL pins
- Check sensor presence
- Sensor software reset
- Set sensor resolutions both for Temperature and Humidity measuring
- Check chuuent sensor resolution
- Turn ON and OFF sensor's internal heater
- Check if sensor's internal heater is on
- Read temperature
- Read relative humidity with option temperature compensation

This driver uses adaptive delays for measuring temperature and humidity based in current sensor resolution (as per documentation)
