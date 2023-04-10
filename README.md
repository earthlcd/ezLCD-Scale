# ezLCD Load Cell Scale Demonstration Project
The following project demonstrates how to turn an ezLCD product into a scale and is a good jumping off point for your own applicaiton.  The application monitors a load cell to detect the weight of an attached load.  The weight of the load is displayed on the screen as well as graphed vs time.  The max load is logged so that the scale can be used for pull testing.  A tare button is provided to tare the scale.  A clear button is provided to clear the graph as well as the max weight readings.
![image](https://user-images.githubusercontent.com/198251/230446212-652fb365-034e-48d2-9957-a07fc63e6923.png)
![image](https://user-images.githubusercontent.com/198251/230446279-e6f0c82c-591b-4b8b-958b-fa490a113ad3.png)


## Compatible Products
Currently this project has only been tested with the ezLCD-5035 product.  It may be ported to other products in the future.

## Demonstrates
- Touch Screen Button Handling
- Displaying BMP Images
- Displaying True Type Fonts
- Plotting data
- Serial Communications in Lua
- I2C Communications in Lua

## Requirements
- [EarthLCD ezLCD-5035-RT 3.5"] (https://earthlcd.com/products/ezlcd-5x)
- [SparkFun Qwiic Scale - NAU7802] (https://www.sparkfun.com/products/15242) (or equivilant)
- [Adafruit Strain Gauge Load Cell - 4 Wires - 20Kg](https://www.adafruit.com/product/4543) (or equivilant)

## Wiring
### ezLCD-5035-RT 3.5" to SparkFun Qwiic Scale
- J6-PIN-2 SDA to SDA
- J6-PIN-18 SCL to SCL
- JP1-V33 to 3V3
- J7-GND to GND
![image](https://user-images.githubusercontent.com/198251/230446453-699fc975-bf5e-4b27-8ee6-d2c23858a368.png)

### SparkFun Qwiic Scale to Load Cell
- GRN to Green Wire
- WHT to White Wire
- BLK to Black Wire
- RED to Red Wire

