# tea5767_kernel_driver
This is a Linux I2C Platform Device Driver for cheap TEA5767 Radio Tuner modules you can find on Amazon:
https://www.amazon.com/TEA5767-Stereo-FM-Radio-Telescopic/dp/B01J3UOUE4/ref=sr_1_4?keywords=tea5767&qid=1579200160&sr=8-4

![Image of TEA5767 FM Radio Tuner Module](https://raw.githubusercontent.com/mozcelikors/tea5767_kernel_driver/master/module-image.jpg)

This driver is not to be confused with Philips TEA5767 chip driver, which is already available at 
https://github.com/spotify/linux/blob/master/drivers/media/common/tuners/tea5767.c

## Adding to Your System
### Adding to Kernel Makefile
Add to your misc driver Makefile as such:
```
diff --git a/drivers/misc/Makefile b/drivers/misc/Makefile
index 16d1c85cfced..8d7fb682f0b9 100644
--- a/drivers/misc/Makefile
+++ b/drivers/misc/Makefile
@@ -59,3 +59,4 @@ obj-$(CONFIG_ASPEED_LPC_SNOOP)	+= aspeed-lpc-snoop.o
 obj-$(CONFIG_PCI_ENDPOINT_TEST)	+= pci_endpoint_test.o
 obj-$(CONFIG_OCXL)		        += ocxl/
 obj-$(CONFIG_MISC_RTSX)		+= cardreader/
+obj-y                                  += tea5767_fm_tuner.o
```
### Adding to Devicetree
Add to your devicetree as such:
```
   diff --git a/arch/arm/boot/dts/bcm2711-rpi-4-b.dts b/arch/arm/boot/dts/bcm2711-rpi-4-b.dts
   index 18c0f9599d3a..bff7c0830af3 100644
   --- a/arch/arm/boot/dts/bcm2711-rpi-4-b.dts
   +++ b/arch/arm/boot/dts/bcm2711-rpi-4-b.dts
   @@ -314,6 +314,12 @@
    	pinctrl-names = "default";
    	pinctrl-0 = <&i2c1_pins>;
    	clock-frequency = <100000>;
   +        status = "okay";
   +
   +        tea5767@60 {
   +            compatible = "mozcelikors,tea5767";
   +            reg = <0x60>;
   +        };
    };
```

## Communicating with the Driver
Load i2c_dev to be able to communicate using sysfs.
```
modprobe i2c_dev
```

### Mute
```
echo "1" > /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/mute
```

### Unmute
```
echo "0" > /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/mute
```

### Check Mute Status
```
cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/mute
```

### Get Current Station Frequency
```
cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/station
```

### Get Available Stations
```
cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/stationlist
```

### Tune into a Station (Frequency in kHZ)
```
echo 97200 > /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/station
```

