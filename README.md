# RaspberryPi-ADC-DAC
Codes to perform Analog to Digital(ADC, using ADS1256 circuit) and Digital to Analog (DAC, using DAC8552) based on Raspberry PI extended with Waveshare High-Precision AD-DA raspberry pi hat/shield.

The provided codes with the hat/shield was not satisfying (IMHO). These codes are cleaner, faster and meant to be used in other projects (as you have a lib `AD-DA-WS-RPI` with proper CMakeLists and Find.cmake to compile it).

## Installation

### BCM2835

You will need BCM2835 library and headers that you can find [here](http://www.airspayce.com/mikem/bcm2835/).
Download the latest version of the library, say `bcm2835-1.xx.tar.gz`, then:

```shell
tar zxvf bcm2835-1.xx.tar.gz
cd bcm2835-1.xx
./configure
make
sudo make check
sudo make install
```

Remarks : tested with version 1.55 and 1.56 (since commit 45ded505c7f1468286e85d9065cf5b9fef636db7, prior commit will not work with versions >=1.56). It looks like it is not compiling with versions < 1.50 because bcm2835_spi_begin() changed its signature. 


### RaspberryPi-ADC-DAC

You will require `git` and `cmake`.

```shell
sudo apt-get update
sudo apt-get install git cmake
```

Then you can clone and build the codes of this repository : 
```shell
git clone https://github.com/AKEOPLUS-boris-bocquet/RaspberryPi-ADC-DAC.git
cd RaspberryPi-ADC-DAC
mkdir build
cd build
cmake ..
make
```

### Enabling SPI

This board communications via [SPI : Serial Peripherical Interface Bus](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus).
Please be sure that you have allowed `SPI` interface on your Raspberry Pi (disable by default).

```shell
sudo raspi-config
```

Go to `5 - Interfacing Options` and then `P4 - SPI` and say `<Yes>` to enable `SPI`. 

## Testing

If you sucessfully installed, you compiled `testAdda`. Before running it, please be sure that you have allowed `SPI` interface. Then correctly plug you hat/shield on your Pi (please do that power off) and put the jumpers on the hat/shield as following : 
* VCC to 5V
* VREF to 5V
* DAC0 to LEDA
* DAC1 to LEDB
* AD0 to ADJ
* AD1 to LDR

Then boot the Pi and go to your build folder. Then run
```shell
sudo ./testAdda
```

In console, current 8 AD input values will be displayed. Also, input value on AD0 is forwarded to DAC0. So if you have correclty placed the jumpers, you can dim the light of the LEDA using the potentiometer knob.

## How it works

* Init the communication and the circuits by calling `ADC_DAC_Init`.
* Get the current values of the inputs (AD0 to AD7) by calling `ADS1256_ReadAdcValues`. Pass the desired channels as an array (e.g. [0,1,2] if you want to read AD0, AD1 and AD2).
* You can convert AD values to microvolts or volts using `ADS1256_AdcArrayToMicroVolts`.
* Write to outputs (DAC0 or DAC1, i.e. Channel_A or Channel_B) by calling `DAC8552_Write` .
* Close / stop the circuits and the communication by calling `ADC_DAC_Close`.

## Code quality

I try to do my best to provide code with a certain level of quality even though C is not my main language. The code is commented and documented. There are some very strict compilation flags, forcing to have zero compiler warnings (at least tried on gcc). I also perform regular cppcheck. But of course all your proposals and pull requests are welcome !

## Donations

This work is definitly NOT to make money on it. My motivations are to provide easy and performant codes for this hat/shield. Of course you may show your enthusiasm by giving a small donation => [![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://paypal.me/borisBocquet?locale.x=fr_FR) 
