# BMF055-flight-controller

I wanted to tank Bosch Sensortec for supporting this project with hardware and knowledge.
The idea about this project is to build a flight controller based on a single chip: the BMF055. This system-in-package contains all necessary components: a 3 axis gyroscope, 3 axis accelerometer and a 3 axis magnetometer. These are connected to an Atmel SAMD20 Cortex M0+ micro controller.
This software gives the BMF055 the ability to control a quadrocopter by just adding a receiver and 4 external motors and speed controllers.

Addiotional information regarding development state, features and hardware setup will be added to the wiki of this repository.

##Idea:
The idea was to set up a flight controller with just a single chip, but the problem was which software it should be based on. Cleanflight is made for the STMFXXX series and lacks some hardware abstraction. It has really gerat features, so the first idea was to either add support for the BMF055 or to port it. But it would have been a lot of work the get it even running. So I chose to take a really simple core and to replace the parts regarding the hardware. Nanowii mw21 came to my mind, because it still has great performance on the 8 bit controllers, flies great and is just simple. So this was the ideal plattform to experiment with the BMF055 and to get to know it.

This software is based on the following components:

### [BMF055 Example Project - Data Stream ](https://gallery.atmel.com/Products/Details/f00f3e26-f14d-40ce-9a74-be14f0db1ff2)
The Atme Studio is based on the data stream example for the BMF055. It was really helpful for initial testing and for getting to know the system. This example sets up the SAMD20 configures the sensors and periaodically reads the sensor values and prints them to a USART interface. The part for setting up, interfacing and reading the sensors was used for this firmware.

### [Cleanflight](https://github.com/cleanflight/cleanflight)
Since I am also involved in Cleanflight development some of those ideas found their way into this firmware. The number of improvements and new features is just awesome. A big thanks to Dominic Clifton and all the contributors.

### [Betaflight](https://github.com/borisbstyle/betaflight/)
This is the experimenting area of Cleanflight maintained by Boris B and in the past tons of experiments were made here. Lots of them werde considered as stable and merged to Cleanflight. This is just a great motor of innovation with all the experiments made there, so I will also add some of the ideas in here.

### Atmel ASF and SAM-BA
The bootloader, which can be flashed to the BMF055 is the Atmel SAM-BA, a special adaption of it can be found in this repository.
The low level hardware interfacing is done using the Atmel Software Framework.

### [Multiwii](https://code.google.com/archive/p/multiwii/)
The core and idea for this firmware is based on the "old" MW architecture which was originally developed for small Atmel 8-bit micro controllers and sensors from the Wii console.

### Multiwii 2.1 nanowii adaptions by FelixNiessen/[flyduino](http://flyduino.net/)
This MW version is a special adaption by Felix, it contains only necessary parts and is focussed on stunt flying. It also involves new features like the oneshot protocoll for higher ESC refresh rates.
