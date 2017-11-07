# MCopter

A quadcopter built from scratch.

Currently there is a working PCB for the flight controller + basic firmware.

### Working (kind-of) features--

* Simple PID flight stabilization
* Radio communication via external radio module (both controlling the copter and sending telemtry data to the controller
* Simple remote controller made out of an old RC toy radio + arduino pro mini + nRF24

### Planned

* Designing custom frame and 3D print it
* Using an embedded radio solution (probably nRF24 based)
* Using 9-axis fusion instead of 6-axis to avoid yaw drift.

# The Hardware

![](https://i.imgur.com/9LejJzh.png)

This is version 2.0 of the flight controller PCB.
The PCB is design with 2 layers to keep down the cost, in the future I might redesign it with 4 layers to make it smaller. I'm also using 0805 to allow for easy hand-soldering.

The microcontroller is STM32F410RBT, a powerful 32-bit ARM Cortex M4 with hardware floating point unit and a lot of useful peripherals.

Other major components (sensors):

* ICM-20689; MEMES gyroscope + accelerometer. This is the newer generation of the popular MPU-6050 used in a lot of commercial flight controllers.
* MAG3110; A magnetometer.
* MS5637; A Barometer

The ICM-20689 togheter with the MAG3110 can be used for 9-axis fusion to provide better heading (yaw) controller and avoid drifting, and with the barometer we can also have an height based control instead of throttle.

![](https://i.imgur.com/nqLHzKc.jpg)

* TODO take a better picture...

