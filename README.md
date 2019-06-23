# Features
* Arming ("Remove before flight")
* Launch detection
* Altimeter (Barometer)
* Accelerometer
* Telemetry (433MHz or Bluetooth)
* Parachute deployment (with emergency triggering)
* Locator beacon (audiovisual)

# Justification
The latest iteration of bottle rocket aerodynamics now means the rocket falls rather fast back towards ground, and thus cars and soft skulls needs to be protected. This can easily be achieved using a parachute, we just need to deploy it at the right time (trajectory apex).

# Known issues
Earlier tests have shown that a simple parachute ejector using a curved PET section bent the wrong way as a spring, and another bent the right way as parachute cover to be released through servo motor and a rubber band can be tricky: The parachute got jammed between rocket body and spring and failed to fully deploy.

# Initial solution proposal
Earlier tests have shown that it is indeed possible to detect the apex using sign inversion of barometric delta, which will indicate that the rocket has begun its descent. After arming and a sustained negative delta pressure, it is also fairly safe to assume apex has been reached when delta pressure reaches zero.

A new solution for parachute deployment is adviced: A simple tube containing the chute itself and a captive spring (with end cap same-ish diameter as tube ID), plus another end cap (preferrably captive; could be attached to parachute string) that is held in place by a pin to be extracted by servo or linear actuator (the head driver from a CD-ROM drive is suggested as light-weight linear actuator).

If the rocket is, as with latest model, painted white, an audiovisual beacon can be achieved using a simple superbright LED lighting the payload compartment from within. A assault alarm buzzer/siren can be utilized for audible assist. Upon receiving a keyword over telemetry link, the beacon mode should disengage to avoid angry neighbours and hearing problems.

Telemetry can be achieved using Bluetooth, but range is suspected to be poor and may lead to loss of data and loss of emergency parachute triggering capability. An alternative can be a generic 433 MHz RF link with unknown range and power draw. Another alternative can be 2.4 GHz RF link, with same unknown parameters.

Important air-to-ground telemetry: ambient pressure, acceleration vector, battery voltage, RF signal strength, state machine information.
Important groud-to-air commands: state machine manipulation (for debugging and emergency control), emergency parachute activation, locator beacon control (both on and off).

Earlier tests have shown that magnetic arming is possible by detecting changes in magnetometer flux magnitude. In combination with arming through RF commands, this may be very efficient, but a light-sensitive approach may be just as viable, if possibly somewhat more error prone. A combination of accelerometer and light/magnetometer should provide a very reliable input.

# Weights
## Motors
Various motors found around my place:
* MDN3BT3DRA: 19.6 g
* Game console vibrator (including weight): 20.3 g
* Game console vibrator (excluding weight): 18.4 g
* "9g Micro Servo": 11.1 g
* Generic toy motor: 17.1 g
* Unknown (including mount): 21.0 g
* Unknown (excluding mount): 17.9 g

So in conclusion we'll go for the 9g micro servo motor.
Ideally, we'll find out how to make it spin without end stops, but it may work even with its limited travel, depending on our deployment mechanism.

## Electronics
Various electronic components intended for inclusion:
* [Arduino Nano V3.0 (prototyping version)](https://www.dx.com/p/2010805): 6.0 g
* [ATMEGA328P 16MHz Electric Block Module (w/o headers)](https://www.dx.com/p/2021500): 2.4 g
* Barometer, accelerometer, magnetometer, gyro board: 2.5 g
* Assault alarm (PCB + piezo element): 4.8 g
* GPS module w/light-weight antenna: 11.3 g
* 433 MHz RF Transceiver module (PCB): 3.7 g
* 433 MHz RF Transceiver module (PCB+Antenna): 10.8 g
* [Bluetooth transceiver module](https://www.dx.com/p/2011902): 3.5 g
* [2.4 GHz RF Transceiver module](https://www.dx.com/p/2017524): 2.2 g


## Batteries
* 2032 coin cell battery: 2.8 g
* 2032 coin cell holder: 1.8 g
* 18650 Cell (Meco): 45.2 g
* 18650 Cell (LG): 47.1 g
* 9V Battery (Biltema): 47.2 g
* AAA 800 mAh NiMH Battery: 12.8 g
* WFT051721 (LiPo?): 2.7 g
* Hi-Watt Alkaline 12V VR22: 8.0 g
* [1s 550mAh - 50/100C - BETAFPV LiHV 2-Pac](https://www.modellers.no/betafpv/41421/1s-550mah--50-100c--betafpv-lihv-2-pac) : 12.9 g
* [1s 500mAh - Syma X5HW X5HC](https://www.modellers.no/syma/35847/1s--500mah--syma-x5hw-x5hc) : 17 g
* [1s 220mAh - 45C - Gens Ace Tattu](https://www.modellers.no/gens-ace/34720/1s--220mah--45c--gens-ace-tattu-5-pack-e-flite): 5.5 g

## Mechanical
* Experimental parachute: 2.8 g
* Latest rocket body (CCCP, no fuel): 117.5 g
* Coca-Cola 1.75L bottle (empty): 44.1 g
* Solo bottle payload capsule (cut-off): 23.1 g

## Suggested payload weight estimate:
*  10 g - Arduino + Barometer
*   5 g - RF Transceiver
*  10 g - Audiovisual beacon
*  15 g - Battery
*  15 g - Servo
*  15 g - Parachutes
* 150 g - Rocket body with payload capsule
*  30 g - Various mechanical assemblages (eg. parachute deployment)
= 250 g Total

# Equations
Energy needed to move our estimated 250 g rocket to a height of, say, 30 meters:
Ep = Ek
mgh = (mv^2)/2

Ep = mgh = 0.25 * 9.81 * 30 [kg * m/(s^2) * m]
Ep = 73.575 [kg * m/(s^2) * m]
Ep = 73.575 [Nm]
Ep = 73.575 [J]

F = ma
=> a = F/m
a = 183.22 / 0.25 [F/m]
a = 732.88 [m/s^2]

v = a*t
v = 732.88 * X [m/s^2 * s]
v = ???

Ek = 0.5 * 0.25 * v^2
Ek = ???


Pressure in vessel:
5 bar = 5.09858 kg/cm^2
= 500kPa
= 500kN/m^2
= 500k * 0.01 * 0.01 [N/cm^2]
= 50 [N/cm^2]

Bottle opening:
Diameter: 21.6 mm
Area: pi*((2.16cm)/2)^2) = 3.6644 cm^2

Initial force:
F = 50 * 3.6644 [N/cm^2 * cm^2 = N]
F = 183.22 [N]
(kgf = ~= 18 [kg])

# System overview
The launch control/monitoring system likely consists of two parts: The launched vessel, and a ground station.

## Ground station
The ground station is a stationary component, probably a laptop will be most suitable. External components to the laptop includes an antenna and the chosen RF transceiver, and possibly an Arduino with some buttons, eg. to activate emergency parachute deployment and arming the rocket for launch.
Depending on the antenna solution chosen for the rocket payload capsule, a Yagi antenna of a couple elements may suffice, otherwise a short helical antenna pointing directly upwards may be appropriate, or simply a cloverleaf antenna.
The ground station should log all received data (pressure and acceleration, time of launch, time of touchdown, time of parachute deployment and time spent in the various stages of the state machine, plus any commands sent to the vessel). A graph of said parameters could be rendered either real-time or upon touchdown. If a GPS is employed, a map should be shown to allow for easier vessel recovery. A 3D view of the trajectory might be cool as well, if the data is sufficient. Some values for thrust, energy, current height and velocity records, magnitude of touchdown impact.

## The rocket payload system
The rocket payload system should at the very least consist of an Arduino, a pressure sensor, an accelerometer, an RF transceiver, and some means of deploying one or more parachutes at (or shortly after reaching) apex of trajectory.
Optional accessories:
* Flash memory for storing data and settings (I2C or SPI).
* Audio-visual beacon (assault alarm works on 12 V down to at least 3.7 V).
* RBF sensor (magnetometer on same module as baro- and accelerometer, or an LDR).
* GPS (most useful during recovery phase, but could possibly help with trajectory mapping).

The payload control system can be implemented using a state machine. 8 states are suggested below.

### Ground, idle state
This is the default power-on state.
When entering this state (such as upon power-on), the memory is cleared.
In this state, only an RF link command will transition to on-pad idle state.
Most power-conservative mode in that no sensors are read, nothing is reported through RF link.
Listen for commands only.
Ignore RBF ("Remove Before Flight") presence.

### Ground, on-pad idle state
Whenever power is applied, enter this state. The system should be power conservative. A simple, occational reporting of (ground-level) averaged air pressure, gravitational vector and battery voltage. Command input must be accepted through RF link. This is the time to configure system parameters such as:
* Sea level pressure of the moment
* Height of launch pad (default launch site is approx. 7.6m ASL according to norgeskart.no).
* Number of readings to average over.
* Panic time-out.

An optical or magnetic switch could be employed to arm the rocket for launch ("Remove before flight"), or a command could be sent through RF link to transition to armed state.

### Ground, armed state
In armed state, the rocket is simply waiting to leave the ground, and must therefore keep a constant watch on the accelerometer to detect changes in acceleration vector. Again, a magnet or optical link could be employed to make it clear that the rocket has left the launch pad, transitioning to the airborne outbound state.

### Airborne, outbound state
Now the air pressure should be kept under constant surveillance. All values should be reported back to ground control, or possibly a slight averaging could be employed to reduce time spent in TX.
When the air pressure derivative becomes positive, transition to airborne deployment state.
Two possible backup conditions for transition to airborne deployment state:
* Ground control command.
* A simple flight-timer; after a certain time spent in airborne outbound state a transition **will** occur. Current flight tests indicate time spent airborne is around 5 seconds, so a timeout of 3.5 seconds may allow for at least a mildly decelerated landing in case of other trigger failures.

### Airborne, deployment state
Simply trigger the parachute deployment, then transition to airborne inbound state.

### Airborne, inbound state
Start looking for a spike in acceleration vector magnitude to indicate a potential touchdown, at which a transition to ground-state recovery should occur.

### Ground, recovery state
Enable audiovisual beacon, and if a GPS is present, start reporting position.
Enable RF beacon mode by reporting RSSI value for any "ping" received (fox-hunting)?
Whenever magnet/optical switch is engaged, or a command through RF link is received, transition to post-launch data-dump state.

### Ground, data-dump state
Similar to idle state in that sensors are not read, and no values reported. Only wait for a command to start dumping all recorded data via RF link or transition to ground idle state.

### State machine diagram
Insert diagram here.

# Hardware/Software tests
## Arduino
### Programming Arduino board
Programming the Arduino Nano board proves to be a little tricky as it doesn't seem to work directly from Arduino, and thus neither from Visual Studio Code's Arduino plugin.
The current solution is to use an AVR Dragon and program the Nano via ISP, using the following command from the git repo root (eg. `~/source/arduino-bottlerocket/`):
`~/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/bin/avrdude -C ~/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/etc/avrdude.conf -v -pm328p -c dragon_isp -Uflash:w:build/botroc.ino.with_bootloader.hex:i`

### Power draw
Power draw in all states.

## RF Link
### Transfer rate
### Range
### RSSI (beacon mode)
### Power draw

## Note on sensor board
For sensors, a single GY-80 board can be used, as it has the following sensors:
* BMP085 Baro- and thermometer
* ADXL345 Accelerometer
* L3G4200D Gyrometer
* HMC5883L Magnetometer

### Sensor board connections
Pins from the top, as seen with the pin header inserted into bread board and chips visible, with mounting holes to the right:
1) `VCC_IN`
2) `VCC_3.3V`
3) `GND`
4) `SCL` -- I2C Data clock
5) `SDA` -- I2C Data pin
6) `M_DRDY` -- Magnetometer data ready. We may want to use this to avoid waiting for data, but since magnetometer will mostly be used while on the pad anyway, perhaps we can ignore this.
7) `A_INT1` -- Accelerometer data ready. We probably want to use this to avoid waiting for data.
8) `T_INT1` -- Temperature data ready. We probably want to use this to avoid waiting for data.
9) `P_XCLR` -- Barometer reset pin
10) `P_EOC` -- Barometer end-of-conversion pin. We probably want to use this to avoid waiting for data.

Pin 1 and 2 indicates an on-board voltage regulator, but to save some power, we should probably used 3.3 V directly from the Arduino.

So in summary, we probably want to monitor at least three interrupt sources. Maybe we need to connect to a a common interrupt pin with resistors and use one interrupt routine that reads the digital pins to figure out which of the pins triggered interrupt.

### Barometer
#### Power draw
#### Noise
#### Averaging
#### Ascent detection
#### Apex detection
#### Descent detection

### Accelerometer
#### Power draw
#### Noise
#### Averaging
#### Apex detection
#### Launch detection
#### Touchdown detection

### Magnetometer (RBF)
#### Power draw
#### Noise
#### Averaging
#### RBF Detection

## Flash memory
Write only full pages to avoid/minimize data loss.
How many erase/write cycles can be expected?
### Power draw
Idle, erasing/writing, reading.


## GPS
### Power draw
### Accuracy
### Update rate
### Acquisition time

## Audio-visual beacon
### Power draw
Idle and active states, audio and visual.
