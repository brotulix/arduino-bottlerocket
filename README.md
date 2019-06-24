# Overview
This is a project that aims to create a bottle rocket (water rocket) with good flight characteristics, a soft and safe descent, and lots of fun for at least some of the participants (it's a family project; the younger team members *may* have a shorter attention span and higher demand for instant gratification).

The project has been running a few years, with Easter and summer vacations naturally seeing the brunt of work. Early work was done using the old (hard) Norwegian 1.5L soda bottles, but has now moved to the new (soft) PET soda bottles.




# Features



## Current
* Rocket body with payload capsule, inspired by design principles of a near eastern space nation of somewhat recent history (CCCP).
* Soft-nose loosely adhering to lax HSE requirements (head of a badminton "ball" attached to the cap of payload capsule).
* Manual application of pressurized air (bicycle pump) allowing tight control of at least *one* launch parameter.
* Pressure-capable plug (from a simple bottle rocket project).
* Sturdy, well-proven release mechanism allowing a (rather) controlled launch (PVC pipes and zip-ties).
* A flight recording camera that loses its data on hard landings (eg. all but tree-top landings; high-G intolerant SD card connection?).



## Future
* Parachute deployment (with emergency triggering).
* Arming ("Remove before flight" tag).
* Accelerometer for launch and landing detection and quantification of thrust parameters.
* Barometer for apex detection (parachute deployment, plus *how high did we go?*).
* Radiotelemetry (433MHz).
* Locator beacon (audiovisual, optionally GPS).
* A flight recording camera that doesn't lose its data (*solder* an SD card to that thing).



# Justification
The latest iteration of bottle rocket aerodynamics now means the rocket falls rather fast back towards ground, and thus cars and soft skulls needs to be protected. This can easily be achieved using a parachute, we just need to deploy it at the right time (just after trajectory apex).



# Known issues
Earlier tests have shown that a simple parachute ejector using a curved PET section bent the wrong way as a spring, and another bent the right way as parachute cover to be released through servo motor and a rubber band can be tricky: The parachute got jammed between rocket body and spring and failed to fully deploy.



# Initial solution proposal
Earlier tests have shown that it is indeed possible to detect the apex using sign inversion of barometric delta (when the delta changes from negative to positive), which will indicate that the rocket has begun its descent. After arming and a sustained negative delta pressure, it is also fairly safe to assume apex has been reached when delta pressure reaches zero (absolute magnitude).

A new solution for parachute deployment is advised: A simple tube containing the chute itself and a captive spring (with end cap same-ish diameter as tube ID), plus another end cap (preferrably captive; could be attached to parachute string) that is held in place by a pin to be extracted by servo or linear actuator (the head drive motor from a CD-ROM drive is suggested as light-weight linear actuator).

If the rocket is, as with latest model, painted white, an audiovisual beacon can be achieved using a simple superbright LED lighting the payload compartment from within. A assault alarm buzzer/siren can be utilized for audible assist. Upon receiving a keyword over telemetry link, the beacon mode should disengage to avoid angry neighbours and hearing problems.

Telemetry can be achieved using Bluetooth, but range is suspected to be poor and may lead to loss of data and loss of emergency parachute triggering capability. An alternative can be a generic 433 MHz RF link with unknown range and power draw. Another alternative can be 2.4 GHz RF link, with same unknown parameters.

Important air-to-ground telemetry: ambient pressure, acceleration vector, battery voltage, RF signal strength, state machine information.
Important groud-to-air commands: state machine manipulation (for debugging and emergency control), emergency parachute activation, locator beacon control (both on and off).

Earlier tests have shown that magnetic arming is possible by detecting changes in magnetometer flux magnitude. In combination with arming through RF commands, this may be very efficient, but a light-sensitive approach may be just as viable, if possibly somewhat more error prone. Benefit of using magnetometer is that a RBF tag can be soft-captive. A combination of accelerometer and light/magnetometer should provide a very reliable input.





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
`#define STATE_GROUND_IDLE 0`

This is the default power-on state.
When entering this state (such as upon power-on), the memory is cleared.
In this state, only an RF link command will transition to on-pad idle state.
Most power-conservative mode in that no sensors are read, nothing is reported through RF link.
Listen for commands only.
Ignore RBF ("Remove Before Flight") presence.



### Ground, on-pad idle state
`#define STATE_GROUND_IDLE_ON_PAD 1`

Whenever power is applied, enter this state. The system should be power conservative. A simple, occational reporting of (ground-level) averaged air pressure, gravitational vector and battery voltage. Command input must be accepted through RF link. This is the time to configure system parameters such as:
* Sea level pressure of the moment
* Height of launch pad (default launch site is approx. 7.6m ASL according to norgeskart.no).
* Number of readings to average over.
* Panic time-out.

An optical or magnetic switch could be employed to arm the rocket for launch ("Remove before flight"), or a command could be sent through RF link to transition to armed state.



### Ground, armed state
`#define STATE_GROUND_ARMED 2`

In armed state, the rocket is simply waiting to leave the ground, and must therefore keep a constant watch on the accelerometer to detect changes in acceleration vector. Again, a magnet or optical link could be employed to make it clear that the rocket has left the launch pad, transitioning to the airborne outbound state.



### Airborne, outbound state
`#define STATE_AIRBORNE_OUTBOUND 3`

Now the air pressure should be kept under constant surveillance. All values should be reported back to ground control, or possibly a slight averaging could be employed to reduce time spent in TX.
When the air pressure derivative becomes positive, transition to airborne deployment state.
Two possible backup conditions for transition to airborne deployment state:
* Ground control command.
* A simple flight-timer; after a certain time spent in airborne outbound state a transition **will** occur. Current flight tests indicate time spent airborne is around 5 seconds, so a timeout of 3.5 seconds may allow for at least a mildly decelerated landing in case of other trigger failures.



### Airborne, deployment state
`#define STATE_AIRBORNE_DEPLOYMENT 4`

Simply trigger the parachute deployment, then transition to airborne inbound state.



### Airborne, inbound state
`#define STATE_AIRBORNE_INBOUND 5`

Start looking for a spike in acceleration vector magnitude to indicate a potential touchdown, at which a transition to ground-state recovery should occur.



### Ground, recovery state
`#define STATE_GROUND_RECOVERY 6`

Enable audiovisual beacon, and if a GPS is present, start reporting position.
Enable RF beacon mode by reporting RSSI value for any "ping" received (fox-hunting)?
Whenever magnet/optical switch is engaged, or a command through RF link is received, transition to post-launch data-dump state.



### Ground, data-dump state
`#define STATE_GROUND_DATA_DUMP 7`

Similar to idle state in that sensors are not read, and no values reported. Only wait for a command to start dumping all recorded data via RF link or transition to ground idle state.



### State machine diagram
Insert diagram here.





# Hardware/Software tests



## Arduino
Starting out with an Arduino Nano V3.0 board (6.0 g, ATMega328P), since it lends itself well to breadboard prototyping. I also have an Arduino-esque board marked *Deek-Robot* which is well suited for the final product (2.4 g, ATMega328P).



### Programming Arduino board
Programming the Arduino Nano board proves to be a little tricky as it doesn't seem to work directly from Arduino, and thus neither from Visual Studio Code's Arduino plugin.
The current solution is to use an AVR Dragon and program the Nano via ISP, using the following command from the git repo root (eg. `~/source/arduino-bottlerocket/`):
`~/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/bin/avrdude -C ~/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/etc/avrdude.conf -v -pm328p -c dragon_isp -Uflash:w:build/botroc.ino.with_bootloader.hex:i`



### Power draw
*Measure power draw in all states*.



## RF Link
### Transfer rate
### Range
### RSSI (beacon mode)
### Power draw



## Sensor board
A single GY-80 board can be used, as it has the following sensors:
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



### Power draw
*Measure power draw*.

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




# Communication


## Protocol
Some form of communication protocol must be in place to send control messages and receive status messages from the vessel.



### Status messages
Some status messages are given, such as
* Accelerometer reading
* Pressure reading
* Temperature reading
* State machine state
* GPS data (optional)

During debugging, it would be nice to know a little about what the state machine is doing, so a debug statement would also be handy.

Status reporting could follow a standard format, such as:
`Sa` followed at relevant frequencies by `Abb.bbb`, `Bcc.ccc`, `Mdd.ddd`, `Ree.eee` and `Tff.fff` for accelerometer, barometer, magnetometer, RSSI and thermometer, respectively. GPS, if implemented, could be reported with `G1gg.ggggggG2hh.hhhhhhG3ii.iii` (lat, lon, height) at no more than 1 Hz. LDR, if implemented, could be reported with `Ljj.jjj`.

Suggested reporting rates:

* `GROUND_IDLE` states:
  - Accelerometer: 2 Hz
  - Magnetometer: 2 Hz
  - Barometer: 2 Hz
  - Thermometer: 0.2 Hz
  - GPS: 0.2 Hz
  - RSSI: 1 Hz
  - Battery voltage: 0.2 Hz

* `GROUND_ARMED` state:
  - Accelerometer: 33 Hz
  - Barometer: 33 Hz
  - Magnetometer: 2 Hz
  - Thermometer: 2 Hz
  - GPS: 1 Hz
  - RSSI: 1 Hz
  - Battery voltage: 2 Hz

* `AIRBORNE_OUTBOUND` and `_INBOUND` states:
  - Accelerometer: 100 Hz
  - Barometer: 100 Hz
  - Magnetometer: None
  - Thermometer: 5 Hz
  - GPS: 1 Hz
  - RSSI: 1 Hz

* `AIRBORNE_DEPLOYMENT` state:
  - None

* `GROUND_RECOVERY` state:
  - GPS: 1 Hz
  - Magnetometer: 2 Hz
  - RSSI: 10 Hz
  - Battery voltage: 1 Hz



### Control messages
A few important control messages are given:

* State machine transition: `Sn`, where `n` is the state level to apply (see above).

* Configuration: `CX`, where `X` is one of the following:
  - `G`: Ground pressure: `Gnnnn.nn`.
  
  - `H`: Launch pad height: `Hnnn.n`.
  
  - `A`: Averaging: `AX:y`, where `X` is `A`, `B`, `M` for accelerometer, barometer, magnetometer, respectively, and `y` indicates how many samples to average over, in a single hex digit (0-F). Default is 9. Maximum depends on available memory.
  
  - `TXy.y`: Timeouts, where `X` is either
    - `O`: Outbound, or
    - `I`: Inbound
    and `y.y` is a timeout in seconds (0.0 to 9.9).

  - `L`: Limits: `LXXyyyyy.yyyy`. `XX` indicates which limit to set:
    - `AO`: accelerometer outbound (launch)
    - `AI`: accelerometer inbound (landing)
    - `B`: barometer delta (launch and landing)
    - `M`: magnetometer delta (arm and recovery)

* Ping: `P`

Every command ends with newline (`\n`).

The maximum command line length is thus `CLAI9999.9999\n` (14 characters). A command buffer of 32 characters is reserved. An interrupt function should perhaps handle incoming data on serial line to make sure successive commands are not dropped/corrupted, and incoming commands are serviced as soon as a newline is encountered.

State machine transition commands will be accepted at **any** time.

Configuration will only occur during `IDLE` states (`STATE_GROUND_IDLE` and `STATE_GROUND_IDLE_ON_PAD`).

Ping command will prompt a Pong reply. Mainly intended for RSSI measurements during recovery phase.





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
***Warning***: Here be dragons, errors and bad, bad numbers.

## Energy ##

Energy needed to move our estimated 250 g loaded rocket to a height of, say, 30 meters:

<!--
Create equations here:
https://www.codecogs.com/latex/eqneditor.php
Then save as .svg.
GitHub doesn't cooperate very well with directly embedded images with url as such:
https://latex.codecogs.com/svg.latex?\Large&space;
-->

Potential and kinetic energy:

- <img src="doc/equations/Eq_001.svg" title="Ep = Ek" />

- <img src="doc/equations/Eq_002.svg" title="mgh = (mv^2)/2"/>

Potential energy:

- <img src="doc/equations/Eq_003.svg" title="Ep = mgh = 0.25 * 9.81 * 30 [kg * m/(s^2) * m]"/>

- <img src="doc/equations/Eq_004.svg" title="Ep = 73.575 [kg * m/(s^2) * m]"/>

- <img src="doc/equations/Eq_005.svg" title="Ep = 73.575 [Nm]"/>

- <img src="doc/equations/Eq_006.svg" title="Ep = 73.575 [J]"/>


Force:

- <img src="doc/equations/Eq_007.svg" title="F = ma => a = F/m"/>

<!-- *All wrong*
- <img src="abcdefga = \frac{183.22}{0.25} [\frac{N}{kg}]" title="a = 183.22 / 0.25 [N/kg]"/>

- <img src="abcdefga = 732.88 []" title="a = 732.88 [m/s^2]"/>
-->


Velocity:

- <img src="doc/equations/Eq_008.svg" title="v = a*t"/>

- <img src="doc/equations/Eq_009.svg" title="v = ???"/>


Kinetic energy:

- <img src="doc/equations/Eq_010.svg" title="Ek = 0.5 * 0.25 * v^2"/>

- <img src="doc/equations/Eq_011.svg" title="Ek = ???"/>



## Pressure in vessel ##

- <img src="doc/equations/Eq_012.svg" title="5 bar = 5.09858 kg/cm^2"/>
 
- <img src="doc/equations/Eq_013.svg" title="= 500k [Pa]"/>

- <img src="doc/equations/Eq_014.svg" title="= 500kN/m^2"/>

- <img src="doc/equations/Eq_015.svg" title="= 500k * 0.01 * 0.01 [N/cm^2]"/>

- <img src="doc/equations/Eq_016.svg" title="= 50 [N/cm^2]"/>


## Bottle opening ##

Diameter: 21.6 mm

Area: <img src="doc/equations/Eq_017.svg" title="pi*((2.16cm)/2)^2) = 3.6644 cm^2" />


## Initial force ##

- <img src="doc/equations/Eq_018.svg" title="F = 50 * 3.6644 [N/cm^2 * cm^2 = N]" />

- <img src="doc/equations/Eq_019.svg" title="F = 183.22 [N]" />

- <img src="doc/equations/Eq_020.svg" title="(kgf = ~= 18 [kg])" />



# Proposals
Some proposals for extended features:
* Grease the release mechanism for a potentially easier launch triggering.
* Booster stage utilizing elastic luggage bands. Uncertain if we'll gain anything, or if it'll *only* complicate things.
* Payload padding (plastic foam).
* Compressed air parachute deployment using an inflated water balloon and solenoid valve. Con: Balloon takes up space, solenoid is probably heavy.
* In case of noisy UART over 433 MHz: Add a few characters that signals start of a command, eg. `<SP><SP><SP>S6<LF>` to enter `STATE_GROUND_RECOVERY`.
* *More to come*...


