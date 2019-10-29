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

***This state is currently not used, using `STATE_GROUND_IDLE` instead.***

Similar to idle state in that sensors are not read, and no values reported. Only wait for a command to start dumping all recorded data via RF link or transition to ground idle state.



### State machine diagram
*Insert diagram here.*





# Hardware/Software tests



## Arduino
Starting out with an Arduino Nano V3.0 board (6.0 g, ATMega328P), since it lends itself well to breadboard prototyping. I also have an Arduino-esque board marked *Deek-Robot* which is well suited for the final product (only 2.4 g, ATMega328P).



### Programming Arduino board
Programming the Arduino Nano board proves to be a little tricky as it doesn't seem to work directly from Arduino, and thus neither from Visual Studio Code's Arduino plugin.
The current solution is to use an AVR Dragon and program the Nano via ISP, using the following command from the git repo root (eg. `~/source/arduino-bottlerocket/`):
`~/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/bin/avrdude -C ~/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/etc/avrdude.conf -v -pm328p -c dragon_isp -Uflash:w:build/botroc.ino.with_bootloader.hex:i`



### Power draw
*Measure power draw in all states*.



## RF Link

### CC110L
The RF link initially chosen for the project is an old-ish 433 MHz module from Deal Extreme, the only markings being `TLC1101_V2.1`. It is based on the Texas Instruments chip [`CC110L`](https://www.ti.com/lit/ds/symlink/cc110l.pdf).

Some of its highlights are:
* Programmable output power, up to +12dBm.
* Programmable data rate from 600 to 600k bps.
* Supports several different modulations: 2- and 4-FSK, GFSK, MSK and OOK.

There is, however, an AtMega48 between us and the CC110L, possibly limiting its functionality somewhat. Other modules using this chip have been found ([here's one example](http://www.yesyes.info/index.php/electronics/rf1100-232-rf-433mhz-transceiver-module/)). It is claimed to require 5V input, and that the sleep mode does not work.

The modules are *plug & play* at 9600 baud and 3.3V.

#### Transfer rate
The board supposedly accepts a simple UART RX/TX connection, making it easy to interface with the Arduino. The 433 MHz is claimed to be able to operate at a baud rates of 4800, 9600 or 19200, using one of 256 selectable channels.

#### Range
At 5V: TBD.

At 3.3V: TBD.

Antenna designs to be evaluated. Clover leaf or helical antennas may be feasible using thin wire taped to the inside of the payload capsule.

Ground control antenna could be a clover leaf or helical for stationary use, and a yagi for direction finding in recovery.

Initial tests using stock antenna indicates about 200 meter LOS range, but also reveals some issues with transmission: some data is transferred successfully, but then only garbled data. Some time later, data is again transferred successfully, before turning garbled again. Some form of checksum bits interfering?

#### RSSI (beacon mode)

#### Power draw
TI's data sheet indicates that the CC110L alone draws about 30mA in +10dBm transmit mode. The board also has an AtMega48 chip.

It can successfully transmit data with 3.3V VCC at minimum distance.

At 3.3V, default settings and 9600 baud, transmitting `\n` or `28857   R       A       57.49   57.54   0\n` over RF yields practically the same results:
* Nominally 20.6mA current draw (97mV/4.7Ohm)
* The RF module changes current draw approximately 2.3ms after end of last character on serial line
* 21,3mA (+0.7/+0.7mA) for 0.9ms
* 37.9mA (+7.3/+6.6mA) for 3.4ms
* 6,4 mA (-12.2/-31.5mA) for 3,7ms
* 11,9mA (-8.7/+5.5mA) for 0.8ms

Setting up the RF module for different power levels, still at 3.3V, yields the following results, average peak-peak measured over 4.7 Ohm resistor for 30 transmissions of `B` at 1 second interval.
* Level `0x00`: 80.8mV
* Level `0x05`: 95.9mV
* Level `0x07`: 136mV
* Any other value: 166mV ~= 10dBm



### nRF24L01
Initial tests with nRF24-modules were unsuccessful as no connection was achieved. New modules have been ordered.

#### Transfer rate
Supposedly up to 2Mbps, but probably not at max range.

#### Range
Supposedly up to 250 meters LOS.

#### RSSI (beacon mode)

#### Power draw
TBD.


### Bluetooth module (HC-06)
#### Transfer rate
Supports at least 115200 Bps.

#### Range
Range up to 70 meters LOS according to GPS test using the integrated bluetooth transceiver of a EliteBook 840 G2.

#### RSSI (beacon mode)

#### Power draw
TBD.




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
The Adafruit library reads barometer pressure values as float. Using `uint16_t` instead may allow us to get enough precision and save some (global) memory by keeping the float value a local variable during sensor input only.

#### Noise
<!--
#### Averaging
*Averaging is identical for all sensors.*
-->

#### Ascent detection
Ascent detection (transition from `STATE_GROUND_ARMED` to `STATE_AIRBORNE_OUTBOUND`) in two ways: barometer magnitude or acceleration magnitude above threshold.

#### Apex detection
Latest implementation will register ground pressure, and subsequently check for any registered pressure lower than this, and continually set the lowest pressure recorded as apex pressure.

#### Descent detection
Basic function of descent detection is to look for an increasing pressure. This could be done by alwas comparing `average` to `prev_average`.

Launch 4 indicated an elevated pressure during thrust that triggered the first version of descent detection.

The next iteration only checks for an increasing pressure if the pressure is less than ground station pressure (recorded at transition from `STATE_GROUND_IDLE_ON_PAD` to `STATE_GROUND_ARMED`). After three instances of a positive pressure difference between apex pressure and current average pressure, the parachute will deploy through state transition from `STATE_AIRBORNE_OUTBOUND` to `STATE_AIRBORNE_DEPLOYMENT`.



<!-- ### Thermometer
#### Noise
#### Averaging
-->

### Accelerometer
#### Noise
<!--#### Averaging-->
<!--#### Apex detection-->
#### Launch detection
A simple check for acceleration magnitude above threshold.

#### Touchdown detection
*Not implemented.*

### Magnetometer (RBF)
#### Noise
There is some noise in the presence of metallic objects and varying somewhat with location, so the threshold value must be watched and tweaked.

<!--#### Averaging-->
#### RBF Detection
A simple threshold detection. Triggering met variable success before the RBF got a  "focusing conduit" for magnetic flux via a nut and bolt. This gave a much more reliable triggering than a "coin" taped to the inside of the payload capsule, and also a visually identifiable spot to attach the RBF.



## Flash memory
Write only full pages to avoid/minimize data loss?

How many erase/write cycles can be expected?

### Power draw
*TBD: Idle, erasing/writing, reading.*


## GPS
The lightest GPS module in my possession at the moment is a (fake?) ublox Neo 6M with a replaced, external chip antenna. It is configurable to higher baud rates, but will seemingly not store its settings.

*GPS is currently not utilized.*

<!--
### Power draw
### Accuracy
### Update rate
### Acquisition time
-->



## Audio-visual beacon

### Power draw
*TBD: Idle and active states, audio and visual.*
Three red/blue flashing LEDs, each drawing about 7 mA at 3.3V, are driven in parallell. The LEDs are driven directly at 5V (has not killed the LEDs yet), but the current draw at 5V has not been recorded.

Buzzer is also driven at 5V, and current draw has not been measured.



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
`Sa` followed at relevant frequencies by `Abb.bbb`, `Bcc.ccc`, `Mdd.ddd`, `Ree.eee` and `Tff.fff` for accelerometer, barometer, magnetometer, RSSI and thermometer, respectively.

GPS, if implemented, could be reported with `G1gg.ggggggG2hh.hhhhhhG3ii.iii` (lat, lon, height) at no more than 1 Hz.

LDR, if implemented, could be reported with `Ljj.jjj`.

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

Implementation does not differentiate reporting rates in different states, but does differentiate on *reporting*:
* Battery voltage is reported periodically in all states.
* `STATE_GROUND_IDLE`: No reporting.
* `STATE_GROUND_ON_PAD`: Magnetometer is reported.
* `STATE_GROUND_ARMED`: Accelerometer and barometer is reported.
* `STATE_AIRBORNE_OUTBOUND`: Accelerometer and barometer is reported.
* `STATE_AIRBORNE_DEPLOYMENT`: Nothing is reported, since this state is only transitional.
* `STATE_AIRBORNE_INBOUND`: Accelerometer and barometer is reported.
* `STATE_GROUND_RECOVERY`: Magnetometer is reported.
* `STATE_GROUND_DATADUMP`: Nothing is reported.

Implemented reporting scheme: `[millis]\tR\t[sensor]\t[average value]\t[diff from last average]\r\n`.

`millis` is simply the result of `millis()`, eg. the uptime of the flight computer.

`sensor` is `A`, `B`, `M` or `V` for accelerometer, barometer, magnetometer and battery voltage, respectively.



### Control messages
A few important control messages are given:

* Configuration: `C`. A single `C` will report the current configuration. Setting configuration options using the following syntax:
```
C       Configuration
|
+ B       Buzzer frequency
|
+ I       Intervals
| + A       Accelerometer
| | + A       Averaging
| | + P       Printing
| | ` R       Read
| |
| + B       Barometer
| | + A       Averaging
| | + P       Printing
| | ` R       Read
| |
| + M       Magnetometer
| | + A       Averaging
| | + P       Printing
| | ` R       Read
| |
| + V       Battery voltage
|   + A       Averaging
|   + P       Printing
|   ` R       Read
|
+ L       Limits
| + A       Accelerometer
| + B       Barometer
| + M       Magnetometer
| ` V       Battery voltage
|
+ S       Servo
| + R       Release
| ` S       Safe
|
` T       Timeouts
  + I       Inbound
  ` O       Outbound
```

Example Configuration report, with field descriptions added in square brackets:
```
Dumping configuration!
2       [interval_read_accelerometer]
50      [interval_read_barometer]
50      [interval_read_magnetometer]
10      [interval_average_accelerometer]
50      [interval_average_barometer]
500     [interval_average_magnetometer]
10      [interval_print_accelerometer]
50      [interval_print_barometer]
500     [interval_print_magnetometer]
5000    [interval_check_battery]
250     [limit_delta_accelerometer]
30      [limit_delta_barometer]
60      [limit_delta_magnetometer]
730     [limit_battery_voltage]
3500    [panic_timeout_outbound]
3000    [panic_timeout_inbound]
4000    [buzzer_frequency]
45      [servo_safe]
0       [servo_release]
```


* Debug: `D`. The function of this command will vary during development. As of 2019-07-20, the effect is a reply in the following format:
```
----- DEBUG -----
Current state is S0
System time is 1197045
Command buffer length: 2
Current Accelerometer value: A: [-8.87, -0.31,  44.72]
45.59
Current Barometer value: B: 101110.00
101110.00
Current Magnetometer value: M: [-1.09, -67.09,  -20.92]
70.28
Current buzzer frequency is 4000 Hz
Peak altitude pressure: -1.00
```



* Ping: `P`. Replies with a `P` of its own. Intended for RSSI measurement.

* State machine transition: `Sn`, where `n` is the state level to apply (see above).



Every command ends with semicolon (`;`). Tried initially to use newline (`\n`), but that proved to be difficult to match.

The maximum command line length is thus `CIAP9999.9999;` (14 characters). A command buffer of 16 characters is reserved for now. An interrupt function should perhaps handle incoming data on serial line to make sure successive commands are not dropped/corrupted, and incoming commands are serviced as soon as a separator is encountered.

State machine transition commands shall be accepted at **any** time.

Configuration will only occur during `IDLE` states (`STATE_GROUND_IDLE` and `STATE_GROUND_IDLE_ON_PAD`).




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
* LoLin NodeMCU V3: 9.9 g
* Geekworm Easy Kit ESP32-C1: 10.9 g



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
* A full 30L bio-degradable bag: 10 g
* Parachute launch tube (K-rør Ø20mm L75mm): 9.1 g
* Parachute deployment mechanism (excl. servo): approx 50 g.



## Suggested payload weight estimate:
*  10 g - Arduino + Barometer
*   5 g - RF Transceiver
*  10 g - Audiovisual beacon
*  ~~15~~ 30 g - Battery
*  15 g - Servo
*  ~~15~~ 3.3 g - Parachute(s)
* ~~150~~ 104 g - Rocket body with payload capsule
*  30 g - Various mechanical assemblages (eg. parachute deployment)
* = ~~250~~ 207.3 g Total

At launch time 2019-07-19, the rocket was weighed in at 283.9 grams (including RBF), somewhat over weight budget. 278.7 g without RBF.

140.7 g total payload (including padding, excluding battery and parachute).





# Mechanical
## Parachute deployment
### Prior experience
An earlier system similar to (insert video URL) was tested. It seemed to work fairly well, but during flight testing, the parachute lodged itself between the parachute hatch and the parachute eject spring.

### Parachute
The previous system used a bio-degradable bag for compostable waste with some success (light, and seemingly strong enough). The bags used, however, meant a rather small parachute, and access to larger bio-degradable trash bags (30L) may allow us to make a larger parachute.

A prototype parachute was made with approximately Ø XX mm and a hole of approximately Ø YY mm in the center. Eight sewing thread cords were attached along the edge of the parachute using tape. 

Later tests demonstrated the need for the thread to be folded back over itself at least one full turn "around" the tape to have sufficient friction to not be torn out from the tape when the parachute deploys.

### Proposed mechanism, Rev. 1
A mechanism made from a compressible spring and a tube containing the parachute to be deployed out through the side of the payload capsule. Strings attached to the front of the spring (towards parachute) will hook onto the arm of a servo motor, holding it coiled until deploymet.

### Prototype, Rev. 1
A quick prototype was attempted, using some Kevlar string, a piece of polycarbonate, a spring and some screws and washers. It proved to be difficult to keep the spring coiled neatly, even inside a tube, and the strings easily got snagged.

### Proposed mechanism, Rev. 2
In the search for a suitable spring, for Rev. 1, a vacuum pump for removal of solder was disassembled; it has a very neat release mechanism and a perfectly sized spring for a OD 20mm *"K-rør"* (ID 17mm).

A piston with a notched rod interacts with a spring-loaded release mechanism (push-button) inspired by a solder vacuum pump. The button can be depressed by the servo motor to eject the parachute from the tube. A clip in another notch near the end of travel prevents the whole piston from escaping the release mechanism.

A 30 liter bio-degradable waste bag just barely fits inside a 75mm long piece of 20mm *"K-rør"*. The vacuum pump spring easily compresses to about half this length, so approximately 45mm is taken as the latching point for the first Rev. 2 prototype.

### Prototype, Rev. 2
The prototype works very well, and the *"9g Micro Servo"* easily triggers the push-button release mechanism.



# Equations
***Warning***: Here be dragons, errors and bad, bad numbers.

## Energy ##

<!--
Create equations here:
https://www.codecogs.com/latex/eqneditor.php
Then save as .svg.
GitHub doesn't cooperate very well with directly embedded images with url as such:
https://latex.codecogs.com/svg.latex?\Large&space;
-->

### Potential and kinetic energy:

- <img src="doc/equations/Eq_001.svg" title="Ep = Ek" />

- <img src="doc/equations/Eq_002.svg" title="mgh = (mv^2)/2"/>

### Potential energy:
#### Initial calculation:
Energy needed to move our estimated 250 g loaded rocket to a height of, say, 30 meters:

- <img src="doc/equations/Eq_003.svg" title="Ep = mgh = 0.25 * 9.81 * 30 [kg * m/(s^2) * m]"/>

- <img src="doc/equations/Eq_004.svg" title="Ep = 73.575 [kg * m/(s^2) * m]"/>

- <img src="doc/equations/Eq_005.svg" title="Ep = 73.575 [Nm]"/>

- <img src="doc/equations/Eq_006.svg" title="Ep = 73.575 [J]"/>

#### Post-fact calculation:
Energy required to move our measured 278 g loaded rocket to a height of 44.54 meters:

- <img src="doc/equations/Eq_023.svg" title="Ep = mgh"/>

- <img src="doc/equations/Eq_024.svg" title="Ep = 0.278 * 9.81 * 44.54 [kg * m/s^2 * m]"/>

- <img src="doc/equations/Eq_025.svg" title="Ep = 121.47 [J]"/>


### Force:

- <img src="doc/equations/Eq_007.svg" title="F = ma => a = F/m"/>

<!-- *All wrong*
- <img src="abcdefga = \frac{183.22}{0.25} [\frac{N}{kg}]" title="a = 183.22 / 0.25 [N/kg]"/>

- <img src="abcdefga = 732.88 []" title="a = 732.88 [m/s^2]"/>
-->


### Velocity:

- <img src="doc/equations/Eq_008.svg" title="v = a*t"/>

- <img src="doc/equations/Eq_009.svg" title="v = ???"/>


### Acceleration
At the release the mass of the rocket is 0.278 kg plus about 0.9 kg of fuel. With 5 bar, the force of `F=ma` should be approximately 183.22 N, so acceleration should be

- <img src="doc/equations/Eq_047.svg" title="F=ma -> a=F/m"/>

<!-- a=\frac{183.22[N]}{0.278[kg]+0.9[kg]}=\frac{183.22[N]}{1.178[kg]}=155.53[\tfrac{m}{s^2}] -->

- <img src="doc/equations/Eq_048.svg" title="a_full=..."/>


- <img src="doc/equations/Eq_049.svg" title="a_empty=..."/>


### Kinetic energy:

- <img src="doc/equations/Eq_010.svg" title="Ek = 0.5 * 0.25 * v^2"/>

- <img src="doc/equations/Eq_011.svg" title="Ek = ???"/>



## Pressure in vessel ##

- <img src="doc/equations/Eq_012.svg" title="5 bar = 5.09858 kg/cm^2"/>
 
- <img src="doc/equations/Eq_013.svg" title="= 500k [Pa]"/>

- <img src="doc/equations/Eq_014.svg" title="= 500kN/m^2"/>

- <img src="doc/equations/Eq_015.svg" title="= 500k * 0.01 * 0.01 [N/cm^2]"/>

- <img src="doc/equations/Eq_016.svg" title="= 50 [N/cm^2]"/>

<!-- V_{eng}^{0} = V_{fuel}+V_{ox}^{0} -->
<!-- Eq_026 -->
- <img src="doc/equations/Eq_026.svg" title="V_{eng}^{0} = V_{fuel}+V_{ox}^{0}"/>

<!-- 1.75[L] = 0.9[L] + 0.85[L] -->
<!-- Eq_027 -->
- <img src="doc/equations/Eq_027.svg" title="1.75[L] = 0.9[L] + 0.85[L]"/>

<!-- V_{eng}^{5}= V_{fuel}+V_{ox}^{5} -->
<!-- Eq_028 -->
- <img src="doc/equations/Eq_028.svg" title="V_{eng}^{5}= V_{fuel}+V_{ox}^{5}"/>

<!-- 5.15[L] = 0.9[L] + 4.25[L] -->
<!-- Eq_029 -->
- <img src="doc/equations/Eq_029.svg" title="5.15[L] = 0.9[L] + 4.25[L]"/>


## Bottle opening ##

Diameter: 21.6 mm

Area: <img src="doc/equations/Eq_017.svg" title="pi*((2.16cm)/2)^2) = 3.6644 cm^2" />


## Initial force ##

- <img src="doc/equations/Eq_018.svg" title="F = 50 * 3.6644 [N/cm^2 * cm^2 = N]" />

- <img src="doc/equations/Eq_019.svg" title="F = 183.22 [N]" />

- <img src="doc/equations/Eq_020.svg" title="(kgf = ~= 18 [kg])" />


## Momentum ##

[Ns]

## Altitude
Altitude based on difference in pressure:

- <img src="doc/equations/Eq_021.svg" title="A = 44330 * (1-(p/p_0)^(1/5.255)"/>
- <img src="doc/equations/Eq_022.svg" title="A ~= 44330 * (1-(p/p_0)^(0.1903)"/>

According to the Wikipedia article on [Atmospheric Pressure vs. Altitude](https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation), "At low altitudes above sea level, the pressure decreases by about 1.2 kPa for every 100 metres", or 12 Pascal per meter.

<!-- A = \frac{\left ( p_0 - p \right ) } {12} -->
- <img src="doc/equations/Eq_038.svg" title="A = (p_0 - p) / 12" />

<!--
A \approx 44330 \cdot \left ( 1 - \left ( \frac{p}{p_0}\right)^{0.1903} \right )
-->

### Launch 3
<!--
```
>> 44330 * (1-(100796.63/101118.75)^0.1903)

ans =  26.908
```
-->
<!-- A_{3} \approx 44330 \cdot \left ( 1 - \left ( \frac{100796.63}{101118.75}\right)^{0.1903} \right ) -->
<!-- Eq_036 -->
<!-- A_{3} \approx 26.908 [m] -->
<!-- Eq_037 -->
- <img src="doc/equations/Eq_036.svg" title="A_{3} \approx 44330 \cdot \left ( 1 - \left ( \frac{100796.63}{101118.75}\right)^{0.1903} \right )" />

- <img src="doc/equations/Eq_037.svg" title="A_{3} \approx 26.908 [m]" />

Simple calculation:

<!-- A = \frac{\left ( 101118.75 - 100796.63 \right ) } {12} -->
- <img src="doc/equations/Eq_039.svg" title="A_{3} = (101118.75 - 100796.63) / 12" />

- <img src="doc/equations/Eq_040.svg" title="A_{3} = 26.843 [m]" />

### Launch 4
<!--
```
>> 44330 * (1-(100819.75/101120.5)^0.1903)

ans =  25.120
```
-->
<!-- A_{4} \approx 44330 \cdot \left ( 1 - \left ( \frac{100819.75}{101120.5}\right)^{0.1903} \right ) -->
<!-- Eq_030 -->
<!-- A_{4} \approx 25.120 [m] -->
<!-- Eq_031 -->
- <img src="doc/equations/Eq_030.svg" title="A_{4} \approx 44330 \cdot \left ( 1 - \left ( \frac{100819.75}{101120.5}\right)^{0.1903} \right )" />

- <img src="doc/equations/Eq_031.svg" title="A_{4} \approx 25.120 [m]" />

Simple calculation:

<!-- A = \frac{\left ( 101118.75 - 100796.63 \right ) } {12} -->
- <img src="doc/equations/Eq_041.svg" title="A_{4} = (101120.5 - 100819.75) / 12" />

- <img src="doc/equations/Eq_042.svg" title="A_{4} = 25.062 [m]" />

### Launch 5
<!--
```
>> 44330 * (1-(100563.5/101080.0)^0.1903)

ans =  43.196
```
-->
<!-- A_{5} \approx 44330 \cdot \left ( 1 - \left ( \frac{100563.5}{101080.0}\right)^{0.1903} \right ) -->
<!-- Eq_032 -->
<!-- A_{5} \approx 43.196 [m] -->
<!-- Eq_033 -->
- <img src="doc/equations/Eq_032.svg" title="A_{5} \approx 44330 \cdot \left ( 1 - \left ( \frac{100563.5}{101080.0}\right)^{0.1903} \right )" />

- <img src="doc/equations/Eq_033.svg" title="A_{5} \approx 43.196 [m]" />

Simple calculation:

<!-- A = \frac{\left ( 101118.75 - 100796.63 \right ) } {12} -->
- <img src="doc/equations/Eq_043.svg" title="A_{5} = (101080.0 - 100563.5) / 12" />

- <img src="doc/equations/Eq_044.svg" title="A_{5} = 43.042 [m]" />

### Launch 6
<!--
```
>> 44330 * (1-(100541.5/101074.0)^0.1903)

ans =  44.539
```
-->
<!-- A_{6} \approx 44330 \cdot \left ( 1 - \left ( \frac{100541.5}{101074.0}\right)^{0.1903} \right ) -->
<!-- Eq_034 -->
<!-- A_{6} \approx 44.539 [m] -->
<!-- Eq_035 -->
- <img src="doc/equations/Eq_034.svg" title="A_{6} \approx 44330 \cdot \left ( 1 - \left ( \frac{100541.5}{101074.0}\right)^{0.1903} \right )" />

- <img src="doc/equations/Eq_035.svg" title="A_{6} \approx 44.539 [m]" />

Simple calculation:

<!-- A = \frac{\left ( 101118.75 - 100796.63 \right ) } {12} -->
- <img src="doc/equations/Eq_045.svg" title="A_{6} = (101074.0 - 100541.5) / 12" />

- <img src="doc/equations/Eq_046.svg" title="A_{6} = 44.375 [m]" />


# Proposals
Some proposals for extended features:
* Grease the release mechanism for a potentially easier launch triggering.
* Booster stage utilizing elastic luggage bands. Uncertain if we'll gain anything, or if it'll *only* complicate things.
* ~~Payload padding (plastic foam).~~ Implemented.
* Compressed air parachute deployment using an inflated water balloon and solenoid valve. Cons: Balloon takes up space, solenoid is probably heavy.
* ~~In case of noisy UART over 433 MHz: Add a few characters that signals start of a command, eg. `<SP><SP><SP>S6<LF>` to enter `STATE_GROUND_RECOVERY`.~~
* ***Someone on the internet*** used soapy water: Why? Test the effect of a few different concentrations (differently coloured; RP-2, RP-3, ...) versus normal water (clear; RP-1).
* In the same theme, try warm (not too warm, the PET bottle may shrink and/or weaken!) and/or cold water?
* Install cords to help arm the parachute deployment mechanism (when the rocket is bumped a little, the piston will not fully depress, preventing the mechanism from locking).
* Estimate outbound velocity based on increase in pressure, and thus acceleration, compare force to force excerted by the pressurized air?
* *More to come*...

# Launches
## 2019-07-19
### Launch 1
Pressure: Approx. 5 bar.

Fuel: 0.9 liter RP-1.

### Launch 2
Pressure: Approx. 3.5 bar.

Fuel: 0.9 liter RP-1.

## 2019-07-20
### Launch 3
Pressure: Approx 3.5 bar.

Fuel: 0.9 liter RP-1.

Achieved altitude: 26.9 meters.

### Launch 4
Pressure: Approx 4.5 bar.

Fuel: 0.9 liter RP-1.

Achieved altitude: 25.1 meters.

### Launch 5
Pressure: Approx. 5 bar.

Fuel: 0.9 liter RP-1.

Achieved altitude: 43.1 meters.

### Launch 6
Pressure: Approx. 5 bar.

Fuel: 0.9 liter RP-1.

Achieved altitude: 44.5 meters.

