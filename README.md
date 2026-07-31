# reddron

This functions as storage for the reddron project. The reddron project name makes a
little more sense if you know the Norwegian word "redde", or "to save". We are trying
to make a drone that finds a person who has fallen into water and drops flotation to
them before a rescue crew can reach the scene.

The design ambition is heavy-weather maritime search and rescue in Finnmark. This
build is a proof of concept flown in benign conditions, but the mission computer is
built to be hardware-agnostic so the same module can move to a larger airframe later.

The reason I picked this problem is that it is one many hear about. Several of my friends
work on maintaing fish farms and king crab boats along the coast, and going overboard is not a
hypothetical, it is a thing that happens. The weather up here turns fast: a workable
morning can become a gale by afternoon, and a deck that was safe an hour ago is suddenly
not. This is water cold enough that survival is measured in minutes rather than hours. 
A person in the sea in Finnmark in winter does not have time to wait for a boat to turn around 
and find them, and the weather thatputs people in the water is often the same weather that makes 
them hard to spot.

What surprised me when I started looking was how little exists for this specific case.
There is plenty of drone hardware built for calm-weather inspection and photography,
and there are full-scale SAR aircraft, but the gap in between is wide: something small
enough for a working vessel to carry, tough enough to launch in the conditions that
caused the emergency, and autonomous enough to find a person and drop flotation without
someone piloting it manually while the crew is busy. That gap is what reddron is aimed
at.

## Project structure

The system is split into three pillars:

- **Pillar A: custom airframe and CFD.** Deferred entirely. A scratch-designed airframe
  was out of scope for the timeframe, so this build uses a commercial mark4 7-inch frame.
  Cutting this was a deliberate decision to keep the remaining two pillars achievable at a level worth presenting.
- **Pillar B: thermal target localization.** An MLX90640 thermal array, a TFmini-S
  LiDAR and the flight controller's EKF attitude are fused to convert a hot pixel into
  a GPS coordinate on the ground. This is the core of the project.
- **Pillar C: ballistic payload drop.** A servo-released payload with a lead
  calculation that accounts for forward velocity, drop height and crosswind drag.

The mission computer is an ESP32 that runs Pillars B and C and talks to the flight
controller over MAVLink. Nothing in it is specific to this airframe, which is the
point: the module is the deliverable, the quadcopter is the test platform.

## Bill of materials

**Flight platform**

| Part | Component |
|---|---|
| Frame | Tarot 7" MK4 carbon fiber |
| Motors | 4x iFlight XING X2806.5 1300KV (6S) |
| FC/ESC | MicoAir H743 V2 + 70A AM32 AIO stack |
| GPS/RemoteID | NewBeeDrone BeeID Pro M10 (u-blox M10, IST8310 compass) |
| Radio | RadioMaster Pocket ELRS LBT + XR3 Nano (CRSF, UART6) |
| Props | HQProp 7x3.5x3 |
| Batteries | 2x CNHL 6S 1300mAh 130C, 2x Align HIGHPOWER 6S 2200mAh 45C |
| Charger | ISDT 608AC (50W AC / 200W DC) |

**Mission computer**

| Part | Component |
|---|---|
| MCU | Seeit ESP32-DEV-30P |
| Thermal | Pimoroni PIM365 MLX90640, 55 degree FOV (I2C, 0x33) |
| LiDAR | Benewake TFmini-S, UART |
| Drop servo | DFRobot SER0006 on FC PWM5 |
| Attitude source | EKF quaternion from ArduPilot over MAVLink |

**Printed parts:** eSUN PETG Black for sensor pod, drop cradle and GPS mast. Landing
skids in PETG with TPU feet.

Firmware target is ArduPilot `MicoAir743v2`. Mission computer firmware is built in
VS Code with PlatformIO.

## Development log

**July 2026**

- **July 13:** The idea had been brewing for a while, but I needed to get started on
  actual hardware instead of leaving it as a thought. I used Claude to work
  through component selection. The suggested parts were mostly good, but I had to go
  elsewhere for the flight controller. The first suggestion was the SpeedyBee F405 V5, it has
  no verified ArduPilot target listed on ardupilot.org despite what the norwegian retailer claims.
  Lesson taken forward: verify every board directly against ardupilot.org, not against
  a product page.
- **July 14:** Settled on the MicoAir H743 V2 with a 70A AM32 AIO stack. The
  `MicoAir743v2` target is verified and supported from ArduPilot v4.6.0 onward. The
  H743 also gives enough spare UARTs for the ESP32 MAVLink link, which the F405 boards
  were tight on.
- **July 17:** Scoped the project into three pillars and cut Pillar A. A custom airframe
  with CFD validation would have consumed the whole timeline and left the sensor work
  unfinished. Better to have two pillars that work than three that don,t.
- **July 18:** Completed the A1/A3 competency course and exam. Discovered during the
  exam that dropping objects from a drone is prohibited in the open category across all
  EASA member states, Norway included. This forced a redesign of the test plan: the drop
  mechanism gets validated on a static bench rig and indoors, while outdoor flights test
  the localization pipeline without a payload. Registration goes through flydrone.no,
  airspace via NinoxDrone2, and all flying stays outside Alta airport's 5 km control zone.
- **July 19:** Because of yesterday's regulatory setback I thought through
  alternatives and read the full EASA open category rules. Indoor flight falls outside
  the regulatory framework entirely, which opened up real testing options. I work at a
  gym next to an indoor pool, and the local indoor sports hall Finnmarkshallen has the ceiling
  height for testing. Between the two I have venues for both the drop mechanism
  and the localization pipeline without needing a payload drop exemption. I also sent a
  note to the Norwegian aviation authority to double-check my interpretation of the
  indoor exemption (I prefer to not get put onto a list)
- **July 20:** Component procurement locked. I secured that the spec is confirmed compatible.
  Final delivery expected around August 6, limited by shipping from central europe to Finnmark.
- **July 27:** Figured out how the printed parts need to relate to each other
  structurally. The sensor pod, landing gear and drop cradle all bolt to the carbon
  frame independently — none of them should carry load for the others. If landing
  impact travels through the pod it shifts sensor alignment and quietly corrupts the
  Pillar B transform. If it travels through the cradle it risks deforming the release
  geometry. The landing skids are designed to be the part that breaks first in a crash,
  so everything else survives.
- **July 31:** Repository initialized. Firmware skeleton started in PlatformIO.

**August 2026**

- **August 1:** Pillar B transform chain implemented and validated in Python. The chain
  runs pixel to angle, angle to body-frame direction vector, body to world frame by
  quaternion rotation, ground intersection using LiDAR slant range, and finally geodetic
  coordinates. The camera is tilted 12.5 degrees forward from nadir so that cruise pitch
  cancels the tilt and the boresight points straight down in forward flight. Ground
  coverage at 5 m altitude is roughly 3.3 m fore-aft by 5.2 m cross-track.

  Monte Carlo results at 5 m: mean localization error 0.148 m, with attitude error
  contributing about 0.128 m RMS and altitude error only about 0.013 m RMS. The useful
  conclusion is that LiDAR precision is not the bottleneck near nadir. Attitude
  estimation dominates the error budget, which means vibration isolation on the flight
  controller matters more to accuracy than a better rangefinder would.
- **August 2:**

## Planned

- Pillar C ballistic lead equation with crosswind drag
- Synthetic thermal frame generator and centroid detection
- MLX90640 intrinsic calibration and axis sign verification once hardware arrives
- Static bench rig: servo actuation delay and empirical drag coefficients
- Indoor hover and drop testing
- Outdoor pipeline testing without payload, A3 category

## Notes on error and uncertainty

Numbers quoted from Monte Carlo simulation are simulation results, not flight results.
Measured localization error against a surveyed ground target is the intended validation
and has not been performed yet.