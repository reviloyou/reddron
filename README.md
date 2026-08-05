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
and find them, and the weather that gets people in the water is often the same weather that makes 
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
  [correction from later date, it does have support however i was looking at the wrong flight controller, 
  explained in log for 05.08]
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

- **August 1-2:** Pillar B implemented and verified in software, five files, all tests
  passing:
  `pillar_b.py` — the coordinate transform chain, six steps: pixel to angle, angle to
  body-frame direction vector, body to world frame by quaternion rotation, ground
  intersection using LiDAR slant range, and finally geodetic coordinates.
  `test_pillar_b.py` — six check groups, all passing, plus a 20,000-run Monte Carlo.
  `synthetic_frames.py` — generates fake 32x24 thermal frames with a Gaussian hot spot.
  `centroid_detector.py` — finds the target by weighted centroid, subpixel precision.
  `test_full_pipeline.py` — 100 synthetic frames through detection and transform end
  to end: 100/100 detections, median centroid error 0.072 m.

  Decisions locked:
  Camera orientation: 55 degree FOV cross-track, 35 degrees fore-aft. The wide axis
  goes across the flight path to maximize search width per pass.
  Forward tilt 12.5 degrees from nadir, verified in code: at 12.5 degrees nose-down
  cruise pitch the boresight offset is 0.0 m.
  Ground footprint at 5 m altitude: 3.33 m fore-aft (0.44 m behind to 2.89 m ahead),
  5.20 m cross-track, centered 1.11 m ahead of the drone in hover.
  LiDAR runs continuously, not only in hover. Readings are rejected above 15 degrees
  of tilt or outside 0.3 to 11 m.
  CAD moves to SolidWorks.

  Numbers locked:
  Median localization error at 5 m: ~0.15 m (attitude + pixel + altitude combined).
  Attitude dominates the error budget at 0.128 m RMS; altitude contributes 0.013 m
  RMS, negligible near nadir. Vibration isolation on the flight controller matters
  more to accuracy than a better rangefinder would.
  Calibration check recovers f_u = 29.9 from a true 30.0 under 0.3 px noise.

  Known limitations, written down while they are fresh:
  The 0.072 m pipeline result only measures detection error, since transform and
  ground truth share the same attitude and height. The honest expected field number
  is ~0.15 m from the Monte Carlo, and that is the figure I will quote.
  The datasheet-derived f_u and f_v disagree by 19.2 percent. Bench calibration
  against a measured target is mandatory before trusting real frames.
  The detector cannot report "nothing there". The adaptive threshold always finds a
  max, so an empty frame returns the centroid of a noise cluster. Needs a minimum
  contrast or SNR gate before a detection is declared.
  Targets near the frame edge read biased toward the center, because the Gaussian
  blob is truncated by the frame boundary. This will happen with the real sensor too.
  The SNR printout is labeled dB but the value is a plain ratio. Cosmetic, but
  mislabeled units are the kind of thing that makes a reviewer distrust the rest.
  LEVER_ARM_B is a placeholder until the pod geometry is measured in SolidWorks.
- **August 3-4:** When going through and explaining the project to an older friend of
  mine i was put on the spot beacuse of some dangerous problems. These two days has
  been spent trying to fix the project but still keep the intended purpose. The most
  important change is in even more static bench tests that are verifiable and then fixing
  the coding mistakes. Also, if you look at the code as it stands now putting two people overboard
  or two hotspots, this would make the drone drop and think that there is a person in the 
  middle of them. This sets us a day back and is something that will be corrected. 
  I also found a caveat for what i was saying earlier with the speedybee F405 V5. This flight
  controller did have full ardupilot support however was not actually beeing sold in norway, 
  I was looking at T motor F7 flight controller.
- **August 5:** The day started of with making the preperations needed for the first bench test.
  This first bench test is made to understand the maratime aspect regarding the proof of concept. The 
  background for the check is beacuse of the LIDAR sensors wavelength, for later versions it is smarter
  either way to use radar instead of LIDAR. However, the purpose behind the proof of concept was just that,
  make a working "cheap" proof of concept.
  
  This specific Bench test was completed with a breadboard, the LIDAR sensor, esp32 and a computer to
  directly read off data from the esp32. The purpose of the test is to check if the sensor can detect the 
  seawater and reflect at sealevel, or if it goes straight through.

  The sensor is bolted flat to a rigid plate which pivots against a printed arc. The arc only locks the
  angle, it does not define it, since print tolerance means a hole labelled 12.5 degrees is not actually
  12.5 degrees. The angle is read off a physical protractor sitting on the same plate as the sensor,
  referenced to nadir (straight down) as 0 degrees, and only that reading is logged. Because the arc is
  swept across positions rather than commanded to a target, its print tolerance never enters the data:
  the exact labelled degree does not matter, only the angle actually recorded. The whole plate sits on
  top of a fixed 1.5 m wooden pole, set plumb, so the rig can be placed repeatably and its height
  verified by tape to the water surface. Firmware on the esp32 validates each 9-byte frame and prints
  one CSV line over USB, and all counting and logging happens on the laptop. Signal strength is logged
  for every sample, not just distance, since strength is what separates the cases: a surface return is
  strong and reads the rig height, a bottom return is weak and reads rig height plus depth, and a
  specular reflection away from the receiver reads near zero. Distance alone cannot tell them apart.
  Everything is saved raw, classification happens later in analysis.

  Everything was verified on land first so that anything odd over water belongs to the water and not the
  rig. Frames parse, reported distance matches a tape measure at 1 through 5 m, and a wall sweep from
  minus 20 to plus 20 degrees locates the true boresight. That last step matters because the optical axis
  is not guaranteed perpendicular to the housing, and a degree of offset would shift the whole angle
  series without ever showing itself.

  At the water, 200 samples per point. First the same measured height over shallow and deep water, which
  is the test that actually answers the question. Then incidence angle from 0 to 30 degrees, run out of
  order and with 0 repeated at the end so rig drift becomes visible. The range comes from the aircraft and
  not the mounting, since the LIDAR points along nadir, meaning the incidence angle is just the drone's own
  attitude, capped by ArduPilots default ANGLE_MAX of 30 degrees. Then the same again with ripples, which
  break the specular reflection and may improve angle tolerance while making the nadir return worse. Last,
  a dark floating object in the beam to see whether it returns where bare water does not.

  Distance equal to rig height at both depths means a surface return and a valid height source. Distance
  following the water depth means it reads the bottom, which would require 1550 nm LIDAR if we wished to
  keep the maratime aspect, or a radar altimeter instead. A bimodal result is the awkward one, and the 
  question then becomes whether strength separates the two modes cleanly enough to filter on. All three are 
  valid outcomes, a negative result here is a characterised limit rather than a failure.

## Planned

- Pillar C ballistic lead equation with crosswind drag
- MLX90640 intrinsic calibration and axis sign verification once hardware arrives
- Static bench rig: servo actuation delay and empirical drag coefficients
- Indoor hover and drop testing
- Outdoor pipeline testing without payload, A3 category

## Notes on error and uncertainty

Numbers quoted from Monte Carlo simulation are simulation results, not flight results.
Measured localization error against a surveyed ground target is the intended validation
and has not been performed yet.
