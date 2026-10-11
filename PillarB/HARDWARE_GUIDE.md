# Hardware guide

From an empty Windows PC to every peripheral verified, for someone who has
never used PlatformIO. Follow the steps in order. Each has the literal
command, what you should see, and what to do if you do not.

**This guide was written before anyone did the setup, so it will contain
errors.** When a step does not behave as written, fix the step in this file
before moving on: change the command, the expected output, or the
troubleshooting note, so the next person gets the corrected version. Steps
known to be fragile are marked **(fragile)**.

Conventions:
- `REPO` means the folder you cloned the repository into, for example
  `C:\Users\you\Documents\GitHub\reddron`.
- `COM5` stands for your ESP32's serial port. Section 5 shows how to find
  the real one.
- Commands go in the **PlatformIO terminal** (step 1.3) unless a step says
  otherwise.

---

## 0. What you need

| Item | Notes |
|---|---|
| ESP32-DEV-30P | ESP32-WROOM-32 DevKit, 30 pins |
| MLX90640 breakout | 3.3 V board with I2C pull-ups (Adafruit, SparkFun, Pimoroni types) |
| TFmini-S | UART version, 5 V |
| Adafruit 254 microSD breakout | With a 64 GB SanDisk High Endurance or Samsung PRO Endurance card |
| MicoAir H743 | Only for step 7.5; SITL (section 8) needs no flight controller |
| USB cable | A **data** cable. Many cables are charge-only and the ESP32 never appears. |
| Capacitors | 100 uF electrolytic and 100 nF ceramic, for the SD card (step 4.4) |
| Jumper wires | Female-female for the DevKit headers |

Software: Windows 10 or 11, VS Code, Mission Planner (for section 8), and the
project's Python environment with numpy, scipy and pyserial (the one the
tests run in).

---

## 1. Install PlatformIO

**1.1** Install VS Code from https://code.visualstudio.com and start it.

**1.2** Press `Ctrl+Shift+X`, type `PlatformIO IDE`, click **Install** on the
extension by PlatformIO.

Expected: an alien-head icon appears in the left bar. The first install
downloads PlatformIO Core and takes several minutes; a notification says when
it is done. Restart VS Code when asked.

If not: the installer needs Python it downloads itself. If it reports a
Python error, restart VS Code and let it retry; do not install Python by hand
for this.

**1.3** Open a PlatformIO terminal: click the alien-head icon, then under
**Quick Access** choose **Miscellaneous > New Terminal**. Type:

```
pio --version
```

Expected: `PlatformIO Core, version 6.1.19` or later.

If not: if `pio` is "not recognized", you are in an ordinary terminal. Either
open the PlatformIO terminal as above, or use the full path
`%USERPROFILE%\.platformio\penv\Scripts\pio.exe` in place of `pio` in every
command below.

---

## 2. The firmware project

All Pillar B firmware is one PlatformIO project in `REPO\PillarB\firmware`,
with one **environment** per program. An environment is a named build
configuration; `-e name` picks one.

**2.1** Go to the project:

```
cd REPO\PillarB\firmware
```

**2.2** What `platformio.ini` says, line by line:

| Line | Why it is there |
|---|---|
| `[platformio]` `default_envs = mlx90640_stream` | What `pio run` builds when you do not name an environment. |
| `[env]` | Settings every environment inherits. |
| `platform = espressif32 @ 7.0.1` | The ESP32 toolchain and Arduino core, **pinned**. 7.0.1 ships Arduino-ESP32 core 2.0.17; a later version can move to core 3.x, which changes APIs this code uses. |
| `board = esp32dev` | The generic ESP32-WROOM-32 DevKit definition, which fits the DEV-30P: pin numbers, flash size, upload speed. |
| `framework = arduino` | `setup()`/`loop()`, `Serial`, `Wire` and friends. |
| `build_flags = -Wall` | All compiler warnings on. The code builds without any. |
| `[env:mlx90640_stream]` `build_src_filter = +<mlx90640_stream.cpp>` | Each program has its own `setup()` and `loop()`. The filter compiles only this program's file, so the programs never collide. |
| `monitor_speed = 921600` | The streamer's baud rate (section 6.3). |
| `[env:mavlink_request]` `lib_deps = okalachev/MAVLink @ 2.0.33` | The MAVLink C library, downloaded automatically on first build. |
| `monitor_speed = 115200` | The harness prints text at 115200. |
| `[env:mavlink_request_sitl]` `extends = env:mavlink_request` | The same program, plus `-DPILLARB_SITL_TCP`, which switches its link from the FC UART to WiFi TCP. |
| `[env:pillarb_stream]` `build_src_filter = +<mlx90640_stream.cpp> +<mavlink_request.cpp>` | The camera streamer and the FC link in **one** program, so the camera's timestamps and the attitude's arrival times come from the same ESP32 clock. The laptop needs both on one clock to map it onto the FC's (step 7.6). |
| `build_flags = ${env.build_flags} -DPILLARB_WITH_MAVLINK` | Switches the FC link on inside `mlx90640_stream.cpp`. |
| `[env:sd_check]` `lib_deps = greiman/SdFat @ 2.3.1` | SdFat for the SD card check (section 3.3). |

**2.3** Build everything except the SITL build (which needs a file from step
8.3 first):

```
pio run -e mlx90640_stream -e pillarb_stream -e mavlink_request -e sd_check
```

Expected: the libraries download the first time, then four lines ending in
`SUCCESS`, and no warnings.

---

## 3. Libraries

**3.1 MLX90640: none, on purpose.** The streamer talks to the sensor's
registers directly, in the same sequence as the Melexis reference driver's
`MLX90640_GetFrameData`. The available Arduino libraries (for example
`adafruit/Adafruit MLX90640 @ 1.1.2`) exist to compute temperatures on the
microcontroller, which is exactly what this project does not do: compensation
runs on the laptop in `frame_reader.py`, so the ESP32 does less work and the
raw measurement stays recoverable. Nothing to install.

**3.2 MAVLink.** Added by the `lib_deps` line in `platformio.ini`. To fetch
it without building:

```
pio pkg install -e mavlink_request
```

Expected: `Library Manager: MAVLink@2.0.33 has been installed!` (or "already
installed").

**3.3 SdFat.** Also added by `lib_deps`:

```
pio pkg install -e sd_check
```

Expected: `Library Manager: SdFat@2.3.1 has been installed!`

The 64 GB card needs **no reformatting**. Cards that size ship formatted
exFAT, and SdFat reads exFAT directly (on the ESP32 its default
configuration, `SDFAT_FILE_TYPE 3`, enables FAT16, FAT32 and exFAT). Do not
reformat it to FAT32 to "make it compatible": exFAT is also the better choice
for the flight log, because preallocated exFAT space reads back as zeros
after a power loss while FAT32 space can hold an earlier flight's records
(`CODE_WALKTHROUGH.md`, section 7).

If a later program uses SdFat, do not include the Arduino core's `SD.h` or
`FS.h` in the same source file. Both define a type called `File` and the
build fails with a conflicting-declaration error.

---

## 4. Wiring

Everything shares one ground: connect every GND to the ESP32's GND.

**4.1 MLX90640 (I2C)**

| MLX90640 breakout | ESP32 |
|---|---|
| VIN (or 3V3) | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

The sensor's address is 0x33. Breakouts from the vendors above have pull-up
resistors on SDA and SCL. A bare module without them needs 2.2 kOhm from each
of SDA and SCL to 3.3 V, or it will not run at 400 kHz.

**4.2 TFmini-S (UART2)**

| TFmini-S | ESP32 |
|---|---|
| 5 V (usually red) | VIN (5 V from USB) |
| GND (usually black) | GND |
| TX (usually green) | GPIO16 (RX2) |
| RX (usually white) | GPIO17 (TX2) |

**(fragile)** Wire colours vary between cable batches. Go by the label or the
datasheet pinout, not the colour. TX always goes to the ESP32's RX.

**4.3 microSD, Adafruit 254 (VSPI)**

| Breakout pin | ESP32 |
|---|---|
| 3V | 3V3 |
| 5V | **not connected** |
| GND | GND |
| CLK | GPIO18 |
| DO | GPIO19 (MISO) |
| DI | GPIO23 (MOSI) |
| CS | GPIO5 |
| CD | not connected |

**3.3 V goes to the `3V` pin, not `5V`.** The `5V` pin feeds the breakout's
own 3.3 V regulator. Feeding it 3.3 V leaves the regulator below its dropout,
so the card gets a sagging, noisy supply and fails in ways that look like
software bugs. The `3V` pin bypasses the regulator.

Nothing else may share this SPI bus. The breakout's level shifter does not
release MISO when CS is high, so a second SPI device on these pins would be
talked over.

**4.4 The SD card's current bursts.** SD cards draw up to about 100 mA in
short bursts while writing. On a weak 3.3 V rail that dips the voltage enough
to trip the ESP32's brownout detector, which shows up as a **random reboot**,
with `Brownout detector was triggered` on the serial monitor just before the
boot messages. Solder the 100 uF electrolytic and the 100 nF ceramic across
`3V` and `GND` at the breakout, keep the 3V3 and GND wires short, and power
the ESP32 from a USB port or supply that can deliver 500 mA.

**4.5 Flight controller (UART1)**

| FC TELEM port | ESP32 |
|---|---|
| TX | GPIO32 (RX1) |
| RX | GPIO33 (TX1) |
| GND | GND |
| 5V | **not connected** |

The ESP32 is powered from USB (or its own supply). Connecting the FC's 5 V
to it as well puts two supplies against each other.

---

## 5. Serial ports, upload, monitor

**5.1 Find the port.** Plug in the ESP32, then:

```
pio device list
```

Expected: an entry like `COM5` with `Silicon Labs CP210x USB to UART Bridge`
or `USB-SERIAL CH340`. That COM number is the one to use.

You can also press `Win+X`, choose **Device Manager**, and open **Ports (COM
& LPT)**. Unplug and replug the ESP32 to see which entry disappears and
comes back.

If nothing appears:
- Try another cable. Charge-only cables are the most common cause.
- Install the USB driver for the chip on your board: the Silicon Labs CP210x
  driver or the WCH CH340 driver, from the chip maker's site. Replug after.

**5.2 Upload a program**

```
pio run -e mavlink_request -t upload
```

Expected: `Connecting....`, then `Writing at 0x00010000... (100 %)`, then
`Hard resetting via RTS pin...` and `SUCCESS`. PlatformIO finds the port by
itself. With more than one board plugged in, add `--upload-port COM5`.

If it fails:
- `could not open port 'COM5': PermissionError(13, 'Access is denied.')`:
  another program has the port open. Close every serial monitor, the Python
  reader or capture script, the Arduino IDE, and Mission Planner if it is
  connected to that COM port, then upload again.
- `Failed to connect to ESP32: Wrong boot mode detected` or `Timed out
  waiting for packet header`: hold the board's **BOOT** button, start the
  upload, and release the button once `Writing at` appears.
- `No serial data received`: the wrong port, or a charge-only cable (5.1).

**5.3 Watch the output**

```
pio device monitor -b 115200
```

Press `Ctrl+C` to leave. The monitor holds the port, so close it before the
next upload.

For the thermal streamer, the corresponding command is:

```
pio device monitor -b 921600
```

but its output is binary packets, so this shows mostly unreadable characters
with an occasional readable `MLXT` message. That is expected. Read the
streamer with `frame_reader.py` instead (step 7.3).

---

## 6. Two sensor facts that decide the configuration

**6.1 `Wire.setClock(400000)` is required.** Every subpage, the streamer
reads the MLX90640's whole RAM image: 832 16-bit words plus two registers,
about 1.7 kB, which with I2C addressing and acknowledge bits is roughly
15,700 clock cycles. At the I2C default of 100 kHz that takes about 157 ms.
Subpages come every 125 ms at 8 Hz, so at 100 kHz the read cannot keep up:
the sensor overwrites RAM mid-read, and frames arrive corrupted or not at
all. At 400 kHz the read takes about 40 ms. The call is in
`mlx90640_stream.cpp`'s `setup()`; do not remove it.

**6.2 The refresh rate is per subpage.** The MLX90640 measures its pixels in
two interleaved halves (a chessboard by default) and the refresh setting
applies to each half. At 8 Hz you get 8 subpages and **4 complete frames**
per second, the two halves of a frame 125 ms apart. The streamer sets 8 Hz
at every boot, because the sensor forgets the setting at power-up.
`orchestrator.py` pairs the halves.

**6.3 Why 921600 baud.** One subpage packet is 1702 bytes, 13.6 kB/s at
8 Hz, about 136 kbit/s on the wire. That does not fit at 115200 baud at all.
At 921600 it uses about 15 percent of the link.

---

## 7. Verify each peripheral

Do these one at a time and do not move on until a step passes. Wire only the
peripheral under test if you are unsure of the rest.

### 7.1 ESP32 alone

```
pio run -e mavlink_request -t upload
pio device monitor -b 115200
```

Expected, with nothing connected to GPIO32/33:

```
Pillar B mavlink_request harness
[uart] FC link on GPIO32/GPIO33 at 115200 baud
[mavlink] waiting for flight controller heartbeat
[mavlink] ERROR: no heartbeat from a flight controller. Check wiring (TX to RX), baud rate and SERIALn_PROTOCOL = 2.
[NOT READY] ATT_Q   0.0 Hz  ATT   0.0 Hz  POS   0.0 Hz  DIST   0.0 Hz   | parse_err 0 seq_gaps 0 unused 0 B
```

The error is correct here: there is no FC. This proves the board uploads,
runs and prints.

### 7.2 TFmini-S

The TFmini uses the Phase 0 bench-test program, a separate project:

```
cd "REPO\Bench tests\LIDAR over seawater\esp32-logger"
pio run -t upload
pio device monitor -b 115200
```

Expected: the header `t_ms,distance_cm,strength,temp_c`, then about 100 lines
per second like `48213,152,4210,38.5`: time, distance in cm to whatever the
sensor points at, signal strength, and the sensor's own chip temperature. The
onboard LED flashes fast while frames arrive. Point the sensor at a wall and
the distance should match a tape measure within a few cm.

If not:
- No lines, LED dark: TX and RX are swapped, or the sensor has no 5 V.
- Distance `0` or strength below 100: nothing within range, or pointing at the
  sky. Point it at a wall between 0.3 m and 11 m.

Go back to the firmware folder afterwards: `cd REPO\PillarB\firmware`.

### 7.3 MLX90640

```
pio run -e mlx90640_stream -t upload
```

Then, in an ordinary terminal with the project's Python environment active
(close any serial monitor on the port first):

```
cd REPO\PillarB
python frame_reader.py COM5 --save first_capture.mlxraw
```

Expected: one message from the ESP32, then one line per second:

```
[mlx90640] MLX90640 ready: control 0x1A01, 8 Hz per subpage (4 Hz frames), chess mode, I2C 400000 Hz
 8.0 subpages/s  Ta  29.4 C  Vdd 3.30 V  chess       To min/med/max  21.3/ 22.6/ 24.0 C at row 7 col 19 | lost 0 crc 0 discarded 0 B i2c 0 tx_drop 0
```

What to check:
- About **8.0 subpages/s**. Much lower means lost subpages (`lost` rises) or
  I2C trouble (`i2c` rises).
- `Ta` is the sensor's own temperature, a few degrees above the room.
- The median `To` is roughly room temperature. Hold your hand 30 cm in front
  of the sensor: the maximum rises to about 30 to 34 C.
- `crc`, `discarded` and `i2c` stay at 0. A steadily rising `discarded` or
  `crc` count means a bad USB cable or connection.

`Ctrl+C` stops it and prints all counters. The raw bytes are saved in
`first_capture.mlxraw`. `python frame_reader.py --file first_capture.mlxraw`
replays them at any time, which is how a compensation fix gets applied to old
data without flying again.

**Orientation check, required before any ground position is trusted.** With
the sensor mounted as it will fly:
1. Hold your hand in front of the side that faces the aircraft's **nose**. The
   hot spot should be near **row 0**. `pillar_b.py` assumes image row 0 is the
   nose side.
2. Hold it on the aircraft's **right** side. The hot spot should be near
   **column 31**.

If either is reversed, the image is flipped relative to what the transform
assumes. Write down which axis, and do not fly: the fix is a code change in
`frame_reader.py` with a test, not a wiring change.

If not:
- `ERROR: no MLX90640 at 0x33`: check SDA on GPIO21, SCL on GPIO22, 3.3 V and
  GND, and pull-ups (4.1).
- `Python was not found`: Windows is running the Microsoft Store placeholder,
  not your environment. Activate the project environment first (for
  example `conda activate spyder-env`), or run that environment's
  `python.exe` by its full path.
- `could not open port`: close the PlatformIO monitor.

### 7.4 SD card

```
pio run -e sd_check -t upload
pio device monitor -b 115200
```

Expected, after a few seconds:

```
Pillar B SD card check
card: 63.9 GB, filesystem exFAT
counting free space (a few seconds on a large card)
free: 63.8 GB
preallocated 4 MB, contiguous: yes (sectors 32768 to 40959)
wrote 84 batches of 12528 bytes with a sync each: mean 15000 us, worst 90000 us, 2 slower than 50 ms
read back 1052352 bytes: all match
SD CHECK PASSED
```

The sizes, sector numbers and times will differ. What matters:
- `filesystem exFAT` for a 64 GB card as shipped.
- `contiguous: yes`. The flight logger depends on it.
- The **worst** batch time. The flight logger's RAM queue absorbs 4 s of
  stall. A worst case in the tens to low hundreds of milliseconds is normal.
  Anything approaching a second, or a count of slow batches that keeps
  growing on repeated runs, means the card is unsuitable.
- `SD CHECK PASSED`. Press the board's EN button to run it again.

If not:
- `SD CHECK FAILED: mount`: card not inserted, wiring (4.3), or 3.3 V on the
  `5V` pin instead of `3V`.
- A random reboot partway through, with `Brownout detector was triggered`:
  the current bursts in 4.4. Add the capacitors.
- `MISMATCH`: a marginal SPI connection. Shorten the wires; if it persists,
  lower `kSck` in `sd_check.cpp` from 16 to 4 MHz and try again.

### 7.5 Flight controller over UART

Do section 8 (SITL) first if you can: it tests the same code with nothing to
wire.

**7.5.1** In Mission Planner, connected to the FC over its own USB: open
**CONFIG > Full Parameter List** and find the parameters for the TELEM port
the ESP32 is wired to (SERIAL1 for TELEM1, SERIAL2 for TELEM2). Set:
- `SERIALn_PROTOCOL` = `2` (MAVLink2)
- `SERIALn_BAUD` = `115` (115200 baud)

Click **Write Params**, then reboot the FC (unplug and replug, or
**Ctrl+F > Reboot Pixhawk**). A baud change only takes effect after a reboot.

The parameter dump saved for this airframe has the TELEM ports at `57`
(57600). If you would rather keep that, skip the baud change and build with
`-DPILLARB_FC_BAUD=57600` added to `build_flags` of `env:mavlink_request`.
The requested streams use about two thirds of a 57600 link.

**7.5.2** Upload and watch:

```
pio run -e mavlink_request -t upload
pio device monitor -b 115200
```

Expected: the same sequence as step 8.4 below. On this airframe
`DISTANCE_SENSOR` stays at 0 Hz and a note says so: the FC has no
rangefinder configured (`RNGFND1_TYPE` = 0), and height comes from the
TFmini.

After the first run, the `MAVn_*` stream rate parameters for that port (named
`SRn_*` in older firmware) read 0 in the Full Parameter List (refresh it).
The ESP32 set them itself.

### 7.6 Camera and flight controller together

Needs 7.3 and 7.5 working first. The wiring is theirs, unchanged: the
MLX90640 on GPIO21/22 and the FC's TELEM port on GPIO32/33.

```
pio run -e pillarb_stream -t upload
```

Then, as in 7.3 (close any serial monitor on the port first):

```
cd REPO\PillarB
python frame_reader.py COM5 --save both_clocks.mlxraw
```

Expected: the messages from 7.3, one saying the MAVLink link is on, the
stream setup's own lines from 7.5 and a line saying how long the camera
paused for it. Then two lines per second:

```
[mlx90640] pillarb_stream: MAVLink on GPIO32/33, attitude pairs sent as MLXA packets
[mlx90640] MAVLink stream setup done; camera paused 3120 ms
 8.0 subpages/s  Ta  29.4 C  Vdd 3.30 V  chess       To min/med/max  21.3/ 22.6/ 24.0 C at row 7 col 19 | lost 0 crc 0 discarded 0 B i2c 0 tx_drop 0
     attitude 50.0 Hz  ESP32 - FC -37251.84 ms, drift +12 ppm | att lost 0 euler 0 rate mismatch 0
```

The offset depends on when each board was switched on and will differ. What
to check:
- `no clock estimate yet` for the first three or four seconds, then a
  number. Frames are refused (and counted) until then, by design.
- `attitude` about **50 Hz**. Lower means the link cannot carry the requested
  rate (7.5.1, baud).
- The offset stays steady from line to line, changing by well under a
  millisecond, and the drift is within about +-50 ppm. A drift of hundreds
  of ppm, or an offset that jumps, means the data is wrong, not the crystals.
- `euler 0`. A rising count means the FC refused `ATTITUDE_QUATERNION` and
  the Euler fallback is in use (7.5).
- `rate mismatch 0`. Anything else means the sensor is not at the 8 Hz the
  exposure correction assumes (`clock_sync.SUBPAGE_RATE_HZ`).

If not:
- No `attitude` line at all: the FC link is not up. Check it with 7.5 alone.
- `ESP32 - FC` jumps after an FC reboot (by however long the FC had been
  up): expected, once. The estimate starts over and is back within a few
  seconds.

Keep `both_clocks.mlxraw`: it holds both clocks for every attitude
message, so the offset can be re-estimated from it later with a better
method, without flying again.

---

## 8. Testing against SITL (no flight controller needed)

SITL is ArduPilot running as a program on the PC, with a simulated vehicle.
Mission Planner can start it. The ESP32 then reaches it over WiFi.

**8.1 Start SITL.** **(fragile: the Mission Planner UI changes between
versions)** In Mission Planner, click **SIMULATION** in the top menu, pick a
multirotor, and start it. The first time, Mission Planner downloads the
ArduCopter SITL build. A console window opens and Mission Planner connects to
the simulated copter by itself.

Expected in the console window, among the start-up lines:

```
bind port 5760 for SERIAL0
bind port 5762 for SERIAL1
bind port 5763 for SERIAL2
```

Mission Planner is connected on 5760. **The ESP32 uses 5762**, a separate
MAVLink port of the same simulated FC. The ports listen on every network
interface of the PC, so the ESP32 can reach them over WiFi.

**8.2 Let the ESP32 through the firewall.** The first time SITL starts,
Windows asks whether to allow the simulator program on networks. Allow it on
**Private networks**. To check from the PC, in PowerShell:

```
Test-NetConnection -ComputerName 127.0.0.1 -Port 5762
```

Expected: `TcpTestSucceeded : True` while SITL runs.

**(fragile)** If you dismissed the prompt, open **Windows Defender Firewall >
Allow an app through firewall** and allow the simulator executable on
private networks. Your WiFi must be marked as a Private network in Windows.

**8.3 WiFi details for the ESP32.** In `REPO\PillarB\firmware\include`, copy
`secrets.example.h` to `secrets.h` (same folder) and edit `secrets.h`:
- `PILLARB_WIFI_SSID`, `PILLARB_WIFI_PASSWORD`: a **2.4 GHz** network (the
  ESP32 cannot use 5 GHz). The same network the PC is on.
- `PILLARB_SITL_HOST`: the PC's address. In PowerShell run `ipconfig` and use
  the **IPv4 Address** of the WiFi adapter, for example `192.168.1.50`.
- `PILLARB_SITL_PORT`: `5762`.

`secrets.h` is in `.gitignore` and never reaches GitHub. Never put real
credentials in `secrets.example.h`.

**8.4 Build, upload, watch**

```
pio run -e mavlink_request_sitl -t upload
pio device monitor -b 115200
```

Expected (addresses and numbers will differ):

```
Pillar B mavlink_request harness
[sitl] joining WiFi 'your-network'.....
[sitl] WiFi up, ESP32 address 192.168.1.73
[sitl] connecting to 192.168.1.50:5762
[sitl] TCP connected
[mavlink] waiting for flight controller heartbeat
[mavlink] flight controller found: sysid 1 compid 1
[mavlink] requested all stream groups off on this link
[mavlink] ATTITUDE_QUATERNION at 50 Hz: MAV_RESULT 0 after 1 attempt(s)
[mavlink] GLOBAL_POSITION_INT at 25 Hz: MAV_RESULT 0 after 1 attempt(s)
[mavlink] DISTANCE_SENSOR at 10 Hz: MAV_RESULT 0 after 1 attempt(s)
[mavlink] attitude source: ATTITUDE_QUATERNION
[mavlink]   ATTITUDE_QUATERNION  requested   50 Hz  ack  0  attempts 1  measured  50.0 Hz
[mavlink]   GLOBAL_POSITION_INT  requested   25 Hz  ack  0  attempts 1  measured  25.0 Hz
[mavlink]   DISTANCE_SENSOR      requested   10 Hz  ack  0  attempts 1  measured   0.0 Hz
[mavlink]   rx 180 msgs 7400 B, parse errors 0, FC seq gaps 0, unused 30 msgs 1600 B
[mavlink]   commands 3 sent, 0 retries, 0 unacked, 0 rejected, 0 duplicate acks
[mavlink]   fallbacks to Euler 0, link losses 0, attitude stalls 0
[mavlink] note: no DISTANCE_SENSOR from the FC. Expected when the FC has no rangefinder (RNGFND1_TYPE = 0); height comes from the TFmini on UART2.
[ok]  ATT_Q  50.0 Hz  ATT   0.0 Hz  POS  25.0 Hz  DIST   0.0 Hz  | t=48213 q=(1.0000 0.0010 -0.0020 0.0000) | parse_err 0 seq_gaps 0 unused 0 B
```

What to check:
- `attitude source: ATTITUDE_QUATERNION` and `ATT_Q` near **50 Hz** every
  second. That is the whole point of F2.
- `MAV_RESULT 0` (accepted) for each request.
- `unused` bytes stop growing after start-up: the stream groups were really
  switched off on this link.
- `parse_err` and `seq_gaps` stay at 0 or near it.
- If you see `FALLING BACK to ATTITUDE (Euler angles)`, the quaternion
  request failed and the fallback is in use. That must never pass unnoticed:
  the ground station also receives it as a message.

Rotate the simulated copter (take off in Mission Planner and fly it around) and
the quaternion `q` changes.

**8.5 Mission Planner on the same link.** A ground station requests its own
stream rates on the link it is connected to, and can override what the ESP32
set. With the setup above this does not arise: Mission Planner is on 5760 and
the ESP32 on 5762, two separate links. If you instead forward Mission
Planner's own link to the ESP32 (its **MAVLink Mirror** feature), they share
one link. Then disconnect Mission Planner (the **DISCONNECT** button) once
the ESP32 reports its rates, or the rates may change under it.

**8.6 Message rates in Mission Planner.** **(fragile)** **Ctrl+F** opens a
tools window with a **MAVLink Inspector**, which lists every message Mission
Planner receives with its rate. It shows Mission Planner's own link (5760),
not the ESP32's. The ESP32's once-a-second line in 8.4 is the measurement for
the ESP32's link.

---

## 9. Done

Every peripheral is verified when:
1. 7.1 prints the harness banner,
2. 7.2 shows sensible distances at about 100 lines per second,
3. 7.3 shows 8 subpages/s, your hand as a hot spot, and the orientation check
   passes,
4. 7.4 ends in `SD CHECK PASSED` with an acceptable worst batch time,
5. 8.4 (and later 7.5) shows `ATTITUDE_QUATERNION` at about 50 Hz.

Record anything you had to change in this file, and the measured numbers
(worst SD batch time, subpage rate, attitude rate) in the README log.
