# Mini-Arm gripper camera

This firmware runs the camera mounted on the Mini-Arm gripper. Homebase is one viewer of its feed; the camera also has its own local page, snapshot endpoint and MJPEG stream.

Prepared for user-managed flashing. **No ESP32 has been flashed by this task.** The user-supplied ESP32-CAM pinout diagram matches the **AI Thinker-compatible ESP32-CAM** layout, so that profile is selected for bring-up. This identifies the diagram, not the physical camera sensor or its operation. The profile uses Espressif's [camera pin definitions](https://github.com/espressif/arduino-esp32/blob/3.3.8/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h). Do not select it for an ESP32-S3 or a differently wired camera board.

## What it provides

- `http://CAMERA_IP/`: camera page with start/stop buttons.
- `http://CAMERA_IP/capture`: a fresh JPEG snapshot, used by Homebase's on-demand preview.
- `http://CAMERA_IP:81/stream`: continuous MJPEG, capped at 8 fps by default.
- `http://CAMERA_IP/status`: firmware identity, readiness, uptime, RSSI and PSRAM status.
- Wi-Fi station mode, reconnect attempts and optional `miniarm-camera.local` discovery. It does not create an open access point.
- VGA JPEG with two frame buffers when PSRAM is available; QVGA with one otherwise. No recording or cloud upload. Flash LED remains off.

This initial service is unauthenticated HTTP intended for your trusted home LAN. It does not implement Internet access or account management. Only one continuous stream viewer is supported by the stream server; pause it when using the Homebase snapshot preview on low-memory boards.

## Configure and flash yourself

1. Use the **AI Thinker ESP32-CAM** board target for the layout you supplied. Check the actual board pin labels against the diagram when connecting the programmer.
2. A local Git-ignored `GripperCamera/camera_config.h` has been prepared with the selected board profile and empty Wi-Fi fields (existing local configurations are preserved). Enter your 2.4 GHz Wi-Fi SSID/password there. For a fresh checkout, copy `camera_config.example.h` to `camera_config.h` and set `CAMERA_BOARD_CONFIRMED` to `1`. Mirror, flip, JPEG quality and stream frame rate are also configurable.
3. Install **esp32 by Espressif Systems 3.3.8** in Arduino IDE Boards Manager, using the [official package index](https://espressif.github.io/arduino-esp32/package_esp32_index.json). Open `GripperCamera/GripperCamera.ino` and select **AI Thinker ESP32-CAM**. That is the compile-tested target (`esp32:esp32:esp32cam`).
4. Connect your camera's programming adapter and select its COM port. Use the board/adapter's upload procedure. On the classic AI Thinker board without automatic boot control, GPIO0 must be low during reset to enter the bootloader; remove that connection and reset after upload. Use the board's specified power input and 3.3 V UART logic.
5. Upload yourself. Open Serial Monitor at **115200 baud**. The camera prints its assigned IP when Wi-Fi connects. Credentials are not printed.
6. Open `http://CAMERA_IP/` and test a snapshot. In Homebase select **Gripper camera → Configure**, save that base address, then **Overview → Start preview**. The direct MJPEG stream link is available there too.

The repository's older test script refers to `192.168.1.185:81/stream`; its base HTTP address did not respond during this task. That address is a historical hint, not a discovered camera. Prefer the address printed by your board.

## Build verification

Compiled successfully on September 6, 2026 using Arduino-ESP32 **3.3.8**, FQBN `esp32:esp32:esp32cam`, and a temporary copy of the example configuration with board confirmation set to 1 and **empty Wi-Fi credentials**. The build used 1,019,585 bytes of application storage and 59,288 bytes of static RAM. This confirms compilation for the reference board, not camera operation on your hardware. The temporary output must not be treated as your configured flash image.

Local compile-only tooling lives in ignored `build/arduino-tools/`. After making your own configuration, an optional command from the repository root is:

```powershell
& build/arduino-tools/cli/arduino-cli.exe --config-file build/arduino-tools/arduino-cli.yaml compile --fqbn esp32:esp32:esp32cam --build-path build/camera-user-build firmware/esp32-camera/GripperCamera
```

No upload command is run by that command. The sketch uses the [Espressif camera API](https://github.com/espressif/esp32-camera) and `esp_http_server`. Camera image orientation, focus, frame rate and recovery after Wi-Fi loss require validation after you flash it.


## Wiring a separate USB-to-UART adapter

If you use an ESP32-CAM-MB USB programming base, seat the camera module in that base and follow its boot/reset procedure. For a separate USB-to-UART adapter, wire with power disconnected:

| Connection | ESP32-CAM pin from your diagram |
|---|---|
| Regulated 5 V supply | **5V** (upper left), not the ambiguous VCC pin |
| Supply/adapter ground | **GND** |
| Adapter TX, **3.3 V logic** | **U0R / GPIO3** |
| Adapter RX | **U0T / GPIO1** |
| Temporary programming jumper | **GPIO0 to GND** |

Use a supply capable of powering the camera; a small adapter's 3.3 V regulator may be inadequate. If using a separate 5 V supply, share ground with the UART adapter and leave the adapter's power pin disconnected. Power input and UART signal voltage are different: use 5 V at the board's 5V input, but 3.3 V UART logic.

Reset with GPIO0 grounded to enter the bootloader, then upload in Arduino IDE. After upload, remove the GPIO0-to-GND jumper and reset again to run the camera. This boot sequence follows [Espressif's boot-mode documentation](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html). GPIO0 also serves the camera clock during normal operation, so do not leave the programming jumper installed.

Select the camera adapter's COM port; **COM3 belongs to the Pico arm controller unless Windows has reassigned ports**. No camera firmware upload has been performed by the assistant.


## First live verification — September 7, 2026

The user uploaded the firmware and supplied the boot log showing `http://192.168.1.189/`. Read-only checks from the host confirmed `/status` HTTP 200, firmware 1.0.0, camera readiness, PSRAM and RSSI -53 dBm at that observation. `/capture` returned a valid 640 x 480 JPEG, and port 81 `/stream` returned multipart MJPEG with multiple complete JPEG frames. The inspected image was very dark; this is a working transport, not a verified usable scene or calibrated view. Check lens exposure, lighting, focus and orientation next. Network addresses and signal levels can change.

Homebase's default gripper-camera address is updated. Existing browser-saved endpoint settings take precedence; if the preview has no address or an old one, set `http://192.168.1.189/` in Gripper camera → Configure. The assistant did not upload any ESP32 firmware.


## Image settings — firmware 1.1.0

Rebuild and upload **GripperCamera.ino** yourself using your existing Git-ignored `camera_config.h`. Keep `camera_controls.h` and `settings_page.h` in the same sketch folder. No credentials, pin assignments or boot settings were changed. The assistant has not flashed this update.

After upload, refresh Homebase and select **Gripper camera → Image settings**. The panel loads the camera's own settings interface, also available at `http://CAMERA_IP/settings` or under **Image settings** on the camera's main page. This keeps the controls usable independently of Homebase. Firmware 1.0.0 does not serve the new settings page; a missing page before updating is expected.

Controls provided:

| Control | Range / behavior |
|---|---|
| Brightness, contrast, saturation | -2 to +2 |
| Automatic exposure | On/off |
| Exposure compensation | -2 to +2; shown when auto exposure is enabled |
| Manual exposure | 0–1200 sensor units; requires automatic exposure off |
| Automatic gain | On/off |
| Manual gain | 0–30 driver steps; requires automatic gain off |
| Automatic white balance | On/off |
| Mirror / flip | Horizontal / vertical |
| JPEG quality | 4–63; lower numbers mean better quality and larger images |

Move a slider and press its **Apply** button; toggles apply immediately. The page reads the sensor-driver settings back after each change. **Reload values** refreshes them without changing the image. **Reset startup settings** reapplies the settings captured after camera initialization, including your configured JPEG quality and mirror/flip values. Changes are kept in RAM and reset on reboot; there is no flash write for each adjustment.

For a darker scene, first try a small brightness increase or positive automatic exposure compensation. Manual exposure values are sensor units, not milliseconds. Longer exposures may blur moving gripper views and additional gain can increase noise. The controls use [Espressif's sensor interface and ranges](https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h).

The firmware serves `GET /settings/state` for current/default values and accepts same-origin JSON `POST /settings/state` with exactly `{"name":"brightness","value":1}` or `{"reset":true}`. Requests are bounded to 192 bytes; unknown names, extra/duplicate fields, noninteger/out-of-range values and manual changes while the corresponding auto mode is on are rejected. No settings-changing GET endpoint or cross-origin API is exposed. Driver failures are reported instead of showing a successful change; a reset interrupted by a driver error may apply only some settings, so reload values before retrying. The service remains trusted-LAN HTTP, not an authenticated Internet service.

Validation: the AI Thinker ESP32-CAM build with Arduino-ESP32 3.3.8 and placeholder Wi-Fi settings succeeds. Host tests compile the actual firmware settings handlers with fake HTTP/sensor I/O and upstream cJSON 1.7.19. They cover fragmented/truncated JSON, bounds, extra fields, origin checks, auto/manual gating, readback, reset and driver failure. No new sensor behavior has been exercised on your physical camera yet. Dashboard TypeScript, lint, unit tests and production compilation are checked; browser interaction tests have not been run.

To repeat the host handler test with the already-downloaded local test dependency:

```powershell
g++ -std=c++17 -I build/camera-control-tests tests/camera_controls_test.cpp build/camera-control-tests/cJSON.c -o build/camera-control-tests/controls-test.exe
& build/camera-control-tests/controls-test.exe
```

The dependency sources are from the [upstream cJSON v1.7.19 release](https://github.com/DaveGamble/cJSON/tree/v1.7.19), stored only in the ignored build folder. These host tests do not communicate with a camera.
