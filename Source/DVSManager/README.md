# DVS Manager for Windows

DVS Manager is the always-on USB control application for the DIY wireless DVS
receiver. It replaces the Wi-Fi access-point portal for normal use while
leaving the receiver fully standalone.

## What it does

- Finds the ESP32-S3 receiver automatically on any COM port.
- Reconnects after receiver resets, USB removal, or COM-port number changes.
- Shows both deck links, battery gauges, RPM, RSSI, packet loss, packet age,
  pairing state, and calibration messages.
- Edits timecode format, output gain, base speed, LED brightness, pairing
  window, low-battery threshold, low-battery LED flashing, and emergency AP
  availability.
- Starts calibration, opens pairing, or clears assignments over USB.
- Sends Windows notifications for low/critical battery, link loss, link
  recovery, and completed calibration.
- Runs in the system tray and can start minimized with Windows.
- Exports the current session's activity log as CSV.

Settings remain in the receiver's NVS. Closing the app does not stop timecode
or change failover behavior.

## First use

1. Flash `receiver_s3_unified_revised.ino` from the companion firmware package.
2. In Arduino IDE, select the ESP32-S3 receiver board and enable **USB CDC On
   Boot** so Windows exposes a COM port.
3. Connect the receiver to the PC with its native USB data cable.
4. Run `DVSManager.exe`. No installer or .NET download is required.
5. Allow several seconds for automatic detection after a fresh receiver boot.

The executable is not code-signed. Windows SmartScreen may show an
unrecognized-app warning on first launch; use **More info → Run anyway** only
if the file came from this package and its SHA-256 matches `SHA256SUMS.txt`.

## Normal operation

Closing the window sends DVS Manager to the system tray. Use the tray icon to
open it again or exit completely.

If a puck link is lost, the receiver keeps its intentional hold-last-stable-RPM
behavior. DVS Manager displays **HOLDING** and notifies you to:

1. Switch that deck to Internal.
2. Reconnect or power-cycle the puck.
3. Wait for the manager to report that the link is restored.
4. Return the deck to Relative.

## Emergency AP

The old web portal is retained as an optional recovery path. It is never needed
for routine management. Disable **Emergency AP fallback** in Settings if the
receiver button should not be allowed to start it.

## Source build

The included source targets .NET 8 WPF on Windows x64.

```powershell
dotnet build DVSManager.csproj -c Release
dotnet publish DVSManager.csproj -c Release -r win-x64 --self-contained true `
  -p:PublishSingleFile=true -p:IncludeNativeLibrariesForSelfExtract=true
```

`lib/System.IO.Ports.dll` is the Windows assembly from Microsoft's
`System.IO.Ports` 8.0.0 NuGet package.

## Safety

Bench-test USB detection, both deck directions, calibration, pairing, battery
warnings, link loss/recovery, Serato, and Traktor before performance use. The
application builds without warnings and its protocol parser passes the included
self-test. The receiver firmware passed delimiter, preprocessor, embedded-page,
and JavaScript validation; the final Arduino compile and USB/radio/audio
behavior must still be verified with the physical receiver and its known-good
board settings.
