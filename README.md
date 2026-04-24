Dual Joy‑Con to USB HID Gamepad (ESP32‑S3)

https://www.youtube.com/watch?v=64u_5oImT9Y 

https://oshwlab.com/scavenrage/gamepad_tablet_pro

This project was only possible thanks to the research done by dekuNukem
https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering

This project transforms a pair of original Nintendo Switch Joy‑Con controllers into a fully‑featured USB HID gamepad using an ESP32‑S3 Waveshare Zero board.
The goal was to create a single, wired, latency‑free controller that behaves like a professional‑grade gamepad and is compatible with systems expecting an Xbox‑style HID layout.

 

Key Features
1. Dual Joy‑Con Integration
Both the left and right Joy‑Cons are connected to the ESP32‑S3 via independent UART interfaces running at 1 Mbps.
Each Joy‑Con is fully decoded using a custom state machine that handles:

Initial handshaking (A5, MAC request, mode configuration)
High‑speed status polling
Stick data extraction (full 12‑bit resolution)
Button matrix decoding (main + shared button fields)
Watchdog and error‑recovery mechanisms
Automatic re‑detection on disconnect
The result is a rock‑solid wired Joy‑Con interface with continuous frame reception and zero packet loss.

2. USB HID Gamepad (TinyUSB)
The ESP32‑S3’s native USB‑OTG interface is used to enumerate as a USB HID game controller.
A custom HID mapping translates Joy‑Con inputs to standard gamepad controls:

Left Stick → HID X / Y
Right Stick → HID RX / RY
ZL / ZR → Analog or digital triggers (Z / RZ)
D‑Pad → HID Hat switch (8‑way + center)
A fully reorganized button‑bit matrix ensures perfect compatibility with the target device, which interprets controllers using an Xbox‑style logic, not the standard HID ordering.
This required a detailed reverse‑engineering of the actual button positions expected by the device, followed by a custom remapping layer.

3. Accurate Stick Normalization
Joy‑Con analog sticks output values in the range 0–4095.
These are converted into signed 8‑bit HID axes (−127…+127), centered at zero, with:

Dead‑zone handling
Y‑axis inversion (to match HID convention)
Smooth scaling and edge clamping
This results in smooth, natural joystick behavior that mirrors high‑quality commercial controllers.

💡 4. On‑Board WS2812 Status LED
A single WS2812 RGB LED on GPIO21 provides real‑time feedback:

Purple: Both Joy‑Cons connected and polling
Blue / Green: Only one Joy‑Con connected
Yellow: Handshake phase
Red: Communication error / watchdog reset
Dim red: Idle state
This makes the device easy to debug and visually intuitive during operation.

🧠 5. Robust Real‑Time Architecture
The firmware uses:

Two independent UART reading tasks (pinned to separate cores)
FreeRTOS message queues for frame transfer
A central state machine for each Joy‑Con
Ultra‑low‑latency HID report generation (1 ms loop)
Even under dropped packets, desyncs, or hot‑unplug events, the system recovers automatically without rebooting.

🎯 6. Final Result
The final device is:

Wired
Lag‑free
Perfectly recognized as a fully functional game controller
Precise, with smooth analog sticks and correct trigger behavior
Stable, thanks to error handling and reconnection logic
Compact & efficient, using a minimal ESP32‑S3 module
It effectively turns two Nintendo Joy‑Cons into a single high‑performance USB gamepad, reusable in gaming, robotics, accessibility applications, and custom interactive systems.
