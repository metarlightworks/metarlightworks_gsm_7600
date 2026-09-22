METARLightworks GSM firmware - LilyGO T-SIM7600G-H / T-SIM7600X ESP32 build

Important Arduino IDE setup:
1) Use the LilyGO TinyGSM fork from:
   https://github.com/Xinyuan-LilyGO/LilyGo-Modem-Series/tree/main/lib
   Do not use the standard Library Manager TinyGSM for this SIM7600 build.
2) Select an ESP32 board target appropriate for the original ESP32 T-SIM7600 board, not an ESP32-S3 board.
3) The sketch selects #define LILYGO_SIM7600X in the .ino.
4) Cellular HTTPS uses SIM7600 modem built-in HTTPS calls and passes the AVWX token as a URL query parameter.
5) OTA remains Wi-Fi-only. If you release this as 1.0.6, update ota.json and the GitHub release asset before using OTA install.

Key 7600G pin map used:
MODEM_RX_PIN 26
MODEM_TX_PIN 27
BOARD_PWRKEY_PIN 4
MODEM_DTR_PIN 32
MODEM_FLIGHT_PIN 25
BOARD_LED_PIN 12
