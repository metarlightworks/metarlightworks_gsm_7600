# SIM7600G Hardware Notes

Target board:
- LilyGO T-SIM7600G / T-SIM7600G-H
- SIMCOM SIM7600G-H modem
- ESP32
- APN: iot.1nce.net

Important:
- Use SIM7600 built-in HTTPS path.
- Do not use TinyGSM isGprsConnected() as the main SIM7600 health check.
- Do not force AT+CFUN=1 / AT+COPS=0 on normal boot.
