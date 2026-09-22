#pragma once
#include <sys/time.h>
#include <time.h>
#include <Arduino.h>
#include <TinyGsmClient.h>

// Reuse your existing globals defined in your main file
extern TinyGsm modem;
extern HardwareSerial SerialAT;   // comes from utilities.h on T-A76XX

// ---- helpers ----
static void applyUserTZ(int tzHours) {
  // Fixed-offset TZ like GMT5 (= UTC-5), GMT7 (= UTC-7), etc.
  String tz = "GMT" + String(-tzHours);
  setenv("TZ", tz.c_str(), 1);
  tzset();
}

static bool setSystemTimeUTC(int Y, int M, int D, int h, int m, int s) {
  setenv("TZ", "UTC0", 1); tzset();      // interpret tm as UTC
  struct tm t = {};
  t.tm_year = Y - 1900;
  t.tm_mon  = M - 1;
  t.tm_mday = D;
  t.tm_hour = h;
  t.tm_min  = m;
  t.tm_sec  = s;
  time_t epoch = mktime(&t);            // as UTC (because TZ=UTC0)
  if (epoch < 100000) return false;

  struct timeval tv = { epoch, 0 };
  settimeofday(&tv, nullptr);
  return true;
}

// Try to get time from the cell network (fast, no GPS required)
static bool syncTimeFromCell(int tzHours) {
  modem.sendAT("+CCLK?");
  if (modem.waitResponse(3000L, "+CCLK:") != 1) {
    modem.waitResponse(); // flush remainder
    return false;
  }
  String line = modem.stream.readStringUntil('\n'); // e.g.  " \"25/08/28,16:23:45-20\"\r"
  line.trim();

  int q1 = line.indexOf('"');
  int q2 = line.indexOf('"', q1 + 1);
  if (q1 < 0 || q2 <= q1) return false;

  String dt = line.substring(q1 + 1, q2); // yy/MM/dd,hh:mm:ss±zz
  if (dt.length() < 17) return false;

  int YY   = dt.substring(0, 2).toInt();
  int Y    = 2000 + YY;
  int Mon  = dt.substring(3, 5).toInt();
  int Day  = dt.substring(6, 8).toInt();
  int Hr   = dt.substring(9, 11).toInt();
  int Min  = dt.substring(12, 14).toInt();
  int Sec  = dt.substring(15, 17).toInt();

  // timezone in quarters of an hour (±zz)
  int tzQuarters = 0;
  if (dt.length() >= 20) {               // has ±zz
    int sign = (dt.charAt(17) == '-') ? -1 : 1;
    tzQuarters = sign * dt.substring(18).toInt();
  }
  int tzMinutes = tzQuarters * 15;

  // First set as-if it's UTC…
  if (!setSystemTimeUTC(Y, Mon, Day, Hr, Min, Sec)) return false;

  // …then adjust to true UTC by subtracting the local offset
  struct timeval tv{};
  gettimeofday(&tv, nullptr);
  tv.tv_sec -= (long)tzMinutes * 60L;
  settimeofday(&tv, nullptr);

  applyUserTZ(tzHours);
  return true;
}

// Try to get time from GNSS (UTC from satellites)
static bool syncTimeFromGNSS(int tzHours, uint32_t maxWaitMs = 15000) {
  // Power GNSS on
  modem.sendAT("+CGNSPWR=1");
  modem.waitResponse(2000);

  uint32_t start = millis();
  while (millis() - start < maxWaitMs) {
    modem.sendAT("+CGNSINF");
    if (modem.waitResponse(2000L, "+CGNSINF:") != 1) {
      modem.waitResponse();
      delay(500);
      continue;
    }
    String line = modem.stream.readStringUntil('\n'); // CSV: <mode>,<fix>,<utc>,<lat>,<lon>,...
    line.trim();
    // pull UTC field (3rd token)
    int c1 = line.indexOf(',');
    if (c1 < 0) continue;
    int c2 = line.indexOf(',', c1 + 1);
    if (c2 < 0) continue;
    int c3 = line.indexOf(',', c2 + 1);
    if (c3 < 0) continue;
    String utc = line.substring(c2 + 1, c3); // yyyymmddhhmmss.s
    if (utc.length() < 14) { delay(500); continue; }

    int Y   = utc.substring(0, 4).toInt();
    int Mon = utc.substring(4, 6).toInt();
    int Day = utc.substring(6, 8).toInt();
    int Hr  = utc.substring(8, 10).toInt();
    int Min = utc.substring(10, 12).toInt();
    int Sec = utc.substring(12, 14).toInt();

    if (!setSystemTimeUTC(Y, Mon, Day, Hr, Min, Sec)) return false;
    applyUserTZ(tzHours);
    return true;
  }
  return false;
}

// Public: call this to ensure the clock is set (cell first, then GPS)
static bool ensureTime(int tzHours) {
  struct tm ti{};
  if (getLocalTime(&ti, 1000)) return true;           // already valid

  if (syncTimeFromCell(tzHours)) {
    Serial.println("⏱️ Time set from cellular network (+CCLK).");
    return true;
  }
  if (syncTimeFromGNSS(tzHours)) {
    Serial.println("🛰️ Time set from GNSS (+CGNSINF).");
    return true;
  }
  Serial.println("⚠️ Time sync failed (cell + GNSS).");
  return false;
}
