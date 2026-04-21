// -----------------------------------------------------------------
//
//    Walter Multi-Source MQTT Location Tracker
//
//    2026 - nmc
//
//    Tracker uses multiple location sources in stages - GNSS as
//    1st priority, CellID location data as 2nd then if all else 
//    fails, uses Google Geolocation using WiFi AP + CellID data.
//
//    Publishes results via MQTT at set interval (adjustable below).
//
//    Walter is also setup to use LTE-M PSM (power saving) & deep
//    sleep between publish intervals.
//
//    The WalterModem libraries and example snippets are all
//    Copyright (C), DP Technics bv:
//
//    https://github.com/QuickSpot/walter-arduino/raw/refs/heads/main/LICENSE
//
// -----------------------------------------------------------------

#include <WalterModem.h>
#include <inttypes.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// ---------- MQTT ----------
#define MQTT_PORT 1883
#define MQTT_HOST "mqtt.server"
#define MQTT_TOPIC "location/walter"
#define MQTT_CLIENT_ID "walter_1"
#define MQTT_USERNAME "mqttuser"
#define MQTT_PASSWORD "mqttpass"

// ---------- Timing ----------
#define PUBLISH_INTERVAL_SECONDS 540   // 9 minutes
#define RETRY_INTERVAL_SECONDS 60      // retry after 1 minute on failure

// ---------- PSM request ----------
// Request a TAU longer than the publish interval.
// Request a short active time.
#define PSM_TAU_SECONDS        900    // 15 minutes
#define PSM_ACTIVE_SECONDS     30      // 30 seconds

// ---------- Cellular / GNSS ----------
#define CELLULAR_APN "hologram"
#define MAX_GNSS_CONFIDENCE 200.0
#define RADIO_TECHNOLOGY WALTER_MODEM_RAT_LTEM

// --------- WiFi location ----------

#define GOOGLE_GEOLOCATION_API_KEY "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#define WIFI_SCAN_MIN_APS 2
#define WIFI_SCAN_MAX_APS 12
#define WIFI_FALLBACK_AFTER_GNSS_FAILS 2

WalterModem modem;

bool mqtt_connected = false;
bool gnss_fix_received = false;
bool assistance_update_received = false;
bool publish_pending = false;
bool last_gnss_ok = false;

volatile bool mqtt_publish_done = false;
volatile bool mqtt_publish_ok = false;
volatile bool http_ring_received = false;
volatile uint16_t http_ring_status = 0;
volatile uint16_t http_ring_length = 0;
volatile uint8_t http_ring_profile = 0;

uint8_t in_buf[4096] = {0};
char psmTauStr[9] = {0};
char psmActiveStr[9] = {0};

WMGNSSFixEvent latestGnssFix = {};
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t publishCycle = 0;
RTC_DATA_ATTR uint32_t gnssFailStreak = 0;
RTC_DATA_ATTR uint32_t lastWifiGeoCycle = 0;

RTC_DATA_ATTR bool cachedWifiLocationValid = false;
RTC_DATA_ATTR float cachedWifiLat = 0.0f;
RTC_DATA_ATTR float cachedWifiLon = 0.0f;
RTC_DATA_ATTR float cachedWifiAcc = 0.0f;
RTC_DATA_ATTR uint8_t cachedWifiApCount = 0;

RTC_DATA_ATTR uint16_t cachedWifiCc = 0;
RTC_DATA_ATTR uint16_t cachedWifiNc = 0;
RTC_DATA_ATTR uint16_t cachedWifiTac = 0;
RTC_DATA_ATTR uint32_t cachedWifiCid = 0;
RTC_DATA_ATTR float cachedWifiRsrp = 0.0f;

bool last_wifi_ok = false;
float last_wifi_lat = 0.0f;
float last_wifi_lon = 0.0f;
float last_wifi_acc = 0.0f;
uint8_t last_wifi_ap_count = 0;

struct {
  uint16_t cc;
  uint16_t nc;
  uint16_t tac;
  uint32_t cid;
  float rsrp;
} lastCell;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

bool waitForNetwork(int timeout_sec = 300)
{
  Serial.print("Connecting to the network...");
  int time = 0;

  while(!lteConnected()) {
    Serial.print(".");
    delay(1000);
    time++;
    if(time > timeout_sec) {
      Serial.println();
      return false;
    }
  }

  Serial.println();
  Serial.println("Connected to the network");
  return true;
}

bool lteDisconnect()
{
  uint32_t startMs = millis();

  if(modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.println("Successfully set operational state to MINIMUM");
  } else {
    Serial.println("Error: Could not set operational state to MINIMUM");
    return false;
  }

  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING) {
    if(millis() - startMs >= 30000) {
      Serial.println("LTE disconnect timeout waiting for NOT_SEARCHING");
      return false;
    }

    delay(100);
    regState = modem.getNetworkRegState();
  }

  Serial.println("Disconnected from the network");
  return true;
}

bool lteConnect()
{
  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println("Successfully set operational state to NO RF");
  } else {
    Serial.println("Error: Could not set operational state to NO RF");
    return false;
  }

  if(modem.definePDPContext(1, CELLULAR_APN)) {
    Serial.println("Created PDP context");
  } else {
    Serial.println("Error: Could not create PDP context");
    return false;
  }

  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Successfully set operational state to FULL");
  } else {
    Serial.println("Error: Could not set operational state to FULL");
    return false;
  }

  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println("Network selection mode was set to automatic");
  } else {
    Serial.println("Error: Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork(60);
}

void configurePSM()
{
  uint32_t actualTauSeconds = 0;
  uint32_t actualActiveSeconds = 0;

  uint8_t tauEnc = WalterModem::durationToTAU(
    PSM_TAU_SECONDS, 0, 0, &actualTauSeconds
  );

  uint8_t activeEnc = WalterModem::durationToActiveTime(
    PSM_ACTIVE_SECONDS, 0, &actualActiveSeconds
  );

  snprintf(psmTauStr, sizeof(psmTauStr), "%s", BINBYTESTR(tauEnc));
  snprintf(psmActiveStr, sizeof(psmActiveStr), "%s", BINBYTESTR(activeEnc));

  Serial.printf("Requesting PSM: TAU=%s (~%lu s), Active=%s (~%lu s)\r\n",
                psmTauStr, (unsigned long)actualTauSeconds,
                psmActiveStr, (unsigned long)actualActiveSeconds);

  if(modem.configPSM(WALTER_MODEM_PSM_ENABLE, psmTauStr, psmActiveStr)) {
    Serial.println("PSM request sent successfully");
  } else {
    Serial.println("Warning: Failed to configure PSM");
  }
}

void goToDeepSleep(uint32_t sleepSeconds)
{
  Serial.println("Preparing for sleep...");

  if(mqtt_connected) {
    if(!modem.mqttDisconnect()) {
      Serial.println("Warning: MQTT disconnect failed");
    }
    mqtt_connected = false;
  }

  if(lteConnected()) {
    if(!lteDisconnect()) {
      Serial.println("Warning: LTE disconnect failed");
    }
  }

  delay(200);
  Serial.printf("Entering Walter deep sleep for %lu seconds...\r\n",
                (unsigned long)sleepSeconds);
  Serial.flush();

  WalterModem::sleep(sleepSeconds, false);
}

static bool mqttPublishMessage(const char* topic, const char* message)
{
  Serial.printf("Publishing to topic '%s': %s\r\n", topic, message);

  if(!modem.mqttPublish(topic, (uint8_t*)message, strlen(message))) {
    Serial.println("Publishing failed");
    return false;
  }

  return true;
}

String buildGoogleGeoPayload(int found, int maxAps = 8)
{
  StaticJsonDocument<1024> doc;

  doc["considerIp"] = false;

  JsonArray wifi = doc.createNestedArray("wifiAccessPoints");

  int used = 0;
  for(int i = 0; i < found && used < maxAps; ++i) {
    String bssid = WiFi.BSSIDstr(i);
    if(bssid.length() == 0) continue;

    JsonObject ap = wifi.createNestedObject();
    ap["macAddress"] = bssid;
    ap["signalStrength"] = WiFi.RSSI(i);
    ap["channel"] = WiFi.channel(i);
    used++;
  }

  JsonArray cells = doc.createNestedArray("cellTowers");
  JsonObject cell = cells.createNestedObject();

  cell["mobileCountryCode"] = lastCell.cc;
  cell["mobileNetworkCode"] = lastCell.nc;
  cell["locationAreaCode"] = lastCell.tac;
  cell["cellId"] = lastCell.cid;
  cell["signalStrength"] = lastCell.rsrp;

  last_wifi_ap_count = used;

  String body;
  serializeJson(doc, body);
  return body;
}

bool googleGeolocateLTE(const String& payload)
{
  WalterModemRsp rsp = {};
  static uint8_t httpBuf[1024];

  last_wifi_ok = false;
  last_wifi_lat = 0.0f;
  last_wifi_lon = 0.0f;
  last_wifi_acc = 0.0f;

  Serial.println("Starting LTE Google Geolocation request...");

  if(!modem.tlsConfigProfile(1, WALTER_MODEM_TLS_VALIDATION_NONE)) {
    Serial.println("TLS profile config failed");
    return false;
  }

  if(!modem.httpConfigProfile(
        0,
        "www.googleapis.com",
        443,
        1,
        false,
        "",
        "",
        180,
        90,
        30)) {
    Serial.println("HTTP profile config failed");
    return false;
  }

  String url = "/geolocation/v1/geolocate?key=" + String(GOOGLE_GEOLOCATION_API_KEY);

  http_ring_received = false;
  http_ring_status = 0;
  http_ring_length = 0;
  http_ring_profile = 0;

  if(!modem.httpSend(
        0,
        url.c_str(),
        (uint8_t*)payload.c_str(),
        payload.length(),
        WALTER_MODEM_HTTP_SEND_CMD_POST,
        WALTER_MODEM_HTTP_POST_PARAM_JSON,
        nullptr,
        0,
        &rsp)) {
    Serial.println("HTTP send failed");
    return false;
  }

  Serial.printf("HTTP send accepted: type=%d result=%d\r\n",
                (int)rsp.type, (int)rsp.result);

  unsigned long startMs = millis();
  while(!http_ring_received && millis() - startMs < 30000) {
    delay(100);
  }

  if(!http_ring_received) {
    Serial.println("Timed out waiting for HTTP RING");
    return false;
  }

  if(http_ring_profile != 0) {
    Serial.printf("Unexpected HTTP profile in RING: %u\r\n", http_ring_profile);
    return false;
  }

  if(http_ring_status != 200) {
    Serial.printf("Google geolocation returned HTTP status %u\r\n", http_ring_status);
    return false;
  }

  if(http_ring_length >= sizeof(httpBuf)) {
    Serial.println("Warning: HTTP response may be truncated");
  }

  memset(httpBuf, 0, sizeof(httpBuf));

  if(!modem.httpReceive(0, httpBuf, sizeof(httpBuf) - 1, &rsp)) {
    Serial.println("HTTP receive failed");
    return false;
  }

  String response = String((char*)httpBuf);
  Serial.println("Google response:");
  Serial.println(response);

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, response);
  if(err) {
    Serial.printf("JSON parse failed: %s\r\n", err.c_str());
    return false;
  }

  if(!doc["location"]["lat"].is<float>() ||
     !doc["location"]["lng"].is<float>() ||
     !doc["accuracy"].is<float>()) {
    Serial.println("Invalid geolocation response");
    return false;
  }

  last_wifi_lat = doc["location"]["lat"].as<float>();
  last_wifi_lon = doc["location"]["lng"].as<float>();
  last_wifi_acc = doc["accuracy"].as<float>();
  last_wifi_ok = true;

  Serial.printf("LTE WiFi+Cell location: %.6f, %.6f (±%.1fm)\r\n",
                last_wifi_lat,
                last_wifi_lon,
                last_wifi_acc);

  return true;
}

bool sameApproxLocationAsCachedWifi()
{
  if(!cachedWifiLocationValid) {
    return false;
  }

  if(lastCell.cc != cachedWifiCc) return false;
  if(lastCell.nc != cachedWifiNc) return false;
  if(lastCell.tac != cachedWifiTac) return false;
  if(lastCell.cid != cachedWifiCid) return false;

  float rsrpDelta = fabs(lastCell.rsrp - cachedWifiRsrp);
  if(rsrpDelta > 10.0f) return false;

  return true;
}

void cacheWifiFallbackLocation()
{
  cachedWifiLat = last_wifi_lat;
  cachedWifiLon = last_wifi_lon;
  cachedWifiAcc = last_wifi_acc;
  cachedWifiApCount = last_wifi_ap_count;
  cachedWifiLocationValid = true;

  cachedWifiCc = lastCell.cc;
  cachedWifiNc = lastCell.nc;
  cachedWifiTac = lastCell.tac;
  cachedWifiCid = lastCell.cid;
  cachedWifiRsrp = lastCell.rsrp;
}

// -----------------------------------------------------------------------------
// Event handlers
// -----------------------------------------------------------------------------

static void myNetworkEventHandler(WMNetworkEventType event, const WMNetworkEventData* data, void* args)
{
  if(event == WALTER_MODEM_NETWORK_EVENT_REG_STATE_CHANGE) {
    switch(data->cereg.state) {
      case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
        Serial.println("Network registration: Registered (home)");
        break;
      case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
        Serial.println("Network registration: Registered (roaming)");
        break;
      case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:
        Serial.println("Network registration: Not searching");
        break;
      case WALTER_MODEM_NETWORK_REG_SEARCHING:
        Serial.println("Network registration: Searching");
        break;
      case WALTER_MODEM_NETWORK_REG_DENIED:
        Serial.println("Network registration: Denied");
        break;
      case WALTER_MODEM_NETWORK_REG_UNKNOWN:
        Serial.println("Network registration: Unknown");
        break;
      default:
        break;
    }

    if(data->cereg.hasPsmInfo) {
      Serial.printf("Network granted PSM: ActiveTime=%s, PeriodicTAU=%s\r\n",
                    data->cereg.activeTime,
                    data->cereg.periodicTau);
    }
  }
}

static void myMQTTEventHandler(WMMQTTEventType event, const WMMQTTEventData* data, void* args)
{
  switch(event) {
    case WALTER_MODEM_MQTT_EVENT_CONNECTED:
      if(data->rc != 0) {
        Serial.printf("MQTT: Connection could not be established. (code: %d)\r\n", data->rc);
      } else {
        Serial.println("MQTT: Connected successfully");
        mqtt_connected = true;
      }
      break;

    case WALTER_MODEM_MQTT_EVENT_DISCONNECTED:
      if(data->rc != 0) {
        Serial.printf("MQTT: Connection was interrupted (code: %d)\r\n", data->rc);
      } else {
        Serial.println("MQTT: Disconnected");
      }
      mqtt_connected = false;
      break;

    case WALTER_MODEM_MQTT_EVENT_PUBLISHED:
    if(data->rc != 0) {
      Serial.printf("MQTT: Could not publish message (id: %d, code: %d)\r\n",
                    data->mid, data->rc);
      mqtt_publish_ok = false;
    } else {
      Serial.printf("MQTT: Successfully published message (id: %d)\r\n", data->mid);
      mqtt_publish_ok = true;
    }
    mqtt_publish_done = true;
    break;

    case WALTER_MODEM_MQTT_EVENT_MESSAGE:
      Serial.printf("MQTT: Message (id: %d) received on topic '%s' (%u bytes)\r\n",
                    data->mid, data->topic, data->msg_length);

      memset(in_buf, 0, sizeof(in_buf));
      if(modem.mqttReceive(data->topic, data->mid, in_buf, data->msg_length)) {
        Serial.printf("Received message: %s\r\n", in_buf);
      } else {
        Serial.println("Could not receive MQTT message");
      }
      break;

    case WALTER_MODEM_MQTT_EVENT_SUBSCRIBED:
    case WALTER_MODEM_MQTT_EVENT_MEMORY_FULL:
    default:
      break;
  }
}

void myGNSSEventHandler(WMGNSSEventType type, const WMGNSSEventData* data, void* args)
{
  uint8_t goodSatCount = 0;

  switch(type) {
    case WALTER_MODEM_GNSS_EVENT_FIX:
      memcpy(&latestGnssFix, &data->gnssfix, sizeof(WMGNSSFixEvent));

      for(int i = 0; i < latestGnssFix.satCount; ++i) {
        if(latestGnssFix.sats[i].signalStrength >= 30) {
          ++goodSatCount;
        }
      }

      Serial.printf(
        "\r\nGNSS fix received: Confidence=%.02f Latitude=%.06f Longitude=%.06f Satcount=%d Good sats=%d\r\n",
        latestGnssFix.estimatedConfidence,
        latestGnssFix.latitude,
        latestGnssFix.longitude,
        latestGnssFix.satCount,
        goodSatCount
      );

      gnss_fix_received = true;
      break;

    case WALTER_MODEM_GNSS_EVENT_ASSISTANCE:
      if(data->assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC) {
        Serial.println("GNSS Assistance: Almanac updated");
      } else if(data->assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS) {
        Serial.println("GNSS Assistance: Real-time ephemeris updated");
      } else if(data->assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_PREDICTED_EPHEMERIS) {
        Serial.println("GNSS Assistance: Predicted ephemeris updated");
      }

      assistance_update_received = true;
      break;

    default:
      break;
  }
}

void myHTTPEventHandler(WMHTTPEventType event, const WMHTTPEventData* data, void* args)
{
  switch(event) {
    case WALTER_MODEM_HTTP_EVENT_RING:
      http_ring_received = true;
      http_ring_status = data->status;
      http_ring_length = data->data_len;
      http_ring_profile = data->profile_id;

      Serial.printf("HTTP RING: profile=%u status=%u len=%u\r\n",
                    data->profile_id,
                    data->status,
                    data->data_len);
      break;

    case WALTER_MODEM_HTTP_EVENT_CONNECTED:
      Serial.printf("HTTP connected: profile=%u rc=%u\r\n",
                    data->profile_id,
                    data->rc);
      break;

    case WALTER_MODEM_HTTP_EVENT_DISCONNECTED:
      Serial.printf("HTTP disconnected: profile=%u\r\n", data->profile_id);
      break;

    case WALTER_MODEM_HTTP_EVENT_CONNECTION_CLOSED:
      Serial.printf("HTTP connection closed: profile=%u\r\n", data->profile_id);
      break;
  }
}

// -----------------------------------------------------------------------------
// GNSS / Assistance
// -----------------------------------------------------------------------------

bool checkAssistanceStatus(WalterModemRsp* rsp, bool* updateAlmanac = nullptr, bool* updateEphemeris = nullptr)
{
  if(!modem.gnssGetAssistanceStatus(rsp) ||
     rsp->type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.println("Could not request GNSS assistance status");
    return false;
  }

  if(updateAlmanac) *updateAlmanac = false;
  if(updateEphemeris) *updateEphemeris = false;

  auto report = [](const char* name, const WMGNSSAssistance& data, bool* updateFlag) {
    Serial.printf("%s data is ", name);

    if(data.available) {
      Serial.printf("available and should be updated within %lds\r\n", data.timeToUpdate);
      if(updateFlag) *updateFlag = (data.timeToUpdate <= 0);
    } else {
      Serial.println("not available.");
      if(updateFlag) *updateFlag = true;
    }
  };

  const WMGNSSAssistance& almanac =
      rsp->data.gnssAssistance[WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC];
  const WMGNSSAssistance& rtEph =
      rsp->data.gnssAssistance[WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS];

  report("Almanac", almanac, updateAlmanac);
  report("Real-time ephemeris", rtEph, updateEphemeris);
  return true;
}

bool validateGNSSClock(WalterModemRsp* rsp)
{
  modem.gnssGetUTCTime(rsp);
  if(rsp->data.clock.epochTime > 4) {
    return true;
  }

  Serial.println("System clock invalid, LTE time sync required");

  if(!lteConnected() && !lteConnect()) {
    Serial.println("Error: Could not connect to LTE network");
    return false;
  }

  for(int i = 0; i < 5; ++i) {
    modem.gnssGetUTCTime(rsp);
    if(rsp->data.clock.epochTime > 4) {
      Serial.printf("Clock synchronized: %" PRIi64 "\r\n", rsp->data.clock.epochTime);
      return true;
    }
    delay(2000);
  }

  Serial.println("Error: Could not sync time with network");
  return false;
}

bool updateGNSSAssistance(WalterModemRsp* rsp)
{
  bool updateAlmanac = false;
  bool updateEphemeris = false;

  if(!checkAssistanceStatus(rsp, &updateAlmanac, &updateEphemeris)) {
    Serial.println("Error: Could not check GNSS assistance status");
    return false;
  }

  if(!updateAlmanac && !updateEphemeris) {
    return true;
  }

  if(!lteConnected() && !lteConnect()) {
    Serial.println("Could not connect to LTE network");
    return false;
  }

  assistance_update_received = false;
  if(updateAlmanac && !modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC)) {
    Serial.println("Could not update almanac data");
    return false;
  }

  uint32_t startMs = millis();
  while(updateAlmanac && !assistance_update_received) {
    if(millis() - startMs >= 30000) {
      Serial.println("Timeout waiting for almanac update");
      return false;
    }
    delay(200);
  }

  assistance_update_received = false;
  if(updateEphemeris &&
     !modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS)) {
    Serial.println("Could not update real-time ephemeris data");
    return false;
  }

  startMs = millis();
  while(updateEphemeris && !assistance_update_received) {
    if(millis() - startMs >= 30000) {
      Serial.println("Timeout waiting for ephemeris update");
      return false;
    }
    delay(200);
  }

  if(!checkAssistanceStatus(rsp)) {
    Serial.println("Error: Could not check GNSS assistance status");
    return false;
  }

  return true;
}

bool attemptGNSSFix()
{
  WalterModemRsp rsp = {};

  if(!validateGNSSClock(&rsp)) {
    Serial.println("Error: Could not validate GNSS clock");
    return false;
  }

  if(!updateGNSSAssistance(&rsp)) {
    Serial.println("Warning: Could not update GNSS assistance data. Continuing without assistance.");
  }

  if(lteConnected() && !lteDisconnect()) {
    Serial.println("Error: Could not disconnect from the LTE network");
    return false;
  }

  mqtt_connected = false;

  Serial.println("Settling after LTE disconnect to improve GNSS fix...");
  delay(3000);

  if(latestGnssFix.estimatedConfidence > 0 &&
     latestGnssFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE &&
     latestGnssFix.latitude != 0.0 &&
     latestGnssFix.longitude != 0.0 &&
     latestGnssFix.satCount > 0) {
    if(modem.gnssConfig(WALTER_MODEM_GNSS_SENS_MODE_HIGH, WALTER_MODEM_GNSS_ACQ_MODE_HOT_START)) {
      Serial.println("GNSS reconfigured for potential quick fix");
    } else {
      Serial.println("Error: Could not reconfigure GNSS");
    }
  }

  const int maxAttempts = 2;
  for(int attempt = 0; attempt < maxAttempts; ++attempt) {
    gnss_fix_received = false;

    if(!modem.gnssPerformAction()) {
      Serial.println("Error: Could not request GNSS fix");
      return false;
    }

    Serial.printf("Started GNSS fix (attempt %d/%d)\r\n", attempt + 1, maxAttempts);

    uint32_t attemptStartMs = millis();

    while(!gnss_fix_received) {
      Serial.print(".");
      delay(500);

      if(millis() - attemptStartMs >= 60000) {  // 1 minute
        Serial.println("\r\nGNSS attempt timed out");

        if(!modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_CANCEL)) {
          Serial.println("Warning: Could not cancel GNSS action");
        } else {
          Serial.println("Cancelled GNSS action");
        }
      
        delay(500);  // give the modem a moment to settle
        break;
      }
    }
    Serial.println();

    if(!gnss_fix_received) {
      Serial.printf("No GNSS fix event received in attempt %d\r\n", attempt + 1);
      continue;   // go to next attempt
    }

    if(latestGnssFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE) {
      Serial.println("Successfully obtained a valid GNSS fix");
      return true;
    }

    Serial.printf("GNSS fix confidence %.02f too low, retrying...\r\n",
                  latestGnssFix.estimatedConfidence);
  }

  return false;
}

// -----------------------------------------------------------------------------
// WiFi Geolocation
// -----------------------------------------------------------------------------

bool tryWiFiGeolocation()
{
  last_wifi_ok = false;

  Serial.println("Scanning WiFi for geolocation...");

  WiFi.mode(WIFI_STA);
  delay(200);

  int found = WiFi.scanNetworks(false, true);

  if(found < WIFI_SCAN_MIN_APS) {
    Serial.println("Not enough APs for geolocation");
    WiFi.scanDelete();
    WiFi.mode(WIFI_OFF);
    return false;
  }

  String payload = buildGoogleGeoPayload(found);

  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);

  return googleGeolocateLTE(payload);
}

// -----------------------------------------------------------------------------
// Setup / Loop
// -----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(2000);

  bootCount++;
  Serial.printf("\r\n\r\n=== WalterModem MQTT Positioning + Deep Sleep + PSM ===\r\n");
  Serial.printf("Boot count: %lu\r\n", (unsigned long)bootCount);

  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("Walter MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if(!modem.begin(&Serial2)) {
    Serial.println("Error: Could not initialize the modem");
    goToDeepSleep(RETRY_INTERVAL_SECONDS);
  }

  Serial.println("Successfully initialized the modem");

  modem.setNetworkEventHandler(myNetworkEventHandler, NULL);
  modem.setMQTTEventHandler(myMQTTEventHandler, NULL);
  modem.setGNSSEventHandler(myGNSSEventHandler, NULL);
  modem.setHTTPEventHandler(myHTTPEventHandler, NULL);

  configurePSM();

  if(modem.mqttConfig(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
    Serial.println("Successfully configured the MQTT client");
  } else {
    Serial.println("Error: Failed to configure MQTT client");
    goToDeepSleep(RETRY_INTERVAL_SECONDS);
  }

  WalterModemRsp rsp = {};

  if(modem.getRAT(&rsp)) {
    if(rsp.data.rat != RADIO_TECHNOLOGY) {
      modem.setRAT(RADIO_TECHNOLOGY);
      Serial.println("Switched modem radio technology");
    }
  } else {
    Serial.println("Error: Could not retrieve radio access technology");
  }

  if(modem.getIdentity(&rsp)) {
    Serial.println("Modem identity:");
    Serial.printf(" - IMEI: %s\r\n", rsp.data.identity.imei);
    Serial.printf(" - IMEISV: %s\r\n", rsp.data.identity.imeisv);
    Serial.printf(" - SVN: %s\r\n", rsp.data.identity.svn);
  }

  if(modem.getSIMCardID(&rsp)) {
    Serial.println("SIM card identity:");
    Serial.printf(" - ICCID: %s\r\n", rsp.data.simCardID.iccid);
    Serial.printf(" - eUICCID: %s\r\n", rsp.data.simCardID.euiccid);
  }

  if(modem.getSIMCardIMSI(&rsp)) {
    Serial.printf("Active IMSI: %s\r\n", rsp.data.imsi);
  }

  if(!modem.gnssConfig()) {
    Serial.println("Error: Could not configure the GNSS subsystem");
    goToDeepSleep(RETRY_INTERVAL_SECONDS);
  }
}

void loop()
{
  static uint32_t loopStartMs = 0;

  if(millis() - loopStartMs > 300000) {  // 5 minutes max runtime
    Serial.println("Failsafe: loop running too long, forcing sleep");
    goToDeepSleep(RETRY_INTERVAL_SECONDS);
  }

  WalterModemRsp rsp = {};

  if(!publish_pending) {
    publishCycle++;
    loopStartMs = millis();

    // -----------------------------
    // 1. Try GNSS first
    // -----------------------------
    last_gnss_ok = attemptGNSSFix();

    if(last_gnss_ok) {
      gnssFailStreak = 0;
      last_wifi_ok = false;
    } else {
      gnssFailStreak++;
      last_wifi_ok = false;
      Serial.printf("GNSS fail streak: %lu\r\n", (unsigned long)gnssFailStreak);
      Serial.println("Warning: GNSS fix failed, continuing with publish");
    }

    mqtt_connected = false;

    // -----------------------------
    // 2. Reconnect LTE
    // -----------------------------
    if(!lteConnected()) {
      if(!lteConnect()) {
        Serial.println("Error: Could not connect to the LTE network");
        publish_pending = false;
        goToDeepSleep(RETRY_INTERVAL_SECONDS);
      }
    }

    // -----------------------------
    // 3. Read current cell info now that LTE is up
    //    and cache it for Wi-Fi + cell geolocation
    // -----------------------------
    bool haveCellNow =
        modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp);

    if(haveCellNow) {
      lastCell.cc = rsp.data.cellInformation.cc;
      lastCell.nc = rsp.data.cellInformation.nc;
      lastCell.tac = rsp.data.cellInformation.tac;
      lastCell.cid = rsp.data.cellInformation.cid;
      lastCell.rsrp = rsp.data.cellInformation.rsrp;
    } else {
      Serial.println("Warning: Could not get cell info before Wi-Fi fallback");
    }

    // -----------------------------
    // 4. Wi-Fi fallback after 3 GNSS failures in a row
    //    (now LTE is available for the Google request)
    // -----------------------------
    if(!last_gnss_ok &&
       gnssFailStreak >= WIFI_FALLBACK_AFTER_GNSS_FAILS &&
       haveCellNow) {
    
      if(sameApproxLocationAsCachedWifi()) {
        Serial.println("GNSS still unavailable, same approximate cell location detected - reusing cached Wi-Fi geolocation");
    
        last_wifi_ok = true;
        last_wifi_lat = cachedWifiLat;
        last_wifi_lon = cachedWifiLon;
        last_wifi_acc = cachedWifiAcc;
        last_wifi_ap_count = cachedWifiApCount;
    
      } else if((publishCycle - lastWifiGeoCycle) >= WIFI_FALLBACK_AFTER_GNSS_FAILS) {
        Serial.printf("GNSS failed %d cycles in a row, trying fresh Wi-Fi + cell geolocation fallback...\r\n",
                      WIFI_FALLBACK_AFTER_GNSS_FAILS);
    
        if(tryWiFiGeolocation()) {
          lastWifiGeoCycle = publishCycle;
          cacheWifiFallbackLocation();
        }
      }
    }

    publish_pending = true;
  }
  // -----------------------------
  // 5. Connect MQTT
  // -----------------------------
  if(!mqtt_connected) {
    if(modem.mqttConnect(MQTT_HOST, MQTT_PORT)) {
      Serial.println("Connecting to MQTT broker...");
      delay(5000);
      return;
    } else {
      Serial.println("Error: Failed to connect to MQTT broker");
      publish_pending = false;
      goToDeepSleep(RETRY_INTERVAL_SECONDS);
    }
  }

  // -----------------------------
  // 6. Gather current telemetry for publish
  // -----------------------------
  uint8_t rat = 0xFF;
  if(modem.getRAT(&rsp)) {
    rat = (uint8_t)rsp.data.rat;
  }

  bool haveCell = modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp);
  float temp = temperatureRead();

  if(haveCell) {
    lastCell.cc = rsp.data.cellInformation.cc;
    lastCell.nc = rsp.data.cellInformation.nc;
    lastCell.tac = rsp.data.cellInformation.tac;
    lastCell.cid = rsp.data.cellInformation.cid;
    lastCell.rsrp = rsp.data.cellInformation.rsrp;
  }

  // -----------------------------
  // 7. Build payload
  // -----------------------------
  char out_msg[512];

  if(last_gnss_ok && haveCell) {
    snprintf(
      out_msg,
      sizeof(out_msg),
      "{\"seq\":%d,\"loc_source\":\"gnss\",\"gnss_ok\":true,\"gnss_acc\":%.2f,"
      "\"lat\":%.6f,\"lon\":%.6f,\"temp\":%.2f,"
      "\"sat\":%u,\"rat\":%u,\"rsrp\":%.2f,"
      "\"cc\":%u,\"nc\":%u,\"tac\":%u,\"cell\":%u}",
      publishCycle,
      latestGnssFix.estimatedConfidence,
      latestGnssFix.latitude,
      latestGnssFix.longitude,
      temp,
      latestGnssFix.satCount,
      rat,
      rsp.data.cellInformation.rsrp,
      rsp.data.cellInformation.cc,
      rsp.data.cellInformation.nc,
      rsp.data.cellInformation.tac,
      rsp.data.cellInformation.cid
    );
  } else if(last_wifi_ok && haveCell) {
    snprintf(
      out_msg,
      sizeof(out_msg),
      "{\"seq\":%d,\"loc_source\":\"wifi\",\"gnss_ok\":false,"
      "\"wifi_ok\":true,\"wifi_acc\":%.2f,\"wifi_aps\":%u,"
      "\"lat\":%.6f,\"lon\":%.6f,\"temp\":%.2f,"
      "\"rat\":%u,\"rsrp\":%.2f,"
      "\"cc\":%u,\"nc\":%u,\"tac\":%u,\"cell\":%u}",
      publishCycle,
      last_wifi_acc,
      last_wifi_ap_count,
      last_wifi_lat,
      last_wifi_lon,
      temp,
      rat,
      rsp.data.cellInformation.rsrp,
      rsp.data.cellInformation.cc,
      rsp.data.cellInformation.nc,
      rsp.data.cellInformation.tac,
      rsp.data.cellInformation.cid
    );
  } else if(!last_gnss_ok && haveCell) {
    snprintf(
      out_msg,
      sizeof(out_msg),
      "{\"seq\":%d,\"loc_source\":\"cell\",\"gnss_ok\":false,"
      "\"temp\":%.2f,\"rat\":%u,\"rsrp\":%.2f,"
      "\"cc\":%u,\"nc\":%u,\"tac\":%u,\"cell\":%u}",
      publishCycle,
      temp,
      rat,
      rsp.data.cellInformation.rsrp,
      rsp.data.cellInformation.cc,
      rsp.data.cellInformation.nc,
      rsp.data.cellInformation.tac,
      rsp.data.cellInformation.cid
    );
  } else {
    snprintf(
      out_msg,
      sizeof(out_msg),
      "{\"seq\":%d,\"loc_source\":\"none\",\"gnss_ok\":false,"
      "\"temp\":%.2f,\"rat\":%u}",
      publishCycle,
      temp,
      rat
    );
  }

  // -----------------------------
  // 8. Publish and wait for publish confirmation
  // -----------------------------
  mqtt_publish_done = false;
  mqtt_publish_ok = false;

  if(!mqttPublishMessage(MQTT_TOPIC, out_msg)) {
    Serial.println("MQTT publish failed");
    mqtt_connected = false;
    publish_pending = false;
    goToDeepSleep(RETRY_INTERVAL_SECONDS);
  }

  unsigned long publishStart = millis();
  while(!mqtt_publish_done && millis() - publishStart < 10000) {
    delay(100);
  }

  if(!mqtt_publish_done || !mqtt_publish_ok) {
    Serial.println("MQTT publish was not confirmed before timeout");
    mqtt_connected = false;
    publish_pending = false;
    goToDeepSleep(RETRY_INTERVAL_SECONDS);
  }

  // -----------------------------
  // 9. Sleep
  // -----------------------------
  publish_pending = false;
  goToDeepSleep(PUBLISH_INTERVAL_SECONDS);
}
