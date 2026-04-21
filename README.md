
Walter Multi-Source MQTT Location Tracker

2026 - nmc

Tracker uses multiple location sources in stages - GNSS as
1st priority, CellID location data as 2nd then if all else 
fails, uses Google Geolocation using WiFi AP + CellID data &
publishes results via MQTT at set interval (adjustable below).

Walter is also setup to use LTE-M PSM (power saving) & deep
sleep between publish interval.

The WalterModem libraries and example snippets are all
Copyright (C), DP Technics bv:

https://github.com/QuickSpot/walter-arduino/raw/refs/heads/main/LICENSE

