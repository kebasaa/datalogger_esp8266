#pragma once
// ============================================================================
//  Datalogger framework internals: h4setup() and HTTPClient()
//  Auto-relocated verbatim from the old monolithic main.cpp. Included ONLY by
//  datalogger.h (one translation unit). Beginners never need to open this.
// ============================================================================

void h4setup(){
  // Heap on entry, i.e. after H4 has started and H4P_ConfigService has loaded the
  // stored globals from LittleFS but before anything here runs. Compare against
  // the figure printed at the end of init to see what setup itself costs: this
  // build has little headroom and H4AsyncTCP refuses connections when free heap
  // drops below H4AT_HEAP_THROTTLE_LO + _heap_alloc.
  Serial.printf("Free heap at setup: %u bytes (largest block %u)\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxFreeBlockSize());

  // Bind each sensor object to its multiplexer channel, build the channel union,
  // and size calibration storage. Must run before any sensor use / header build.
  bind_sensors();

  // Show info on WIFI
  Serial.print(wifitype);
#if !H4P_USE_WIFI_AP
  Serial.print(F(" connecting to ")); Serial.println(WIFI_SSID);
#else
  Serial.println(F(" connecting")); 
#endif

#if USE_MQTT
	h4p[brokerTag()]=MQTT_SERVER;
	for (int i=0;i<BIG_SIZE;i++) {
		big[i]=i;
	}
#endif

#if H4P_SECURE
#if USE_HTTPREQ && SECURE_HTTPREQ
	auto testRootCA = reinterpret_cast<const uint8_t*>(const_cast<char*>(test_root_ca.c_str()));
	h4ah.secureTLS(testRootCA, test_root_ca.length() + 1);
	Serial.printf("HTTP CERT Validation: %s\n", H4AsyncClient::isCertValid(testRootCA, test_root_ca.length() + 1) ? "SUCCEEDED" : "Failed");
#endif // SECURE_HTTPREQ

#if USE_MQTT && SECURE_MQTT
	auto mqCert = reinterpret_cast<const uint8_t*>(const_cast<char*>(MQTT_CERT.c_str()));
	Serial.printf("MQTT CERT Validation: %s\n", H4AsyncClient::isCertValid(mqCert,MQTT_CERT.length()+1) ? "SUCCEEDED" : "Failed");
	h4mqtt.secureTLS(mqCert, MQTT_CERT.length()+1);
#endif // USE_MQTT

#if SECURE_WEBSERVER
	Serial.printf("WEBSERVER CERT Validation: %s\n", H4AsyncClient::isCertValid((const uint8_t*)WEBSERVER_CERT.c_str(), WEBSERVER_CERT.length() + 1) ? "SUCCEEDED" : "Failed");
	Serial.printf("WEBSERVER KEY Validation: %s\n", H4AsyncClient::isPrivKeyValid((const uint8_t*)WEBSERVER_PRIV_KEY.c_str(), WEBSERVER_PRIV_KEY.length() + 1) ? "SUCCEEDED" : "Failed");
	h4wifi.hookWebserver([](){
		h4wifi.secureTLS((const uint8_t*)WEBSERVER_PRIV_KEY.c_str(), WEBSERVER_PRIV_KEY.length() + 1, 
							NULL, 0,
						(const uint8_t*)WEBSERVER_CERT.c_str(), WEBSERVER_CERT.length() + 1);
		h4wifi.useSecurePort();
	});
#endif // SECURE_WEBSERVER

#endif // H4P_SECURE

  // HTTP basic authentication, in station mode only.
  //
  // On a shared network anyone routable can reach the dashboard, so the password
  // matters. In access-point mode it does not: getting far enough to make a
  // request already means knowing the WPA2 hotspot password. What it does cost
  // there is real - an extra 401 round-trip on a whole separate socket for every
  // page load, against an lwIP built with MEMP_NUM_TCP_PCB 5 - and Android opens
  // captive portals in a restricted webview that handles basic-auth prompts
  // badly, so the sign-in page would often just fail to appear.
#if !H4P_USE_WIFI_AP
	h4wifi.authenticate(DATALOGGER_UI_USER, DATALOGGER_UI_PASSWORD);
#endif

  // Register the SPA and the JSON API. This has to go through hookWebserver
  // rather than run here: H4P_WiFi::_startWebserver() destroys every handler
  // and rebuilds them on each WiFi transition, so routes registered once at
  // setup would disappear the first time the device switched between AP and
  // station mode.
  h4wifi.hookWebserver(dl_register_api);

#if USE_MQTT
	h4.every(300, []()
			 {
#if defined(ARDUINO_ARCH_ESP32)
				Serial.printf("H=%u M=%u m=%u S=%u\n", _HAL_freeHeap(MALLOC_CAP_INTERNAL), _HAL_maxHeapBlock(MALLOC_CAP_INTERNAL), _HAL_minHeapBlock(MALLOC_CAP_INTERNAL), uxTaskGetStackHighWaterMark(NULL));
#else
				Serial.printf("H=%u M=%u m=%u\n", _HAL_freeHeap(), _HAL_maxHeapBlock(), _HAL_minHeapBlock());
#endif
				h4p["heap"] = _HAL_freeHeap();
				h4p["pool"] = mbx::pool.size();
				});
#endif

  Serial.println("Starting v"WS_VERSION);
  boot_ms = millis();
#if defined(ARDUINO_ARCH_ESP8266)
  boot_id = ESP.getChipId() ^ micros();
  sanitize_csv_text(ESP.getResetReason(), reset_reason_buf, sizeof(reset_reason_buf));
#else
  boot_id = micros();
  sanitize_csv_text(String("unknown"), reset_reason_buf, sizeof(reset_reason_buf));
#endif
  RtcRestartRecord rtc_record;
  rtc_time_valid = read_rtc_restart_record(rtc_record);
  if (rtc_time_valid) {
    rtc_boot_epoch = rtc_record.epoch;
    strncpy(boot_time_source_buf, "waiting", sizeof(boot_time_source_buf) - 1);
    boot_time_source_buf[sizeof(boot_time_source_buf) - 1] = '\0';
  }

  Serial.println(F(""));
  Serial.println(F("Initialisation:"));

  // The calibration command set. This is the ONLY calibration interface: the
  // web UI reaches exactly these commands over /rest/<command>/<params>, so
  // anything possible in the browser is possible over a serial cable, and vice
  // versa. "cal" and "set" only work between cal_start and cal_stop.
  h4p.addCmd("cal", 0, 0, cal_cmd);
  h4p.addCmd("set", 0, 0, cal_set_cmd);
  h4p.addCmd("show_cal", 0, 0, cal_show_cmd);
  h4p.addCmd("cal_start", 0, 0, cal_start_cmd);
  h4p.addCmd("cal_stop", 0, 0, cal_stop_cmd);
  h4p.addCmd("cal_discard", 0, 0, cal_discard_cmd);
  h4p.addCmd("cal_reset", 0, 0, cal_reset_cmd);
  h4p.addCmd("cal_select", 0, 0, cal_select_cmd);
  // Start the clock by hand so the logger can record before it has a GPS fix.
  h4p.addCmd("settime", 0, 0, settime_cmd);

  // Non-i2c devices
#if RUN_TEST
  Serial.print(F("- TEST:                     "));
  Serial.println(test.init() ? F("Success") : F("Failed"));
#endif

  // SPI devices
#if USE_MICROSD
Serial.print(F("- MicroSD:                  "));
  if(! sd.init()){
    Serial.println(F("Failed (SD card missing)"));
  } else {
    Serial.println(F("Success"));
    // Latch these for the status page as well as printing them. They used to be
    // filled in only after a successful data write, but writing is gated on a
    // valid GPS time - so a logger still waiting for a fix reported "-1 MB" for
    // both, which reads as a broken card rather than "not measured yet". We are
    // already paying for the SPI walk here, so keep the answer.
    dl_sd_cap_mb  = sd.getCapacityMB();
    dl_sd_free_mb = sd.getFreeMB();
    Serial.print("  Storage capacity:      "); Serial.print(dl_sd_cap_mb, 2); Serial.println(" MB");
    Serial.print("  Free storage capacity: "); Serial.print(dl_sd_free_mb, 2); Serial.println(" MB");
  }
#endif

  // Analogue devices
#if USE_BATTERY
  Serial.print(F("- Battery:                  "));
  Serial.println(bat.init() ? F("Success") : F("Failed (Battery not detected)"));
#endif

  // A slave left mid-transaction by a previous reset holds the bus down and stays
  // that way across reboots, because the ESP resets but its sensors do not. Clear
  // it before probing anything, so this recovers by itself rather than needing
  // somebody to walk to the logger and pull the power.
  Wire.begin();
  i2c_bus_ok = dl_i2c_bus_reset();
  Serial.print(F("- I2C bus:                  "));
  Serial.println(i2c_bus_ok
    ? F("Ready")
    : F("STUCK (SCL held low - power-cycle the logger)"));

  // i2c multiplexer
#if I2C_MULTI
  Serial.print(F("- i2c multiplexer           "));
  if(! i2c_bus_ok){
    Serial.println(F("Skipped (bus stuck)"));
  } else if(! mp.init()){
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Success"));
  }
#endif

#if USE_GPS
  Serial.print(F("  - XA1110 GPS:             "));
  Serial.println(!i2c_bus_ok ? F("Skipped (bus stuck)")
                             : gps.init() ? F("Success") : F("Failed"));
  dl_i2c_apply_limits();   // SparkFun_I2C_GPS calls Wire.begin(): resets the limit
#endif

// Creates the list of everything that can be calibrated
#if USE_BME280
  quantities.push_back("temperature"); // Add to the quantities list
  quantities.push_back("h2o"); // Add to the quantities list
  quantities.push_back("pressure"); // Add to the quantities list
#endif
#if USE_SCD30
  quantities.push_back("co2"); // Add to the quantities list
#endif
#if USE_SEN0465
  quantities.push_back("o2"); // Add to the quantities list
#endif
#if USE_CAL
  // Bind the calibration value store now that the quantity list is complete, and
  // load /cal.dat. Must happen before anything reads a calibration. Values live
  // in a fixed array rather than H4's globals map: the map cost ~120 bytes an
  // entry and a fully calibrated logger has 60 of them, which is most of the
  // heap the access point needs. Also migrates anything an older firmware left
  // in the globals, once.
  cal.storage_begin(quantities, numSensors);
#endif

  // Initialise each sensor instance (type-major, one line per bus it sits on).
#if USE_BME280
  for (size_t i = 0; i < BME_COUNT; ++i) {
    Serial.printf("  - BME280 sensor (bus %u):    ", (unsigned)bme_buses[i]);
    Serial.println(!i2c_bus_ok ? F("Skipped (bus stuck)")
                               : bme_sensors[i]->init() ? F("Success") : F("Failed"));
  }
#endif

#if USE_SCD30
  for (size_t i = 0; i < SCD_COUNT; ++i) {
    Serial.printf("  - SCD-30 sensor (bus %u):    ", (unsigned)scd_buses[i]);
    if(!i2c_bus_ok){ Serial.println(F("Skipped (bus stuck)")); continue; }
    Serial.println(scd_sensors[i]->init() ? F("Success") : F("Failed"));
    scd_sensors[i]->enable_self_calibration(false); // don't calibrate, we want to do that manually
    scd_sensors[i]->set_interval(2);                // minimum every 2s
  }
#endif

#if USE_SEN0465
  for (size_t i = 0; i < SEN_COUNT; ++i) {
    Serial.printf("  - SEN0465 sensor (bus %u):   ", (unsigned)sen_buses[i]);
    Serial.println(!i2c_bus_ok ? F("Skipped (bus stuck)")
                               : sen_sensors[i]->init() ? F("Success") : F("Failed"));
  }
#endif

#if USE_ADS1115
  for (size_t i = 0; i < ADS_COUNT; ++i) {
    Serial.printf("  - ADS1115 (bus %u):          ", (unsigned)ads_buses[i]);
    Serial.println(!i2c_bus_ok ? F("Skipped (bus stuck)")
                               : ads_sensors[i]->init() ? F("Success") : F("Failed"));
  }
#endif

#if USE_MLX90614
  for (size_t i = 0; i < MLX_COUNT; ++i) {
    Serial.printf("  - MLX90614 sensor (bus %u):  ", (unsigned)mlx_buses[i]);
    Serial.println(!i2c_bus_ok ? F("Skipped (bus stuck)")
                               : mlx_sensors[i]->init() ? F("Success") : F("Failed"));
  }
#endif

  // Several drivers call Wire.begin() from their own init(), which resets the
  // clock-stretch limit to the 150 ms core default. Re-apply it now that the last
  // one has run, otherwise the bound is never actually in force at runtime.
  dl_i2c_apply_limits();

  Serial.println(F("Initialisation completed"));
  Serial.println(F(""));

  // In AP mode localIP() is the (unused) station address and prints as
  // "(IP unset)", which reads like a failure. Report the address clients will
  // actually use.
  WiFiMode_t wm = WiFi.getMode();
  Serial.print(F("IP address: "));
  Serial.println((wm == WIFI_AP || wm == WIFI_AP_STA) ? WiFi.softAPIP() : WiFi.localIP());
  // This build runs close to the heap floor; H4AsyncTCP stops accepting
  // connections below ~13 KB. Worth seeing on every boot report.
  Serial.printf("Free heap: %u bytes (largest block %u)\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxFreeBlockSize());

  build_data_header();

#if USE_GPS
  // Drain the GPS FIFO in bounded slices. This keeps Wi-Fi, the web UI and the
  // sensor sampler responsive even when the GPS delivers a burst of NMEA data.
  gps_poll_timer = h4.every(GPS_POLL_INTERVAL_MS, [](){
    if (gps_quarantine_active) return;
    uint32_t chars = gps.update_values(GPS_SLICE_MAX_MS, GPS_SLICE_MAX_CHARS);
    gps_chars_since_sample += chars;
    if (gps.pollTimedOut()) gps_poll_timeout_count++;
  });
#endif

  // --- TEMPORARY: access-point heartbeat -----------------------------------
  // Diagnostic for "the phone sees the hotspot but never gets an IP address".
  // The station count is the discriminator: if it never reaches 1 the phone is
  // not associating at all (beacon, channel or WPA2 handshake); if it reaches 1
  // and drops back with no lease, association works and the DHCP exchange is
  // what is failing, which on this board points at heap. Remove once the AP is
  // known good - it costs a print every 2 s.
  h4.every(2000, [](){
    WiFiMode_t m = WiFi.getMode();
    if(m != WIFI_AP && m != WIFI_AP_STA) return;
    // H4AsyncServer::checkMemory() refuses a connection - and lwIP then sends RST,
    // which is what a browser calls "the connection was reset" - unless free heap
    // exceeds H4AT_HEAP_THROTTLE_LO + _heap_alloc AND the largest block exceeds
    // H4AT_HEAP_THROTTLE_LO + _block_alloc. Print both margins so it is obvious
    // from the log whether the gate is the problem.
    // MUST track _heap_alloc in H4P_WiFi::_startWebserver(). This has now been
    // wrong twice - printed 2100 when the gate was 4600, then 3200 when the gate
    // was 2200-or-3800-depending-on-open-connections - and each time I read the
    // log as evidence and drew the wrong conclusion from it. A duplicated
    // constant in a diagnostic is worse than no diagnostic.
    const uint32_t need_heap  = H4AT_HEAP_THROTTLE_LO + 1600; // + _heap_alloc
    const uint32_t need_block = H4AT_HEAP_THROTTLE_LO;        // + _block_alloc(0)
    uint32_t heap  = ESP.getFreeHeap();
    uint32_t block = ESP.getMaxFreeBlockSize();
    // q and mbx are the two things that can grow without bound and take the heap
    // with them. q is H4's task queue - every h4.once()/h4.every() is an entry,
    // and the OOM backtraces point straight at H4::add(), so a queue that climbs
    // means timers are being created faster than they retire. mbx is
    // H4AsyncTCP's buffer pool; that one climbing means connections are not
    // giving their buffers back. A crash at 8488 bytes free on a 365-byte
    // request says the collapse happens between two of these lines, so the trend
    // matters more than any single reading.
    //
    // oc/uc (live H4AsyncClient objects) and act/tw (lwIP's own pcb lists) were
    // here too while the memory faults were being tracked down. Both are gone
    // now that they have answered their questions: oc/uc showed connection
    // objects were NOT accumulating, and act/tw showed TIME_WAIT was not eating
    // the heap either. Add them back the same way if the trail ever goes cold
    // again - lwIP's lists need <lwip/priv/tcp_priv.h>, walked with
    // "for(struct tcp_pcb* p=tcp_tw_pcbs; p; p=p->next)".
    Serial.printf("[ap] mode=%s ip=%s stations=%u heap=%u/%u block=%u/%u q=%u mbx=%u %s\n",
                  (m == WIFI_AP) ? "AP" : "AP_STA",
                  WiFi.softAPIP().toString().c_str(),
                  (unsigned)WiFi.softAPgetStationNum(),
                  (unsigned)heap, (unsigned)need_heap,
                  (unsigned)block, (unsigned)need_block,
                  (unsigned)h4.size(), (unsigned)mbx::pool.size(),
                  (heap >= need_heap && block >= need_block) ? "accepting" : "REFUSING");

    // Self-heal a station interface the scan could not put away.
    //
    // WiFi.mode(WIFI_AP) needs heap to tear the station down, so it is most
    // likely to fail exactly when a sweep has just spent the heap - and when it
    // fails, the station it left up is what keeps the heap too low to try
    // again. One capture sat in AP_STA for 43 consecutive heartbeats with free
    // heap pinned at 2272 and every accept refused, because the one restore
    // attempt happened at the worst possible moment and nothing ever retried.
    //
    // This costs one comparison per beat and only acts when the radio is in a
    // state it should not be resting in. h4pWiFiScanRestoreAP() declines while a
    // sweep is actually using the station, so this cannot interrupt a scan.
    if(m == WIFI_AP_STA && !h4pWiFiScanRestoreAP()){
      // The restore failed, which means WiFi.mode(WIFI_AP) could not get the
      // heap it needs to tear the station down - and the station it could not
      // remove is exactly what is holding the heap down. Retrying the same call
      // every beat cannot break that; one capture sat at heap=1768 block=576
      // printing SCAN MODE RESTORE FAILED while refusing 27 accepts in a row.
      //
      // Idle connections are the only memory here we own outright, at roughly
      // 1.5 KB apiece. Give them up: the client's page is already dead in this
      // state (every request is being refused), so nothing is lost that was not
      // lost already, and the next beat gets a real chance to put the radio
      // back. This is recovery from a state that should no longer be reachable
      // now that the scan start floors price in enableSTA - it is the net under
      // that fix, not a substitute for it.
      auto conns = H4AsyncClient::openConnections;   // copy: close() mutates it
      for(auto c : conns) if(c) c->close();
      if(conns.size())
        Serial.printf("[ap] restore stuck in AP_STA - released %u connection(s)\n",
                      (unsigned)conns.size());
    }
  });

  // Set up regular measurements
  h4.every(MEASUREMENT_INTERVAL * 1000, processData);

  // Every minute, check all calibrations and fix if necessary. Skipped while a
  // calibration is in progress: it would otherwise overwrite values in between
  // the measuring and the storing step of a running calibration.
  #if USE_CAL
  cal_fix_timer = h4.every(60 * 1000, [](){
    if(calibration_running || cal.session_open()) return;
    cal.open_session();
    cal.fix_all_calibrations(quantities, numSensors);
    cal.commit_session();
  });

  // Latch the Calibration tab's live reading here rather than inside the HTTP
  // handler that reports it. measure_gas() blocks on I2C, and on a
  // single-threaded device that stalled ACK processing for every open
  // connection each time the page polled. 2 s is well inside the 10 s poll, so
  // the number on screen is no staler than it was, and the bus is now read at a
  // fixed rate instead of once per client per poll. Does nothing at all until a
  // sensor is selected on the Calibration tab.
  h4.every(2000, [](){ dl_cal_live_update(); });
  #endif

  // One scan after boot, so the WiFi tab is never empty on a fresh device.
  //
  // Timing is the whole trick. NOT from here directly: at this point in setup()
  // the access point does not exist yet - the device is still in WIFI_STA with a
  // deliberately-doomed WiFi.begin() in flight, and the AP only comes up later,
  // asynchronously, when that connect fails. Scanning in that window is exactly
  // what the H4HARDEN:ap_no_background_scan note in H4P_WiFiAP.cpp records as
  // having caused a crash loop: the scan is aborted by the AP's own mode change,
  // and its disconnect events feed back into the path trying to start the AP.
  //
  // So wait for the AP to be genuinely up AND for nobody to have connected. In
  // that window the largest free block measures ~6960 - six times what a scan
  // needs - versus ~1136 once a browser is on the page. Runs once, then stops.
  // REMOVED: the boot scan.
  //
  // It worked exactly as designed - "[scan] boot scan, heap=7112 block=6544"
  // followed by "SCAN COMPLETE: 4 network(s)", on every boot - and it still had
  // to go, because a scan permanently costs ~640 bytes of heap that never comes
  // back (recorded at H4P_WiFi.cpp:645). Spending that before the user has even
  // connected meant every page load afterwards started poorer.
  //
  // Measured consequence: a five-reboot crash loop in four minutes. Boot, scan,
  // phone connects, page transfer starts at heap 920 / block 488, "[tx] out of
  // memory queueing 485 bytes - closing", OOM, reboot, scan again. The device
  // paid for a network list on every restart and then could not serve the page
  // that would have displayed it.
  //
  // Pressing Search still scans, which is what the user actually asked for; it
  // just no longer happens unasked at the worst possible moment.
}

/*
void publishDevice(const std::string &topic, const std::string &payload){
	Serial.printf("Publishing %s to %s\n", CSTR(payload), CSTR(topic));
#if USE_MQTT
	h4mqtt.publishDevice(topic, payload, 1);
#endif
}

void publishDevice(const std::string &topic, long long payload){
	publishDevice(topic, stringFromInt(payload, "%lu"));
}
*/
#if USE_HTTPREQ
void HTTPClient() {
#if SECURE_HTTPREQ
	h4ah.GET("https://www.howsmyssl.com/a/check", [](ARMA_HTTP_REPLY reply){
#else
	h4ah.GET("http://jsonplaceholder.typicode.com/todos/1", [](ARMA_HTTP_REPLY reply){
#endif
		auto rCode = reply.httpResponseCode;
		auto response = reply.asJsonstring();
		auto headers = reply.responseHeaders;
		Serial.printf("code %d response %s\n", rCode, response.c_str());

		for (auto &h:headers) {
			Serial.printf("%s : %s\n", h.first.c_str(), h.second.c_str());
		}
		headers.clear();
		publishDevice("response", response);
	});
}
#endif

