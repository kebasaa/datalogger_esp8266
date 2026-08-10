#pragma once
// lwIP internals, for the TIME_WAIT purge in dl_start_scan(). Private header:
// nothing here uses it beyond walking tcp_tw_pcbs and aborting what is on it.
extern "C" {
#include <lwip/priv/tcp_priv.h>
}
// ============================================================================
//  Datalogger framework internals: HTTP/JSON API and static file serving
//  Included ONLY by datalogger.h (one translation unit). Beginners never need
//  to open this.
//
//  This file replaces the old h4UI widget dashboard. Instead of declaring
//  widgets in C++ and letting the stock h4.js draw them, the device now serves
//  a single-page app from LittleFS and exposes plain JSON:
//
//    GET  /               the app shell           (gzipped, from /h4/app.htm.gz)
//    GET  /app.css        stylesheet              (gzipped)
//    GET  /app.js         application             (gzipped)
//    GET  /api/status     one-shot health snapshot
//    GET  /api/events     the same payload once a second, as SSE
//    GET  /api/cal        stored calibration coefficients
//    GET  /api/wifi/scan  visible networks
//    POST /api/wifi/connect   provisioning
//    GET  /rest/<cmd>     every serial command, unchanged (H4Plugins)
//
//  Calibration deliberately has no bespoke routes: it goes through /rest and
//  the same commands the serial port uses. See dl_webui.h.
// ============================================================================

// ---------------------------------------------------------------------------
//  Buffers and caches
// ---------------------------------------------------------------------------

// The status payload is built here rather than on the heap. It is emitted up
// to once a second and the ESP8266 refuses new TCP connections below ~13 KB
// free, so repeatedly allocating and freeing a ~1 KB string is exactly the
// fragmentation this device cannot afford.
// Shared by the status and calibration payloads - only one request is served at
// a time, so they cannot collide.
//
// Do NOT grow this to make a payload fit. Every byte here is .bss, and .bss comes
// straight off the heap ceiling: measured free heap while a request is actually
// being served on this board is around 850 bytes, so an extra 512 bytes of buffer
// costs nearly two thirds of the working memory a request has. Make the payload
// smaller instead - dl_send_cal_json() omits values it has nothing to report.
static char dl_api_buf[1792];

// Server-initiated message ("Calibration saved", a late error from a /rest
// command) and a counter so the client can tell a new one from a repeat. These
// replace the SSE stream - see dl_toast() below. Declared here because
// dl_status_json() carries them and is defined before it.
static char     dl_msg_buf[96] = {0};
static uint32_t dl_msg_seq = 0;

// sd.getFreeMB() walks every file in the root directory over SPI, so it must
// never run from a request handler. dl_sampling.h refreshes this once per
// sample instead; -1 means "not measured yet".
float dl_sd_free_mb   = -1.0f;
float dl_sd_cap_mb    = -1.0f;

// Live raw/calibrated readout for the Calibration tab. Reading a sensor means
// a blocking I2C transaction, so it only happens once the user has actually
// selected a sensor (cal_select); currentSensor keeps its -9999 "nothing
// selected" sentinel until then.
static bool dl_cal_live(){ return currentSensor != -9999; }

// The live readout is LATCHED by a timer, not measured inside the request.
//
// dl_status_json() used to call measure_gas() directly, which is a blocking I2C
// transaction - inside an HTTP handler, and in direct contradiction of the rule
// stated at the top of that function. H4 is single-threaded: while that blocks,
// nothing services TCP, so ACK processing stalls for EVERY open connection, not
// just the one being answered. It also scaled the wrong way - once per status
// poll, so more clients or more visibility changes meant more bus traffic.
//
// A timer decouples the two: the bus is read at a fixed, known rate, and the
// handler only formats a number that is already sitting in RAM.
static float dl_cal_raw  = NAN;
static float dl_cal_val  = NAN;
static int   dl_cal_flag = -1;

void dl_cal_live_update(){
  if(!dl_cal_live() || calibration_running){
    dl_cal_raw = dl_cal_val = NAN;
    dl_cal_flag = -1;
    return;
  }
  dl_cal_raw = measure_gas(currentGas, currentSensor);
  Cal::CalibrationResult cr = cal.calibrate_linear(currentGas, currentSensor, dl_cal_raw);
  dl_cal_val  = cr.calibratedValue;
  dl_cal_flag = cr.flag;
}

// ---------------------------------------------------------------------------
//  JSON helpers
// ---------------------------------------------------------------------------

// Escape the few characters that would otherwise produce a payload the browser
// cannot parse. Only needed for values we do not control: SSIDs, device names.
static std::string dl_json_escape(const std::string& s){
  std::string out;
  out.reserve(s.size() + 8);
  for(char c : s){
    switch(c){
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': break;
      case '\t': out += "\\t";  break;
      default:
        // Anything below 0x20 is illegal unescaped in JSON; drop it rather
        // than emit a \u escape nothing here will ever need.
        if((unsigned char)c >= 0x20) out += c;
    }
  }
  return out;
}

// A float that is NaN or infinite has no JSON representation - "null" is the
// standard stand-in, and matches how the CSV writer leaves such fields empty.
static const char* dl_json_num(float v, char* tmp, size_t n, int dp = 2){
  if(isnan(v) || isinf(v)){ return "null"; }
  snprintf(tmp, n, "%.*f", dp, v);
  return tmp;
}

// ---------------------------------------------------------------------------
//  Static files
//
//  Mirrors H4AW_HTTPHandler::_serveFile (chunked transfer, so the whole file is
//  never resident), with two things the library version cannot do:
//
//   - it sets Content-Encoding, because the library derives Content-Type from
//     the filename and would label app.js.gz as text/plain;
//   - it checks the file exists FIRST. h4t::readFileChunks writes nothing at
//     all for a missing file while _execute() still reports success, so the
//     route loop stops, the 404 handler never runs, and the browser hangs
//     until the 20 s scavenger closes the socket. With the SPA owning "/",
//     that failure mode would look like a dead device.
// ---------------------------------------------------------------------------
// Bytes of file data per chunk. Small enough that chunk + framing fits one TCP
// segment, and small enough that the read buffer readFileChunks allocates stays
// well under the largest free block even on a fragmented heap. Kept modest
// because the framing buffer below is static: every byte here is a byte of .bss
// that can never be heap, and heap is what this board runs out of.
// 256, halved from 512.
//
// The peak CONTIGUOUS memory a page load needs is one chunk copy plus the
// response headers - and that peak, not the size of the page, is what decides
// whether a load succeeds. Free heap with a client is 2100-5700 but the largest
// contiguous block is typically only 1100-1500, so a 512-byte chunk (648 with
// framing) plus headers was routinely more than the device could find.
//
// Halving it halves the chunk half of that demand. The cost is twice as many
// pump ticks for the same file, which is time rather than memory and the pump is
// paced on the transmit backlog anyway - it sends the next piece as soon as the
// wire has taken the last, so a smaller piece simply means more, shorter waits.
//
// Note this is NOT a page-size problem: shrinking the page reduces the NUMBER of
// chunks, never the peak. That is why trimming UI features bought far less than
// this one line.
#define DL_CHUNK 256

// ---------------------------------------------------------------------------
//  Returning the socket
//
//  lwIP is built with MEMP_NUM_TCP_PCB 5 - five TCP connections for the whole
//  device - and nothing in H4AsyncWebServer closes a server-side connection. The
//  only reclamation is H4AsyncTCP's 20 second scavenger, so without this a couple
//  of page loads exhaust the pool. lwIP then stops answering SYNs altogether
//  (silently: no RST), and the browser sits there until it times out.
//
//  Every response sets "Connection: close", so a well-behaved client normally
//  closes first. This is the safety net for when it does not.
//
//  The delay matters in BOTH directions, which is why it is a parameter.
//
//  Too short and a large reply is truncated: TX() only hands lwIP what
//  altcp_sndbuf() will take right now (TCP_SND_BUF = 2 * TCP_MSS = 2920) and
//  queues the rest to drain as ACKs return, so closing immediately after an
//  8.6 KB page threw most of it away -> ERR_EMPTY_RESPONSE.
//
//  Too long and everything else starves. Measured with a phone joining the AP:
//  idle heap 9448, but only 1920 left by the time the page handler ran, with a
//  1496-byte largest block. Android fires several captive-portal probes at once
//  on joining, each accepted connection costs roughly 1.5 KB, and holding them
//  all for five seconds spanned exactly the window the page needed memory in.
//  The chunk buffers then could not be allocated and were silently dropped - a
//  response that looks complete on the device and arrives empty at the browser.
//
//  So: small fixed-length replies (302s, JSON) fit entirely in the send buffer
//  and are gone the moment they are written - release those almost at once.
//  Only the chunked page needs to linger.
//
//  Guarded on BOTH identity checks: openConnections proves the client is still
//  live, and _creatTime proves it is the same one - over seconds the allocator
//  could otherwise hand the same address to a new connection and we would close
//  a stranger's socket.
// ---------------------------------------------------------------------------
#define DL_CLOSE_SMALL_MS  150     // reply already fits in the send buffer
#define DL_CLOSE_BULK_MS  3000     // chunked page still draining

// ---------------------------------------------------------------------------
//  No single-tenant gate.
//
//  There was one here: first by client IP, then by session cookie, to show a
//  second browser a "someone else is using it" page instead of a silent
//  failure. Both versions are removed.
//
//  The IP version could not work at all - two browsers on one phone share an
//  address, and it fired zero times. The cookie version then worked too well: a
//  page transfer that stalled left the browser with an incomplete response and
//  no stored cookie, so every retry arrived anonymous, was told another session
//  held the device, and got the busy page. That other session was its own
//  abandoned first request - it locked the only user out of the logger.
//
//  The feature cost more than it gave. Serve whoever asks, and spend the effort
//  on why a reload fails instead.
// ---------------------------------------------------------------------------

static void dl_close_soon(H4AW_HTTPHandler* h, uint32_t after_ms = DL_CLOSE_SMALL_MS){
  auto* r = h->client();
  if(!r) return;
  uint32_t born = r->_creatTime;
  h4.once(after_ms, [r, born]{
    if(H4AsyncClient::openConnections.count(r) && r->_creatTime == born) r->close();
  });
}

// The bulk version, for the chunked page.
//
// DL_CLOSE_BULK_MS was a fixed 3 second wait chosen before there was any way to
// see the transmit queue - long enough to be safe, and therefore far longer than
// needed. Three seconds is a whole connection and ~1.5KB held across exactly the
// window where the browser starts polling /api/status and Android is still
// firing captive-portal probes, against an lwIP with five PCBs in total.
//
// Now that txBacklog() exists we can ask instead of guess: poll until everything
// queued has actually reached lwIP, then close. Typically milliseconds. The
// original timeout stays as a ceiling for a client that stops ACKing.
// State is static, not new/delete'd per transfer, for two reasons.
//
// The first version allocated an H4_TIMER on the heap for every page load. On a
// device whose whole problem is a largest-free-block that shrinks as the session
// runs (measured: 4688 -> 2504 -> 2400 -> 952 over five requests), adding a
// small short-lived allocation to each one is the wrong direction.
//
// It was also unsound. The timer captured a pointer to its own handle and did
// "if(*slot) h4.cancel(*slot); delete slot;" from inside its own callback - but
// *slot is only assigned after h4.every() returns. A fire before that assignment
// sees nullptr, skips the cancel, deletes the block, and every later fire is a
// use-after-free on a timer that now never stops.
//
// Only one transfer runs at a time (dl_xfer.busy), so one static slot is enough.
static H4_TIMER      dl_drain_timer = nullptr;
static H4AsyncClient* dl_drain_cli  = nullptr;
static uint32_t      dl_drain_born  = 0;
static uint32_t      dl_drain_end   = 0;

static void dl_drain_tick(){
  auto* r = dl_drain_cli;
  bool gone = !r || !H4AsyncClient::openConnections.count(r) ||
              r->_creatTime != dl_drain_born;
  if(gone || !r->txBacklog() || (int32_t)(millis() - dl_drain_end) >= 0){
    if(!gone) r->close();
    dl_drain_cli = nullptr;
    if(dl_drain_timer){ h4.cancel(dl_drain_timer); dl_drain_timer = nullptr; }
  }
}

static void dl_close_when_drained(H4AW_HTTPHandler* h, uint32_t ceiling_ms = DL_CLOSE_BULK_MS){
  auto* r = h->client();
  if(!r) return;
  if(dl_drain_timer){ h4.cancel(dl_drain_timer); dl_drain_timer = nullptr; }
  dl_drain_cli  = r;
  dl_drain_born = r->_creatTime;
  dl_drain_end  = millis() + ceiling_ms;
  dl_drain_timer = h4.every(20, dl_drain_tick);
}

// ---------------------------------------------------------------------------
//  JSON replies, written in pieces
//
//  H4AW_HTTPHandler::send() mallocs headers+body as ONE contiguous block. For
//  the status payload that is ~879 bytes of JSON plus ~200 of headers, so every
//  API reply demanded a ~1100-byte contiguous block - and that, not free heap,
//  is what the accept gate's block half had to cover.
//
//  Measured with a phone attached, the largest free block wanders between about
//  1400 and 2700 depending on how the SDK happens to have fragmented the heap.
//  A gate of 1500 therefore sat INSIDE the band the hardware fluctuates in: one
//  session settled at 1768 and worked, the next settled at 1408 and refused
//  every request by 92 bytes. That is a coin toss, not a threshold.
//
//  Writing the headers and then the body in DL_CHUNK pieces drops the largest
//  single allocation to ~512, which lets the gate move safely below the band
//  instead of living inside it. Content-Length is known up front, so no chunked
//  framing is needed - these are just successive writes to the same stream.
// ---------------------------------------------------------------------------
static void dl_send_json(H4AW_HTTPHandler* h, const char* body, size_t len){
  auto* cli = h->client();
  if(!cli) return;
  char hdr[160];
  int n = snprintf(hdr, sizeof(hdr),
                   "HTTP/1.1 200 OK\r\n"
                   "Content-Type: application/json\r\n"
                   "Content-Length: %u\r\n"
                   "Connection: close\r\n"
                   "Cache-Control: no-store\r\n\r\n",
                   (unsigned)len);
  if(n <= 0) return;
  cli->TX((const uint8_t*)hdr, (size_t)n);
  for(size_t off = 0; off < len; off += DL_CHUNK){
    size_t take = len - off;
    if(take > DL_CHUNK) take = DL_CHUNK;
    cli->TX((const uint8_t*)body + off, take);
  }
}

// ---------------------------------------------------------------------------
//  Paced file transfer
//
//  The obvious implementation - hand the file to h4t::readFileChunks and TX every
//  chunk in the callback - does not work on this board, and the reason is worth
//  writing down.
//
//  TX() only passes lwIP what altcp_sndbuf() accepts at that instant
//  (TCP_SND_BUF = 2 * TCP_MSS = 2920 bytes). Everything beyond that is copied
//  onto the heap in H4AsyncClient's transmit queue, to be drained as ACKs come
//  back. Streaming an 8.6 KB page in one uninterrupted loop therefore asks for
//  roughly 5.7 KB of heap at once - and measured free heap when the page handler
//  actually ran, with a phone attached, was 1920 bytes with a 1496-byte largest
//  block. The allocations failed, the chunks were dropped without complaint, and
//  the browser was handed a response that looked complete on the device and
//  arrived empty at the other end: ERR_EMPTY_RESPONSE.
//
//  So the file goes out one chunk at a time - but the gate is the transmit
//  BACKLOG, not the clock. An earlier version of this paced on a timer alone and
//  assumed "between ticks the ACKs arrive and the queue drains". That assumption
//  is false on a slow link: a timer bounds the rate we offer data, not the amount
//  outstanding, so the queue still grew to hold the whole file. Measured, it did
//  exactly that - heap 7048 -> 768 while serving one page, followed by an OOM
//  panic on a 241-byte allocation.
//
//  Waiting for txBacklog() to clear makes the wire itself the pacer, so peak heap
//  is one chunk regardless of how slow the client is. The timer is now only a
//  polling interval, so it can be short.
//
//  One transfer at a time, which is not worth engineering around: the UI is a
//  single self-contained page and lwIP here has five TCP connections in total.
// ---------------------------------------------------------------------------
#define DL_PACE_MS 10

static struct DlXfer {
  H4AW_HTTPHandler* h    = nullptr;
  H4AsyncClient*    cli  = nullptr;
  uint32_t          born = 0;
  fs::File          f;
  H4_TIMER          timer = nullptr;
  bool              busy  = false;
  uint32_t          progress = 0;           // millis() of the last chunk sent
} dl_xfer;

// A transfer must never be able to pin state indefinitely, and the first version
// of the backlog gate could. Measured: "[http] start ... heap=1024 block=312"
// with no matching "done" - the transfer stalled and held the open file, the
// socket and ~3KB for the rest of the session, so every later connection was
// refused and the logger needed a reboot.
//
// Two ways it can stall, and neither announces itself:
//   - the phone stops ACKing without sending FIN, so connected() stays true and
//     the backlog never drains
//   - the pump's own heap guard cannot be satisfied, and since only this
//     transfer completing would free memory, it starves itself. A livelock, not
//     a deadlock: the timer keeps firing and nothing ever moves.
//
// So: if no chunk has gone out for this long, give up and hand everything back.
//
// 1500, down from 4000. The common case is not a slow link, it is a peer that
// has gone away - a reload abandons the previous page's transfer without always
// sending FIN, so connected() stays true and the pump waits out the whole
// timeout. Measured: four consecutive stalls with byte-identical figures
// ("start heap=1232 block=840" -> "STALLED heap=2120 block=1200 q=1"), which is
// the same dead-peer path each time, not varying memory pressure.
//
// At 4 s each, and a browser retrying immediately, the device spent most of its
// time holding memory for connections that were never coming back. 1500 ms is
// still far longer than a live client needs to ACK one 512-byte chunk on the
// same room's WiFi, and it frees the socket nearly three times sooner.
#define DL_XFER_STALL_MS 1500

static void dl_xfer_stop(bool send_terminator){
  if(dl_xfer.timer){ h4.cancel(dl_xfer.timer); dl_xfer.timer = nullptr; }
  if(dl_xfer.f) dl_xfer.f.close();
  bool live = dl_xfer.cli &&
              H4AsyncClient::openConnections.count(dl_xfer.cli) &&
              dl_xfer.cli->_creatTime == dl_xfer.born;
  if(send_terminator && live){
    static const uint8_t terminator[5] = {'0','\r','\n','\r','\n'};
    dl_xfer.cli->TX(terminator, 5);
    if(dl_xfer.h) dl_close_when_drained(dl_xfer.h);
  }
  else if(live){
    // An abandoned transfer used to just drop the state and leave the socket to
    // the 20 second scavenger. That is the worst possible time to hold one: the
    // browser is already retrying, and each retry arrives to find even less
    // memory than the attempt before. Give it straight back.
    dl_xfer.cli->close();
  }
  dl_xfer.busy = false;
  dl_xfer.h    = nullptr;
  dl_xfer.cli  = nullptr;
}

static void dl_xfer_pump(){
  if(!dl_xfer.busy) return;
  // The client can vanish mid-transfer; stop rather than write into nothing.
  if(!dl_xfer.cli || !H4AsyncClient::openConnections.count(dl_xfer.cli) ||
     dl_xfer.cli->_creatTime != dl_xfer.born || !dl_xfer.cli->connected()){
    dl_xfer_stop(false);
    return;
  }
  if(!dl_xfer.f || !dl_xfer.f.available()){ dl_xfer_stop(true); return; }

  if(millis() - dl_xfer.progress > DL_XFER_STALL_MS){
    Serial.printf("[http] STALLED, abandoning: heap=%u block=%u q=%u\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxFreeBlockSize(),
                  (unsigned)dl_xfer.cli->txBacklog());
    dl_xfer_stop(false);          // closes the file and the socket
    return;
  }

  // Flow control, and the reason this function was rewritten.
  //
  // Pacing on a timer limits the RATE we offer data; it does not limit how much
  // is OUTSTANDING. H4AsyncClient::TX() hands what it can to lwIP and mallocs a
  // copy of the remainder into an unbounded internal queue (H4AsyncTCP.cpp,
  // "_queue.push(new TCPData{...})"). So on a link slower than the timer - which
  // over WiFi to a phone is most of them - the queue is the file.
  //
  // Measured with the old burst-of-4 version: "[http] start ... heap=768
  // block=248" - the whole 8.6KB page had been copied into a 7KB heap, and the
  // OOM panics were 241-byte allocations failing in that hole.
  //
  // The rule now is simply: nothing new goes out until the last chunk is
  // actually on the wire. Peak cost is one chunk, whatever the link speed does.
  // Low-water marks (lowheap/lowblock/maxq) were tracked here and reported by
  // "[http] done". They proved the gate works - peak cost really is one chunk -
  // and are gone with the report, rather than left costing two ESP calls per
  // tick to feed nothing.
  if(dl_xfer.cli->txBacklog()) return;

  static uint8_t chunk_buf[DL_CHUNK + 8];
  // Read past the space the length prefix needs so the chunk is framed without a
  // second copy. The prefix is always 5 bytes: "%03x\r\n" is fixed width for
  // anything up to 4095, and DL_CHUNK is well under that.
  const size_t hdr = 5;

  // Headroom for ONE chunk copy plus a little slack - not two. Requiring 2x was
  // both wrong and self-defeating: TX() copies a single chunk, and demanding
  // 1040 bytes when the largest block was hovering at ~1072 meant the pump could
  // sit there refusing to make the very progress that would free the memory.
  size_t need = DL_CHUNK + 8 + 128;
  if(ESP.getMaxFreeBlockSize() < need) return;   // wait a tick; the stall timer
                                                 // bounds how long that can last
  int n = dl_xfer.f.read(chunk_buf + hdr, DL_CHUNK);
  if(n <= 0){ dl_xfer_stop(true); return; }
  char hex[8];
  snprintf(hex, sizeof(hex), "%03x\r\n", (unsigned)n);
  memcpy(chunk_buf, hex, hdr);
  memcpy(chunk_buf + hdr + n, "\r\n", 2);
  dl_xfer.cli->TX(chunk_buf, hdr + n + 2);
  dl_xfer.progress = millis();
}

void dl_send_static(H4AW_HTTPHandler* h, const char* fn, const char* mime, bool gz){
  size_t fsize = 0;
  {
    auto probe = HAL_FS.open(fn, "r");
    if(probe){ fsize = probe.size(); probe.close(); }
  }
  if(!fsize){
    Serial.printf("HTTP 404 %s (missing or empty - did you run 'pio run -t uploadfs'?)\n", fn);
    h->send(404, "text/plain", 9, "not found");
    dl_close_soon(h);
    return;
  }
  // A second request while one is in flight would scribble over the transfer
  // state. Refuse it rather than corrupt both.
  if(dl_xfer.busy){
    Serial.println(F("HTTP 503 (a transfer is already in progress)"));
    h->send(503, "text/plain", 4, "busy");
    dl_close_soon(h);
    return;
  }

  // Answer a reload with 304 instead of resending nine kilobytes.
  //
  // This is the cheapest fix available for the symptom that has dominated
  // testing: reloading the page gives ERR_CONNECTION_RESET. The measured cause
  // is that free heap with a client attached routinely sits at 2100-3100, the
  // accept gate wants more than that, and the reload's connection is refused -
  // 98 times in one four-minute session, at heap 2520-3064.
  //
  // Tuning the gate cannot fix it: serving a request needs ~2000 bytes, and
  // refusing stops only below ~2100, so there is no value that both accepts
  // reliably and can deliver. The way out is to stop the reload being expensive.
  //
  // The file is static between builds, so its size is a perfectly good ETag. A
  // normal reload then sends If-None-Match and gets ~90 bytes back instead of
  // 9118, with no transfer, no chunk pump and no 1.5 s of held socket. Only a
  // hard reload (Ctrl-Shift-R, which sends no If-None-Match) pays full price.
  {
    char etag[24];
    snprintf(etag, sizeof(etag), "\"%x\"", (unsigned)fsize);
    auto& rh = h->req_headers();
    auto it = rh.find("IF-NONE-MATCH");
    if(it != rh.end() && it->second.find(etag+1) != std::string::npos){
      h->addHeader("ETag", etag);
      h->addHeader("Cache-Control", "max-age=600");
      h->addHeader("Connection", "close");
      h->send(304, "text/html", 0, nullptr);
      dl_close_soon(h);
      return;
    }
  }

  // Refuse fast rather than start a transfer that cannot move.
  //
  // dl_xfer_pump needs DL_CHUNK+8+128 contiguous to copy one chunk. If the block
  // is already below that when the handler runs, the pump returns on every tick
  // without sending anything and the stall timer abandons it 1.5 s later - by
  // which point the browser has an incomplete response and reports
  // ERR_EMPTY_RESPONSE. Measured three times in one session:
  //
  //   [http] start /h4/app.htm.gz 9130 bytes heap=1040 block=488
  //   [http] STALLED, abandoning: heap=2944 block=1288 q=1
  //
  // A 503 costs one small reply and the browser retries by itself; a stall costs
  // 1.5 s of held socket and gives the user a broken page. Note the block had
  // recovered to 1288 by the time it gave up - so retrying is likely to work,
  // which makes failing fast the right answer rather than a resignation.
  // Test for what the pump will need AFTER everything else has been spent, not
  // for what is free right now.
  //
  // The first version compared the block against the pump's own 648-byte chunk
  // requirement and passed - and then the transfer failed anyway, because
  // between this check and the first chunk the handler still opens the file (a
  // LittleFS cache) and H4AW_HTTPHandler::send() mallocs the response headers as
  // one block. Measured: the guard saw 824, never fired, and the pump then found
  // 488:
  //
  //   [http] start /h4/app.htm.gz 8721 bytes heap=1160 block=488
  //   [tx] out of memory queueing 485 bytes - closing
  //
  // "503 too tight to serve" appeared zero times in that capture while three
  // page loads failed, which is a guard that measures the wrong moment. Add the
  // headroom those later allocations take, so the decision is made against the
  // state the pump will actually see.
  // 1600, because 1048 was still measured too early. The guard passed at 1048
  // and the pump then found 488 - a 560-byte drop spent between the two on the
  // LittleFS file cache and, mostly, H4AW_HTTPHandler::send() building the
  // response headers as one malloc'd block. Three page loads died that way in a
  // capture where "503 too tight to serve" fired exactly once.
  // Down from 1600, because the two things it was reserving for are now much
  // smaller: the chunk is 256 rather than 512, and the response headers no
  // longer go through H4AW_HTTPHandler::send()'s single malloc - they are
  // written straight from a stack buffer. What remains to reserve is one framed
  // chunk plus the LittleFS file cache.
  static const size_t DL_SERVE_NEED = DL_CHUNK + 8 + 128   // one framed chunk
                                    + 200;                 // file cache + slack
  if(ESP.getMaxFreeBlockSize() < DL_SERVE_NEED){
    Serial.printf("[http] 503 too tight to serve: heap=%u block=%u (need %u)\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxFreeBlockSize(),
                  (unsigned)DL_SERVE_NEED);
    // A navigation shows whatever comes back, so send something that recovers by
    // itself rather than the word "busy". Two seconds is longer than the block
    // typically takes to come back (measured recovering to 1288 within 1.5 s of
    // a failed attempt), and it means a momentarily tight device looks like a
    // slow load instead of a dead one.
    static const char wait[] =
      "<!doctype html><meta name=viewport content=\"width=device-width\">"
      "<meta http-equiv=refresh content=2><title>Datalogger</title>"
      "<body style=\"font:16px sans-serif;padding:2em;text-align:center\">"
      "<p>Logger busy &mdash; retrying&hellip;</p>";
    h->addHeader("Retry-After", "1");
    h->addHeader("Connection", "close");
    h->send(503, "text/html", sizeof(wait)-1, wait);
    dl_close_soon(h);
    return;
  }

  dl_xfer.f = HAL_FS.open(fn, "r");
  if(!dl_xfer.f){
    h->send(404, "text/plain", 9, "not found");
    dl_close_soon(h);
    return;
  }

  // Content negotiation, and the reason the captive portal offered a download.
  //
  // We only hold the page gzipped, and we were sending Content-Encoding: gzip to
  // everyone without ever looking at Accept-Encoding. A client that did not ask
  // for gzip gets a stream of binary labelled text/html: browsers do not render
  // that, they save it - which is the ".bin file" the captive portal offered.
  // Android's CaptivePortalLogin webview is exactly such a client.
  //
  // A restricted portal webview is a poor host for the dashboard anyway (no
  // persistent storage, often no JS), so rather than ship an uncompressed copy of
  // the whole page just for it, hand it a small plain page with a link. Tapping
  // that opens the real browser, which does send Accept-Encoding: gzip.
  if(gz){
    auto& rh = h->req_headers();                 // stored uppercased, single copy
    auto it  = rh.find("ACCEPT-ENCODING");
    if(it == rh.end() || it->second.find("gzip") == std::string::npos){
      dl_xfer.f.close();
      static const char plain[] =
        "<!doctype html><meta name=viewport content=\"width=device-width\">"
        "<title>Datalogger</title>"
        "<body style=\"font:16px sans-serif;padding:2em;text-align:center\">"
        "<h2>Datalogger</h2><p>Open the dashboard in your browser:</p>"
        "<p><a href=\"http://192.168.4.1/\">http://192.168.4.1/</a></p>";
      Serial.println(F("[http] client does not accept gzip - sent plain stub"));
      h->addHeader("Connection", "close");
      h->send(200, "text/html", sizeof(plain)-1, plain);
      dl_close_soon(h);
      return;
    }
  }

  // Write the headers from the stack, not through h->send().
  //
  // This is the single biggest lever on the peak CONTIGUOUS memory a page load
  // needs, which is what actually decides whether it succeeds - not the size of
  // the page. H4AW_HTTPHandler::send() builds the whole header set by repeatedly
  // appending to a std::string, then mallocs one block for it. Measured, that
  // path took the largest free block from 1048 down to 488 between the serve
  // guard and the first chunk, and the transfer then died with "[tx] out of
  // memory queueing 485 bytes".
  //
  // dl_send_json already writes its headers this way. Doing the same here means
  // a page load's peak contiguous demand is one chunk copy and nothing else.
  // "Connection: close" because we close after each response rather than let a
  // client hold a keep-alive socket against a five-connection pool; the ETag is
  // the file size, so the next reload can be answered with 304.
  {
    auto* c = h->client();
    if(!c){ dl_xfer.f.close(); return; }
    char hdr[260];
    int n = snprintf(hdr, sizeof(hdr),
                     "HTTP/1.1 200 OK\r\n"
                     "Content-Type: %s\r\n"
                     "Transfer-Encoding: chunked\r\n"
                     "Cache-Control: max-age=600\r\n"
                     "ETag: \"%x\"\r\n"
                     "Connection: close\r\n"
                     "%s\r\n",
                     mime, (unsigned)fsize,
                     gz ? "Content-Encoding: gzip\r\nVary: Accept-Encoding\r\n" : "");
    if(n <= 0 || n >= (int)sizeof(hdr)){ dl_xfer.f.close(); return; }
    c->TX((const uint8_t*)hdr, (size_t)n);
  }

  dl_xfer.h    = h;
  dl_xfer.cli  = h->client();
  dl_xfer.born = dl_xfer.cli ? dl_xfer.cli->_creatTime : 0;
  dl_xfer.busy = true;
  dl_xfer.progress = millis();
  dl_xfer.timer = h4.every(DL_PACE_MS, dl_xfer_pump);
}

// ---------------------------------------------------------------------------
//  Status payload
//
//  Formats only values the sampler has already latched. Nothing here may touch
//  the I2C bus: a stuck device blocks the H4 loop for up to the clock-stretch
//  limit (I2C_CLOCK_STRETCH_US, which the SCD30 forces to 200 ms) on every
//  bit-wait, which starves the TCP ACK processing the connection depends on. The
//  one exception is the calibration readout, and it is off until the user selects
//  a sensor.
// ---------------------------------------------------------------------------
// Builds the payload into dl_api_buf and returns its length. Deliberately NOT a
// std::string: returning one copied the whole ~1.5 KB payload onto the heap, and
// H4AW_HTTPHandler::send() then mallocs headers+body and copies it a second
// time - two transient allocations of a kilobyte and a half each, for a poll the
// dashboard makes every three seconds. Measured free heap while a request is
// actually being handled on this board is around 800 bytes, so that mattered.
//
// It used to be by value because H4AW_HTTPHandlerSSE::send() ran strtok over the
// string it was handed, writing NULs into it, so a shared buffer would have been
// shredded after the first push. The SSE stream is gone, and with it that reason.
size_t dl_status_json(){
  char n1[16], n2[16], n3[16], n4[16];
  int  p = 0;
  const int  N = sizeof(dl_api_buf);
  char* b = dl_api_buf;

  // "bootid", not "boot": the object below is already called boot, and emitting
  // the name twice made this invalid-ish JSON that only worked because browsers
  // keep the last occurrence - silently discarding boot_id in the process.
  p += snprintf(b + p, N - p,
    "{\"dev\":\"%s\",\"ver\":\"%s\",\"up\":%lu,\"bootid\":%lu,\"n\":%lu",
    dl_json_escape(h4p[deviceTag()]).c_str(), WS_VERSION,
    (unsigned long)(millis() - boot_ms), (unsigned long)boot_id,
    (unsigned long)sample_counter);

  // The clock as of RIGHT NOW, not as of the last sample. The GPS card reads its
  // time from timestamp_calc_buf, which is only written inside the sampler every
  // MEASUREMENT_INTERVAL seconds - so after setting the clock by hand the page
  // showed nothing at all for up to twenty seconds, which looks exactly like the
  // command having been ignored. Computed into a local buffer; the CSV writer's
  // buffers must not be touched from a request handler.
  {
    const char* src = "";
    uint32_t now_epoch = calculated_timestamp_epoch(false, false, millis(), &src);
    char nowbuf[20];
    if(now_epoch){
      format_utc_timestamp(now_epoch, nowbuf, sizeof(nowbuf));
      p += snprintf(b + p, N - p, ",\"now\":\"%s\"", nowbuf);
    } else {
      p += snprintf(b + p, N - p, ",\"now\":null");
    }
  }

  // Server-initiated message, carried here now that the SSE stream is gone. The
  // counter is what lets the client tell a new message from the same one being
  // polled again.
  p += snprintf(b + p, N - p,
    ",\"msg\":\"%s\",\"msgseq\":%lu",
    dl_json_escape(dl_msg_buf).c_str(), (unsigned long)dl_msg_seq);

  p += snprintf(b + p, N - p,
    ",\"heap\":{\"free\":%u,\"max\":%u,\"min\":%u}",
    heap_free_now(), heap_max_block_now(), heap_min_block_now());

  // "locked" gates everything else: until the first sane GPS or RTC time, the
  // sampler returns early and every counter below is still zero. Without this
  // the page would look perfectly healthy while nothing is being recorded.
  p += snprintf(b + p, N - p,
    ",\"boot\":{\"locked\":%s,\"src\":\"%s\",\"reset\":\"%s\",\"rtc\":%s}",
    gps_boot_locked ? "true" : "false", boot_time_source_buf,
    reset_reason_buf, rtc_time_valid ? "true" : "false");

#if USE_GPS
  p += snprintf(b + p, N - p,
    ",\"gps\":{\"date\":%s,\"fresh\":%s,\"sane\":%s,\"loc\":%s,\"sat\":%lu,\"hdop\":%s"
    ",\"age\":%lu,\"stale\":%lu,\"rec\":%lu,\"quar\":%s,\"rej\":%lu,\"chars\":%lu"
    ",\"ts\":\"%s\",\"lat\":%s,\"lon\":%s,\"alt\":%s}",
    isGPSDateValid() ? "true" : "false", isGPSTimeFresh() ? "true" : "false",
    gps_time_sane ? "true" : "false", gps.locationValid() ? "true" : "false",
    (unsigned long)gps.satellites(), dl_json_num(gps.hdop(), n1, sizeof(n1)),
    (unsigned long)gps.gpsAgeMs(), (unsigned long)gps_stale_count,
    (unsigned long)gps_recovery_count, gps_quarantine_active ? "true" : "false",
    (unsigned long)gps_rejected_time_count, (unsigned long)gps_chars_since_sample,
    timestamp_calc_buf,
    dl_json_num(gps.get_lat(), n2, sizeof(n2), 6),
    dl_json_num(gps.get_lon(), n3, sizeof(n3), 6),
    dl_json_num(gps.get_alt(), n4, sizeof(n4), 1));
#endif

#if USE_BATTERY
  p += snprintf(b + p, N - p, ",\"bat\":{\"mv\":%s,\"pc\":%s}",
    dl_json_num(bat.battery_mV(), n1, sizeof(n1), 0),
    dl_json_num(bat.battery_pc(), n2, sizeof(n2), 0));
#endif

#if USE_MICROSD
  p += snprintf(b + p, N - p,
    ",\"sd\":{\"cap\":%s,\"free\":%s,\"st\":%u,\"miss\":%lu,\"open\":%lu,\"write\":%lu}",
    dl_json_num(dl_sd_cap_mb, n1, sizeof(n1), 0),
    dl_json_num(dl_sd_free_mb, n2, sizeof(n2), 0),
    last_data_write_status,
    (unsigned long)sd.cardMissingCount(),
    (unsigned long)(sd.headerOpenFailCount() + sd.appendOpenFailCount()),
    (unsigned long)(sd.printFailCount() + sd.flushFailCount() + sd.closeFailCount()));
#endif

  // Totals first, then one entry per bus. "ce" is a live streak that clears
  // the moment a bus reads cleanly, so it must not be labelled as a total.
  p += snprintf(b + p, N - p, ",\"i2c\":{\"slow\":%lu,\"err\":%lu,\"rec\":%lu,\"bus\":[",
    (unsigned long)i2c_slow_total(), (unsigned long)i2c_error_total(),
    (unsigned long)i2c_recovery_total());
  for(size_t i = 0; i < num_i2c_buses; i++){
    p += snprintf(b + p, N - p, "%s{\"ch\":%u,\"e\":%lu,\"ce\":%lu,\"r\":%lu}",
      i ? "," : "", i2c_buses[i],
      (unsigned long)i2c_error_count[i],
      (unsigned long)i2c_consecutive_error_count[i],
      (unsigned long)i2c_recovery_count[i]);
  }
  p += snprintf(b + p, N - p, "]}");

  p += snprintf(b + p, N - p,
    ",\"sup\":{\"crit\":%lu,\"hard\":%lu,\"rst\":%lu,\"pend\":%s,\"why\":\"%s\",\"valid\":%lu,\"missing\":%lu}",
    (unsigned long)critical_sensor_loss_count, (unsigned long)gps_i2c_hard_fault_count,
    (unsigned long)supervisor_reset_count, supervisor_restart_pending ? "true" : "false",
    supervisor_restart_reason_buf,
    (unsigned long)valid_sensor_value_count, (unsigned long)missing_sensor_value_count);

  // Calibration session state, plus the live readout when a sensor is selected
  p += snprintf(b + p, N - p,
    ",\"cal\":{\"open\":%s,\"pending\":%u,\"status\":\"%s\",\"running\":%s"
    ",\"gas\":\"%s\",\"sensor\":%d,\"dgas\":\"%s\"",
    cal.session_open() ? "true" : "false", (unsigned)cal.pending_count(),
    cal_status_text().c_str(), calibration_running ? "true" : "false",
    currentGas.c_str(), currentSensor, currentDiffGas.c_str());
  // Latched by dl_cal_live_update() on a timer - no I2C on this path.
  if(dl_cal_live() && !calibration_running){
    p += snprintf(b + p, N - p, ",\"raw\":%s,\"cal\":%s,\"flag\":%d",
      dl_json_num(dl_cal_raw, n1, sizeof(n1), 3),
      dl_json_num(dl_cal_val, n2, sizeof(n2), 3), dl_cal_flag);
  }
  p += snprintf(b + p, N - p, "}");

  // Which network we are on, so the WiFi tab can show the current state
  WiFiMode_t wm = WiFi.getMode();
  bool ap = (wm == WIFI_AP || wm == WIFI_AP_STA);
  p += snprintf(b + p, N - p, ",\"wifi\":{\"ap\":%s,\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d}}",
    ap ? "true" : "false",
    dl_json_escape(ap ? std::string(h4p[deviceTag()]) : std::string(WiFi.SSID().c_str())).c_str(),
    (ap ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(),
    ap ? 0 : (int)WiFi.RSSI());

  // snprintf reports what it WOULD have written, so a payload that outgrew the
  // buffer is silently truncated into invalid JSON. Say so instead.
  if(p >= N){
    Serial.printf("dl_status_json truncated (%d >= %d)\n", p, N);
    p = snprintf(dl_api_buf, N, "%s", "{\"err\":\"status payload too large\"}");
  }
  return (size_t)p;
}

// ---------------------------------------------------------------------------
//  Server-sent events
// ---------------------------------------------------------------------------

// There is no server-sent-events stream any more. An SSE handler holds a TCP
// connection open for as long as the dashboard is on screen, and lwIP here is
// built with MEMP_NUM_TCP_PCB 5 - the page was occupying a fifth of the device's
// entire TCP capacity to receive something it could just as well ask for.
//
// Server-initiated messages instead sit in this fixed buffer and travel out with
// the next /api/status poll, tagged with a counter so the client can tell a new
// message from a repeat. Static, because the whole point is to stop spending
// heap on the dashboard. (The buffer itself is declared at the top of this file,
// since dl_status_json() needs it and comes first.)
//
// Declared in dl_webui.h so dl_notify() can reach it from calibration code
// that is compiled before this file.
void dl_toast(const std::string& msg){
  strncpy(dl_msg_buf, msg.c_str(), sizeof(dl_msg_buf) - 1);
  dl_msg_buf[sizeof(dl_msg_buf) - 1] = '\0';
  dl_msg_seq++;
}

// ---------------------------------------------------------------------------
//  Calibration coefficients
// ---------------------------------------------------------------------------
// Built into the shared static buffer, not grown on the heap.
//
// This used to assemble a ~1.8 KB payload by repeatedly += -ing a std::string,
// which reallocates as it doubles and needs a contiguous block the size of the
// whole result. Measured free heap while a request is being served on this board
// is around 800 bytes, so it simply failed - which is what "calibration could not
// be read" was in the browser. Same treatment as dl_status_json().
static void dl_send_cal_json(H4AW_HTTPHandler* h){
  // This endpoint was once suspected of being the most expensive on the device
  // and given a low-memory bail-out. Both halves of that were wrong, and the
  // instrumentation added to prove it disproved it instead:
  //
  //     [http] cal 596 bytes, heap 3040->3040 block 2112->2112
  //
  // Zero heap delta, 596-byte payload. It costs nothing. The bail-out was also
  // actively harmful: it returned an error object with no "q" array, the client
  // threw on j.q.map, state.quantities stayed empty, and its "load once" guard
  // then re-fired /api/cal from every single status poll - a request storm
  // triggered by the device reporting that it was short of memory.
  //
  // The real cause of those crashes was several connections being accepted at
  // once, none of them this one.
  int   p = 0;
  const int N = sizeof(dl_api_buf);
  char* b = dl_api_buf;
  char  tmp[24];

  p += snprintf(b + p, N - p, "{\"sensors\":%d,\"q\":[", numSensors);
  for(size_t i = 0; i < quantities.size() && p < N; i++){
    p += snprintf(b + p, N - p, "%s{\"name\":\"%s\",\"s\":[",
                  i ? "," : "", dl_json_escape(quantities[i].c_str()).c_str());
    for(int s = 0; s < numSensors && p < N; s++){
      Cal::CalibrationCoeffs c = cal.get_calibration_coefficients(quantities[i], s);
      p += snprintf(b + p, N - p, "%s{\"gain\":%s", s ? "," : "",
                    dl_json_num(c.gain, tmp, sizeof(tmp), 6));
      p += snprintf(b + p, N - p, ",\"offset\":%s",
                    dl_json_num(c.offset, tmp, sizeof(tmp), 6));
      p += snprintf(b + p, N - p, ",\"flag\":%d", c.flag);
      // Only the two reference values the table actually shows.
      //
      // This used to emit all six sen_/ref_ x zero/span/diff values per sensor.
      // loadCal() in app.js reads exactly gain, offset, flag, ref_zero and
      // ref_span - the other four were built, sent and discarded. Dropping them
      // roughly halves the payload and, because each one is a name lookup,
      // removes forty of the sixty read_calibration_var() calls a single request
      // used to make.
      //
      // Values that do not exist are omitted rather than sent as null: "absent"
      // already means "not calibrated" everywhere else in this firmware, and
      // fmt() in the browser renders a missing field as an em dash either way.
      for(const char* t : {"zero", "span"}){
        if(p >= N) break;
        float rv = cal.read_calibration_var("ref", t, quantities[i], s);
        if(!isnan(rv)) p += snprintf(b + p, N - p, ",\"ref_%s\":%s", t,
                                     dl_json_num(rv, tmp, sizeof(tmp), 4));
      }
      p += snprintf(b + p, N - p, "}");
    }
    p += snprintf(b + p, N - p, "]}");
  }
  p += snprintf(b + p, N - p, "]}");

  // snprintf reports what it WOULD have written, so an overrun is silently
  // truncated into invalid JSON. Say so rather than serve a broken payload.
  if(p >= N){
    Serial.printf("dl_send_cal_json truncated (%d >= %d)\n", p, N);
    p = snprintf(b, N, "%s", "{\"err\":\"calibration payload too large\"}");
  }
  dl_send_json(h, b, (size_t)p);
  dl_close_soon(h);
}

// ---------------------------------------------------------------------------
//  WiFi
// ---------------------------------------------------------------------------

// Filled by the library's scanner, which is one-shot and starts only when we ask
// it to (see the ap_scan_on_demand hunk in scripts/patches/h4plugins.patch). It
// used to run on a permanent 5 s timer armed by _startAP(); on the ESP8266 that
// took the radio off the AP's channel and heap-allocated the result array over
// and over, which panicked the device roughly every 12 seconds and made it
// impossible for a client to finish DHCP. Never call WiFi.scanNetworks() here
// directly - the SDK has exactly one scanner and these entry points own it.
extern std::vector<std::string> wf_ssids;
extern bool h4pWiFiScanBusy();
extern bool h4pWiFiScanStart();
extern bool h4pWiFiScanRunning();   // a pass is genuinely under way, not just claimed
extern bool h4pWiFiScanRestoreAP(); // put the radio back to AP; false if it would not go

// Two entry points, deliberately.
//
// GET /api/wifi/scan REPORTS ONLY. It used to start a scan whenever it found none
// running, which made a poll loop self-sustaining: a sweep finished, the next
// 700 ms poll restarted it, and the response carrying the finished list was
// discarded by the client because it also said "scanning". Nothing appeared for
// fourteen seconds. A GET that mutates the thing it is polling cannot work.
//
// POST /api/wifi/scan starts one. Beginning a scan is an action; asking how it is
// going is not.
static void dl_send_scan_json(H4AW_HTTPHandler* h){
  WiFiMode_t wm = WiFi.getMode();
  bool apMode   = (wm == WIFI_AP || wm == WIFI_AP_STA);

  int p = 0;
  const int N = sizeof(dl_api_buf);
  char* b = dl_api_buf;
  p += snprintf(b + p, N - p, "{\"nets\":[");
  if(apMode){
    for(size_t i = 0; i < wf_ssids.size() && p < N; i++){
      p += snprintf(b + p, N - p, "%s\"%s\"", i ? "," : "",
                    dl_json_escape(wf_ssids[i]).c_str());
    }
  }
  p += snprintf(b + p, N - p, "],\"scanning\":%s", h4pWiFiScanBusy() ? "true" : "false");
  if(!apMode) p += snprintf(b + p, N - p, ",\"error\":\"the logger is already on a network\"");
  p += snprintf(b + p, N - p, "}");
  if(p >= N){ p = snprintf(b, N, "%s", "{\"nets\":[],\"scanning\":false,\"error\":\"too many networks to list\"}"); }
  dl_send_json(h, b, (size_t)p);
  dl_close_soon(h);
}

static void dl_start_scan(H4AW_HTTPHandler* h){
  WiFiMode_t wm = WiFi.getMode();
  if(wm != WIFI_AP && wm != WIFI_AP_STA){
    const char* e = "{\"ok\":false,\"msg\":\"the logger is already on a network\"}";
    h->send(200, "application/json", strlen(e), e);
    dl_close_soon(h);
    return;
  }
  // Wait for a quiet moment rather than refusing outright.
  //
  // h4pWiFiScanStart() needs ~3200 free heap and a ~1200-byte block, because a
  // scan spends memory twice: bringing the station interface up, and then one
  // contiguous bss_info array in _scanDone (measured: an 880-byte allocation
  // whose failure panicked the device). Those figures are real, but free heap
  // during and just after a page interaction is 1800-2100 - so testing once, on
  // the next tick, refused every single attempt: "SCAN REFUSED: heap 2056<3200",
  // four times in one session, with the user having pressed the button.
  //
  // The device does return to 4000-6600 between requests. So keep asking for a
  // few seconds instead of giving up on the first look. The user pressed a
  // button; the right behaviour is to honour it as soon as it is safe, and to
  // say so plainly if it never becomes safe.
  static H4_TIMER scan_wait = nullptr;
  static uint8_t  scan_tries = 0;
  if(scan_wait){ h4.cancel(scan_wait); scan_wait = nullptr; }
  scan_tries = 0;
  // 800 ms, not 400. Each tick sweeps the open connections and provokes a poll
  // from the client, and both allocate - so a fast retry spends the window it is
  // waiting through. The check itself is cheap; the traffic around it is not.
  scan_wait = h4.every(800, []{
    // Give memory back before asking for more - but from IN HERE, not from the
    // request handler.
    //
    // The first version of this released connections at the top of
    // dl_start_scan, which closed every open connection INCLUDING the one about
    // to carry this request's own reply. The reply went nowhere: the browser
    // reported "Could not start the scan", and the same thing happening mid-page
    // gave ERR_EMPTY_RESPONSE. A straightforward self-inflicted regression -
    // scanning had been working before it.
    //
    // By the time this timer first fires the handler has long since replied and
    // dl_close_soon has released its socket, so everything still open here is
    // genuinely idle. Each one is ~1.5KB and all of them are about to be useless
    // anyway, because the radio leaves the access point's channel for the
    // duration of the sweep.
    // A sweep is genuinely under way - leave it alone and stand down. Note this
    // is h4pWiFiScanRunning(), not h4pWiFiScanBusy(): "busy" goes true the moment
    // the scan is CLAIMED, before the first pass has been attempted, so watching
    // it made this loop cancel itself and then the attempt would die quietly with
    // nothing left to try again. That is how a single refusal killed the button.
    if(h4pWiFiScanRunning()){
      if(scan_wait){ h4.cancel(scan_wait); scan_wait = nullptr; }
      return;
    }

    auto conns = H4AsyncClient::openConnections;   // copy: close() mutates it
    for(auto c : conns) if(c) c->close();

    // Ask again only when the previous attempt has finished failing. Each tick
    // re-measures, having just released whatever the browser left open, so this
    // waits for conditions to actually change rather than repeating the same
    // question faster.
    // Defragment the only way this allocator allows: delete, do not move.
    //
    // umm_malloc never relocates a live block, so there is no compaction to
    // call - but a block that nothing needs any more is a different matter. A
    // TIME_WAIT pcb is ~200 bytes lwIP holds purely so a late duplicate segment
    // cannot be misdelivered to a new connection reusing the same 4-tuple. It
    // is tiny, and if it lands mid-heap it splits the largest free region in
    // half. Measured across a single page load:
    //
    //   [ap] stations=1 heap=4720 block=3496 tw=0    <- associated, before the page
    //   [http] done heap=3552 block=1696
    //   [ap] stations=1 heap=5616 block=2016 tw=1    <- heap UP, block HALVED
    //
    // Free heap rose and the largest block still halved, which is fragmentation
    // and nothing else. That is why the scan floor has been "sometimes
    // reachable": the achievable block swung between 2016 and 3240 depending on
    // where one pcb happened to land, and no floor can be right for both.
    //
    // The scan is the one operation that needs a large CONTIGUOUS block, and
    // this is the moment it is about to ask for one. Purging here is bounded and
    // deliberate: on a private AP with one phone attached, the connection this
    // protects against reusing does not exist, and the alternative is a scan
    // that never runs.
    //
    // NOTE: this has never yet been observed to do anything. In the one capture
    // where it could have mattered there were no TIME_WAIT pcbs to purge and the
    // scan succeeded on its own. Kept because it is cheap, bounded, and fires
    // only in the moment before a scan - but it is unproven, so do not treat it
    // as load-bearing.
    for(struct tcp_pcb* p = tcp_tw_pcbs; p; ){
      struct tcp_pcb* next = p->next;   // tcp_abort unlinks p
      tcp_abort(p);
      p = next;
    }

    // Stop on "I claimed it", not on "I caught it running".
    //
    // This loop used to exit only via h4pWiFiScanRunning() above, which is
    // scanner != nullptr - true only from the moment a pass genuinely starts
    // (after a 300 ms settle) until the sweep ends. That was survivable while
    // scans were being refused. Once they started working the sweep became
    // shorter than the 800 ms tick, so a whole scan could begin and finish
    // BETWEEN two ticks: the loop never observed it, never cancelled itself,
    // and started another one. Measured: one button press, four sweeps -
    //
    //   SCAN COMPLETE: 4 network(s)      x3, back to back
    //   [ap] ... heap=5328 block=2560 accepting     (before)
    //   [ap] ... heap=1936 block=1104 REFUSING      (after)
    //
    // and from there every accept was refused, 28 of them, so the results the
    // device had successfully found could never be fetched. The scan worked and
    // the user still saw no networks.
    //
    // h4pWiFiScanStart() returns true when a sweep is claimed (and immediately
    // true if one is already claimed), so this is the honest success signal and
    // it cannot be missed by being sampled at the wrong moment. It also makes
    // the h4pWiFiScanBusy() pre-check redundant - that is the same test the
    // function does on its own first line.
    if(h4pWiFiScanStart()){
      if(scan_wait){ h4.cancel(scan_wait); scan_wait = nullptr; }
      return;
    }

    if(++scan_tries >= 15){                          // ~12 s at the 800 ms tick
      Serial.println(F("SCAN GAVE UP: conditions never allowed a scan to start"));
      if(scan_wait){ h4.cancel(scan_wait); scan_wait = nullptr; }
    }
  });
  const char* ok = "{\"ok\":true}";
  h->send(200, "application/json", strlen(ok), ok);
  dl_close_soon(h);
}

static void dl_wifi_connect(H4AW_HTTPHandler* h){
  std::string ssid = h->params().count("ssid") ? h->params()["ssid"] : "";
  std::string psk  = h->params().count("psk")  ? h->params()["psk"]  : "";

  if(ssid.empty()){
    const char* err = "{\"ok\":false,\"msg\":\"no ssid\"}";
    h->send(400, "application/json", strlen(err), err);
    dl_close_soon(h);
    return;
  }

  const char* ok = "{\"ok\":true,\"msg\":\"connecting, the device will reboot\"}";
  h->send(200, "application/json", strlen(ok), ok);
  dl_close_soon(h);

  // Answer first, act afterwards. Switching to station mode drops the access
  // point the caller is almost certainly connected through, so doing it inline
  // means the reply never arrives and the user cannot tell success from a
  // typo'd password.
  h4.once(1500, [ssid, psk]{
    // psk BEFORE ssid: setting ssid while the station is up triggers
    // H4P_WiFi's own _restart() (H4P_WiFi.cpp:373), which would otherwise
    // reconnect using the previous password.
    h4p.gvSetstring(pskTag(),  psk,  true);
    h4p.gvSetstring(ssidTag(), ssid, true);
    Serial.printf("Provisioning: joining %s\n", ssid.c_str());
    h4wifi.HAL_WIFI_startSTA();
    // Deliberately not the "Go" global: gvSetstring suppresses no-op writes,
    // so once Go is "1" a second attempt would never fire the event again.
    h4.once(1000, []{ h4pevent(h4pSrc, H4PE_REBOOT, wifiTag()); });
  });
}

// ---------------------------------------------------------------------------
//  Registration
//
//  MUST run from inside h4wifi.hookWebserver(). _startWebserver() calls
//  reset(), which deletes every handler, and it runs more than once per boot
//  (from _gotIP and again from _startAP), so anything registered once at setup
//  would vanish on the first WiFi transition.
// ---------------------------------------------------------------------------
void dl_register_api(){
  // Logo.gif is the only image we still ship; without this the library's MIME
  // table falls back to text/plain and the browser refuses to render it.
  H4AW_HTTPHandler::mimeTypes["gif"] = "image/gif";
  H4AW_HTTPHandler::mimeTypes["svg"] = "image/svg+xml";

  // The stock cache header is a year, which means a rebuilt stylesheet never
  // reaches a browser that has already visited this device.
  h4wifi._cacheAge = 600;

  // Take over "/". The library registered its own root handler pointing at the
  // now-deleted sta.htm before this hook runs, and routing is first-match-wins.
  // _handlers is public, so this needs no library patch. "/" is compared
  // exactly (H4AsyncHandlers.cpp:105 falls through to == for a 1-char path),
  // so our handler cannot swallow any other route.
  for(auto it = h4wifi._handlers.begin(); it != h4wifi._handlers.end(); ++it){
    if((*it)->_path == "/" && (*it)->_verb == HTTP_GET){
      delete *it;
      h4wifi._handlers.erase(it);
      break;
    }
  }

  // One route for the whole UI: scripts/build_web.py inlines the CSS and the JS
  // into this page, so there is nothing else to fetch. Every route removed here
  // is both a heap-allocated handler object and, more importantly, one fewer
  // simultaneous connection against an lwIP built with MEMP_NUM_TCP_PCB 5.
  h4wifi.on("/", HTTP_GET, [](H4AW_HTTPHandler* h){ dl_send_static(h, "/h4/app.htm.gz", "text/html", true); });

  // --- Captive portal ------------------------------------------------------
  // The DNS server already answers every name with 192.168.4.1, so the phone's
  // connectivity probe arrives here. What it got until now was 401: the library's
  // file catch-all (H4AW_HTTPHandlerFile) matches EVERY GET, authentication fails,
  // and the handler loop stops there - so the probe never even reached the 404
  // handler. Android wants 204 for "online" and treats 3xx as "portal"; 401 is
  // neither, so it marked the network "no internet access", kept mobile data as
  // the default route and never offered to sign in.
  //
  // These must be registered unauthenticated (the false argument), or they hit
  // exactly the same 401. Registered here, they precede the file catch-all, and
  // they cannot shadow "/" because that is compared exactly - _match() only does
  // prefix matching for paths longer than one character.
  static const char* const portal_probes[] = {
    "/generate_204",        // Android
    "/gen_204",             // Android (older)
    "/canonical.html",      // Android fallback
    "/success.txt",         // Firefox (detectportal.firefox.com)
    "/hotspot-detect.html", // iOS / macOS
    "/ncsi.txt",            // Windows
    "/connecttest.txt",     // Windows
  };
  for(auto probe : portal_probes){
    h4wifi.on(probe, HTTP_GET, [](H4AW_HTTPHandler* h){
      h->addHeader("Location", "http://192.168.4.1/");
      h->addHeader("Cache-Control", "no-store");
      // 302, not the library's redirect() helper: that sends 303, which some
      // captive-portal detectors do not treat as a sign-in prompt.
      static const char body[] = "redirecting";
      h->send(302, "text/plain", sizeof(body)-1, body);
      dl_close_soon(h);
    }, false);
  }

  h4wifi.on("/api/status", HTTP_GET, [](H4AW_HTTPHandler* h){
    size_t n = dl_status_json();          // payload is in dl_api_buf, no copy
    dl_send_json(h, dl_api_buf, n);
    dl_close_soon(h);
  });
  h4wifi.on("/api/cal",       HTTP_GET,  dl_send_cal_json);
  h4wifi.on("/api/wifi/scan", HTTP_GET,  dl_send_scan_json);   // report only
  h4wifi.on("/api/wifi/scan", HTTP_POST, dl_start_scan);       // begin a sweep
  h4wifi.on("/api/wifi/connect", HTTP_POST, dl_wifi_connect);

  // No /api/events handler and no push timer: see the note above dl_toast(). The
  // client polls /api/status instead, and server-initiated messages ride along
  // with it.
  Serial.printf("Web API ready on port %d\n", H4P_WEBSERVER_PORT);
}
