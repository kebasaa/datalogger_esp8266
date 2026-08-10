/* Datalogger web UI.
   Plain ES2018, no framework and no build step beyond minify+gzip: the whole
   file has to fit in a few KB because the device serves it out of a heap that
   refuses new connections below ~13 KB free. */
'use strict';

var $ = function (id) { return document.getElementById(id); };
var state = { status: null, quantities: [], poll: 0, msgseq: undefined, clockBase: 0, clockAt: 0 };

/* ------------------------------------------------------------------ util -- */

function toast(msg) {
  var box = $('toasts');
  var el = document.createElement('div');
  el.className = 'toast';
  el.textContent = msg;
  box.appendChild(el);
  setTimeout(function () { el.remove(); }, 6000);
}

function fmt(v, dp, unit) {
  if (v === null || v === undefined || v !== v) return '—';
  var s = typeof v === 'number' ? v.toFixed(dp === undefined ? 0 : dp) : String(v);
  return unit ? s + ' ' + unit : s;
}

function kb(bytes) { return fmt(bytes / 1024, 1, 'kB'); }

function duration(ms) {
  var s = Math.floor(ms / 1000);
  var d = Math.floor(s / 86400), h = Math.floor(s % 86400 / 3600), m = Math.floor(s % 3600 / 60);
  if (d) return d + 'd ' + h + 'h';
  if (h) return h + 'h ' + m + 'm';
  return m + 'm ' + (s % 60) + 's';
}

/* ------------------------------------------------------- one at a time -- */

/* Every request on this page goes through here, and only one is ever in flight.

   The device has 5-7 KB of free heap with a phone attached and ONE accepted
   connection costs about 1.5 KB - the client object, a full copy of the request
   held for the whole handler, and a ~544-byte transmit-queue node allocated
   before a single byte is sent. Its accept gate is checked once, in the lwIP
   accept callback, before any of that is allocated, and never re-checked - so
   several requests arriving together all pass a gate sized for one, and the
   device panics on whatever small allocation comes next.

   The page was guaranteeing exactly that. On load it fired GET / (socket held up
   to 3 s), then /api/status, then loadCal() unawaited from apply(), which let the
   promise chain reach connect() and fire a SECOND status poll in the same turn:
   four sockets, ~6 KB, against 5-7 KB free.

   So: serialise. Each request waits for the previous to settle - settle, not
   succeed, or one failure would wedge the queue for good. The AbortController
   bounds how long any single request can hold the slot. */
var gate = Promise.resolve();

function req(path, opts, ms) {
  var go = function () {
    var ctl = window.AbortController ? new AbortController() : null;
    var o = opts || {};
    if (ctl) {
      o.signal = ctl.signal;
      var timer = setTimeout(function () { ctl.abort(); }, ms || 8000);
    }
    return fetch(path, o).then(
      function (r) { clearTimeout(timer); return r; },
      function (e) { clearTimeout(timer); throw e; }
    );
  };
  var p = gate.then(go, go);              // run whatever happened before
  gate = p.then(function () {}, function () {});
  return p;
}

function getJSON(path, opts) {
  return req(path, opts).then(function (r) { return r.json(); });
}

/* Run a device command. Same surface the serial port uses: the path after
   /rest is the command and its parameters, separated by slashes. */
/* Two different failures, and both have to be visible. j.res is the device
   rejecting the command; a rejected promise is the request never getting there
   at all - which on this hardware is common, and used to produce absolutely
   nothing on screen. A button that silently does nothing is worse than one that
   says why it could not. */
function rest(parts) {
  return getJSON('/rest/' + parts.map(encodeURIComponent).join('/')).then(function (j) {
    if (j.res) { toast('Error: ' + j.msg); throw new Error(j.msg); }
    return j;
  }, function (e) {
    toast('No reply from the logger - try again');
    throw e;
  });
}

/* A command whose reply the user should see, e.g. a button press. */
function run(parts) {
  return rest(parts).then(function (j) {
    if (j.lines && j.lines.length) toast(j.lines[j.lines.length - 1]);
    return j;
  }).catch(function () { /* rest() already reported it */ });
}

/* --------------------------------------------------------------- theming -- */

/* The manual light/dark toggle was removed to keep the built page under the
   build's size cap. The page still follows the phone's system theme through
   prefers-color-scheme in app.css - only the manual override is gone. */

/* ------------------------------------------------------------------ tabs -- */

function initTabs() {
  var buttons = [].slice.call(document.querySelectorAll('#tabs button'));
  buttons.forEach(function (b) {
    b.addEventListener('click', function () {
      buttons.forEach(function (o) {
        var on = o === b;
        o.setAttribute('aria-selected', on ? 'true' : 'false');
        $('p-' + o.dataset.tab).hidden = !on;
      });
      if (b.dataset.tab === 'cal') loadCal();
      // Opening the wifi tab READS the list; it never starts a scan. Scanning
      // takes the radio off the hotspot's channel and drops whoever is looking
      // at this page, so a sweep still only happens when the user asks.
      //
      // The GET is safe by construction - /api/wifi/scan was split into a POST
      // that starts a sweep and a GET that only reports, precisely so a poll
      // could not restart the scan it was observing. Without this the device's
      // remembered list was invisible until you pressed Search, which is also
      // the one thing that could discard it.
      if (b.dataset.tab === 'wifi') {
        getJSON('/api/wifi/scan')
          .then(function (j) { if (j && j.nets) renderNets(j.nets); })
          .catch(function () { /* nothing to show yet; not worth a toast */ });
      }
    });
  });
}

/* ------------------------------------------------------------ sparklines -- */

var series = {};

function push(key, value) {
  if (value === null || value === undefined || value !== value) return;
  var a = series[key] || (series[key] = []);
  a.push(value);
  if (a.length > 120) a.shift();          // ~2 minutes at 1 Hz
}

/* Inline SVG rather than a charting library: a polyline over a normalised
   viewBox is all a sparkline is, and it costs a few hundred bytes. */
function spark(key) {
  var a = series[key];
  if (!a || a.length < 2) return '';
  var lo = Math.min.apply(null, a), hi = Math.max.apply(null, a);
  var span = (hi - lo) || 1;
  var pts = a.map(function (v, i) {
    return (i / (a.length - 1) * 100).toFixed(2) + ',' + (100 - (v - lo) / span * 100).toFixed(2);
  }).join(' ');
  return '<svg class="spark" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">'
       + '<polyline points="' + pts + '"/></svg>';
}

/* ---------------------------------------------------------------- status -- */

function dot(cls, label) { return '<span class="' + cls + '"><span class="dot"></span>' + label + '</span>'; }

/* The clock, ticking between polls.

   The status poll runs every 10 s, so a clock painted only from the reply sat
   frozen for ten seconds at a time and looked broken - which is exactly what it
   looked like. Polling faster is the obvious fix and the wrong one: each poll is
   a fresh TCP connection against an lwIP with five in total, and connection
   pressure is what this whole exercise has been about.

   So take the time from the device and advance it locally, resyncing on every
   reply. The displayed second hand is honest to within one poll of drift, and it
   costs the logger nothing.

   Written tight rather than spread out: the built page is close to the build's
   own size cap, and comments are stripped by the minifier so they cost nothing
   while the code does. */
function startClock() {
  setInterval(function () {
    var e = $('clk');
    if (state.clockBase && e) e.textContent =
      new Date(state.clockBase + (Date.now() - state.clockAt)).toISOString().slice(0, 19).replace('T', ' ');
  }, 1000);
}

/* Parse "YYYY-MM-DD HH:MM:SS" as UTC. Date.parse on that form without the Z is
   implementation-defined and several browsers read it as local time, which would
   silently shift the display by the phone's offset. */
function syncClock(n) {
  var m = n ? Date.parse(n.replace(' ', 'T') + 'Z') : NaN;
  state.clockBase = isNaN(m) ? 0 : m;
  state.clockAt = Date.now();
}

function card(title, rows, extra) {
  var body = rows.map(function (r) {
    return '<dt>' + r[0] + '</dt><dd>' + r[1] + '</dd>';
  }).join('');
  return '<div class="card"><h2>' + title + '</h2><dl>' + body + '</dl>' + (extra || '') + '</div>';
}

/* Send this device's clock to the logger, in UTC.
   getUTC* deliberately, never getHours()/getMonth() - those are local time, and
   the mistake would be invisible anywhere the browser happens to sit on UTC and
   silently an hour or more out everywhere else. getUTCMonth() is 0-based. */
function setClock() {
  var d = new Date();
  rest(['settime',
        d.getUTCFullYear(), d.getUTCMonth() + 1, d.getUTCDate(),
        d.getUTCHours(), d.getUTCMinutes(), d.getUTCSeconds()])
    .then(poll)
    .catch(function () { /* rest() reports both failure kinds now */ });
}

/* Delegated, and bound once. The banner's contents are rewritten as the status
   changes, so a listener attached to the button itself would be thrown away with
   it. */
function initBanner() {
  $('banner').addEventListener('click', function (e) {
    if (e.target && e.target.id === 'set-clock') setClock();
  });
}

function renderStatus(s) {
  syncClock(s.now);
  // The single most important thing on the page. Until the logger has a sane
  // time from GPS or the RTC it does not record anything at all, and every
  // counter below stays at zero - which would otherwise look perfectly healthy.
  var banner = $('banner');
  if (!s.boot.locked) {
    // Rewrite only when the text actually changes. renderStatus runs on every
    // poll, and reassigning innerHTML destroys and recreates the button - a tap
    // landing in that window is simply lost, which is indistinguishable from a
    // button that does not work. The click handler is delegated from the banner
    // in initBanner(), so it survives whatever happens to the child nodes.
    var html = 'Waiting for a valid time (' + s.boot.src + '). '
      + 'Nothing is being recorded until the GPS gets a fix. '
      + 'You can start logging now using this device’s clock — GPS will '
      + 'take over and correct it once it has a fix. '
      + '<button id="set-clock" type="button">Set clock</button>';
    if (banner.innerHTML !== html) banner.innerHTML = html;
    banner.hidden = false;
  } else {
    banner.hidden = true;
  }

  $('devname').textContent = s.dev;
  $('subtitle').textContent = 'v' + s.ver + ' · up ' + duration(s.up)
    + ' · ' + s.n + ' samples';

  var out = [];

  out.push(card('Logger', [
    ['Recording', s.boot.locked ? dot('ok', 'yes') : dot('warn', 'not yet')],
    // The device clock as of this request, so setting it by hand shows up at
    // once rather than at the next sample.
    ['Clock (UTC)', '<span id="clk">' + (s.now || '—') + '</span>'],
    ['Samples', s.n],
    ['Uptime', duration(s.up)],
    ['Time source', s.boot.src],
    ['Last reset', s.boot.reset]
  ]));

  if (s.gps) {
    var g = s.gps;
    out.push(card('GPS', [
      ['Fix', g.loc ? dot('ok', 'yes') : dot('bad', 'no')],
      ['Time', g.fresh && g.sane ? dot('ok', 'good') : dot('warn', g.date ? 'stale' : 'none')],
      ['Satellites', g.sat],
      ['HDOP', fmt(g.hdop, 1)],
      ['UTC', g.ts || '—'],
      ['Position', g.lat === null ? '—' : fmt(g.lat, 5) + ', ' + fmt(g.lon, 5)],
      ['Altitude', fmt(g.alt, 0, 'm')],
      ['Stale streak', g.stale],
      ['Recoveries', g.rec],
      ['Quarantined', g.quar ? dot('bad', 'yes') : 'no']
    ]));
  }

  if (s.bat) {
    out.push(card('Battery', [
      ['Charge', fmt(s.bat.pc, 0, '%')],
      ['Voltage', fmt(s.bat.mv, 0, 'mV')]
    ]));
  }

  if (s.sd) {
    var d = s.sd;
    out.push(card('MicroSD', [
      ['Last write', d.st === 0 ? dot('ok', 'ok') : dot('bad', 'error 0x' + d.st.toString(16))],
      ['Free', fmt(d.free, 0, 'MB')],
      ['Capacity', fmt(d.cap, 0, 'MB')],
      ['Card missing', d.miss],
      ['Open failures', d.open],
      ['Write failures', d.write]
    ]));
  }

  // No trend line here any more - the three numbers say everything the trend did,
  // and the page has to fit through a device with a few hundred bytes of
  // contiguous heap when a request is in flight.
  out.push(card('Memory', [
    ['Free heap', kb(s.heap.free)],
    ['Largest block', kb(s.heap.max)],
    ['Lowest seen', kb(s.heap.min)]
  ]));

  var busRows = s.i2c.bus.map(function (b) {
    return ['Channel ' + b.ch,
      (b.ce ? dot('bad', b.ce + ' failing now') : dot('ok', 'ok'))
      + ' <span class="muted">' + b.e + ' err / ' + b.r + ' rec</span>'];
  });
  out.push(card('I2C buses', busRows.concat([
    ['Slow reads', s.i2c.slow],
    ['Total errors', s.i2c.err],
    ['Total recoveries', s.i2c.rec]
  ])));

  // These are live streaks, not totals: they reset the moment things recover.
  // Labelling them "since boot" would be actively misleading.
  var sup = s.sup;
  out.push(card('Health', [
    ['Values this sample', dot(sup.missing ? 'warn' : 'ok', sup.valid + ' ok, ' + sup.missing + ' missing')],
    ['Sensor loss streak', sup.crit],
    ['GPS/I2C fault streak', sup.hard],
    ['Supervisor resets', sup.rst + ' <span class="muted">(this boot)</span>'],
    ['Restart pending', sup.pend ? dot('bad', sup.why || 'yes') : 'no']
  ]));

  $('cards').innerHTML = out.join('');
}

/* ----------------------------------------------------------- calibration -- */

function renderCal(s) {
  var c = s.cal;
  $('cal-status').textContent = c.status;
  $('cal-start').disabled = c.open;
  $('cal-stop').disabled = !c.open;
  $('cal-discard').disabled = !c.open;

  var busy = c.running;
  ['cal-do-zero', 'cal-do-span', 'cal-do-low', 'cal-do-high', 'cal-reset'].forEach(function (id) {
    $(id).disabled = busy;
  });

  if (c.raw !== undefined) {
    $('cal-raw').textContent = fmt(c.raw, 3);
    $('cal-cal').textContent = fmt(c.cal, 3);
    push('raw', c.raw);
    $('cal-spark').innerHTML = spark('raw');
  } else {
    $('cal-raw').textContent = busy ? 'measuring…' : '—';
    $('cal-cal').textContent = '—';
  }
}

/* Populate the quantity pickers from whatever the firmware was built with,
   rather than from a hard-coded list that can silently drift out of date. */
function fillQuantities(list) {
  state.quantities = list;
  [['cal-gas', false], ['cal-dgas', true]].forEach(function (pair) {
    var sel = $(pair[0]);
    if (sel.options.length) return;
    var html = pair[1] ? '<option value="all">All</option>' : '';
    html += list.map(function (q) {
      return '<option value="' + q + '">' + q + '</option>';
    }).join('');
    sel.innerHTML = html;
  });
  var sensors = $('cal-sensor');
  if (!sensors.options.length) {
    var n = state.sensorCount || 2, o = '';
    for (var i = 0; i < n; i++) o += '<option value="' + i + '">' + i + '</option>';
    sensors.innerHTML = o;
  }
}

/* Guard j.q before touching it.

   The device can answer with an error object instead of a table. When it did,
   j.q was undefined, j.q.map threw, and state.quantities stayed empty - which
   left the "load it once" guard in apply() permanently true, so /api/cal was
   re-fired from EVERY ten-second status poll, for ever. A device already short
   of memory got a request storm precisely because it had said it was short of
   memory. Report it and stop. */
function loadCal() {
  req('/api/cal')
    .then(function (r) { return r.json(); })
    .then(function (j) {
      if (!j || !j.q) { toast(j && j.err ? j.err : 'No calibration data'); return; }
      state.sensorCount = j.sensors;
      fillQuantities(j.q.map(function (q) { return q.name; }));

      var head = '<tr><th>Quantity</th><th>Sensor</th><th>Gain</th><th>Offset</th>'
               + '<th>Flag</th><th>Zero</th><th>Span</th></tr>';
      var body = '';
      j.q.forEach(function (q) {
        q.s.forEach(function (c, i) {
          body += '<tr><td>' + q.name + '</td><td>' + i + '</td>'
                + '<td>' + fmt(c.gain, 4) + '</td><td>' + fmt(c.offset, 4) + '</td>'
                + '<td>' + c.flag + '</td>'
                + '<td>' + fmt(c.ref_zero, 2) + '</td><td>' + fmt(c.ref_span, 2) + '</td></tr>';
        });
      });
      $('cal-table').innerHTML = head + body;
    })
    .catch(function () { toast('Could not read calibration'); });
}

function select() {
  return run(['cal_select', $('cal-gas').value, $('cal-sensor').value, $('cal-dgas').value]);
}

/* Wiring is table-driven rather than nine near-identical addEventListener calls.
   Each entry is [element id, function returning the command, reload after?] -
   returning null from the argument function aborts without sending. */
function initCal() {
  [
    ['cal-start',   function () { return ['cal_start']; }],
    ['cal-stop',    function () { return ['cal_stop']; },   1],
    ['cal-discard', function () { return ['cal_discard']; }],
    ['cal-do-zero', function () {
      return ['cal', 'zero', $('cal-gas').value, $('cal-sensor').value,
              $('cal-zero-ref').value || '0'];
    }],
    ['cal-do-span', function () {
      var v = $('cal-span-ref').value;
      if (v === '') { toast('Enter a span reference value first'); return null; }
      return ['cal', 'span', $('cal-gas').value, $('cal-sensor').value, v];
    }],
    ['cal-do-low',  function () { return ['cal', 'diff', $('cal-dgas').value, 'low']; }],
    ['cal-do-high', function () { return ['cal', 'diff', $('cal-dgas').value, 'high']; }],
    ['cal-reset',   function () {
      if (!confirm('Clear every stored calibration coefficient?')) return null;
      return ['cal_reset'];
    }, 1]
  ].forEach(function (e) {
    $(e[0]).addEventListener('click', function () {
      var args = e[1]();
      if (!args) return;
      var p = run(args);
      if (e[2]) p.then(loadCal);
    });
  });

  $('cal-refresh').addEventListener('click', loadCal);
  ['cal-gas', 'cal-sensor', 'cal-dgas'].forEach(function (id) {
    $(id).addEventListener('change', select);
  });
}

/* ------------------------------------------------------------------ wifi -- */

function renderWifi(s) {
  $('wifi-now').innerHTML =
      '<dt>Mode</dt><dd>' + (s.wifi.ap ? 'Access point' : 'Joined a network') + '</dd>'
    + '<dt>Network</dt><dd>' + (s.wifi.ssid || '—') + '</dd>'
    + '<dt>Address</dt><dd>' + s.wifi.ip + '</dd>'
    + (s.wifi.ap ? '' : '<dt>Signal</dt><dd>' + s.wifi.rssi + ' dBm</dd>');
}

/* The device sweeps one channel at a time and publishes what it has found so
   far, so results are rendered on every poll rather than only when the sweep
   finishes. The old code checked "scanning" first and threw away a fully
   populated list whenever a scan was still running, which is why nothing
   appeared for fourteen seconds. */
var scanPolls = 0;

function renderNets(nets) {
  var sel = $('wifi-ssid'), keep = sel.value;
  sel.innerHTML = nets.length
    ? nets.map(function (n) { return '<option>' + n + '</option>'; }).join('')
    : '<option value="">(none found yet — or type the name below)</option>';
  if (keep) sel.value = keep;          // do not fight the user mid-sweep
}

function pollScan() {
  req('/api/wifi/scan')
    .then(function (r) { return r.json(); })
    .then(function (j) {
      if (j.error) {
        state.scanning = false;
        $('wifi-rescan').disabled = false;
        $('wifi-hint').textContent = j.error;
        return;
      }
      renderNets(j.nets);              // show progress, do not wait for the end
      // Poll slowly until the sweep is actually running, quickly once it is.
      //
      // Every poll is a TCP connection, and a connection is ~1.5 KB of heap in
      // pieces - which is exactly what stops the device finding the contiguous
      // block a scan needs. Polling every 1.5 s while it waited meant the client
      // was destroying the headroom it was waiting for: the block sat pinned at
      // 1136 for the whole ten-second window, ~40 bytes under the threshold.
      //
      // Once "scanning" is true the memory question is already settled, so there
      // is no harm in watching closely for partial results.
      if (j.scanning && scanPolls++ < 20) {
        setTimeout(pollScan, 1500);
        return;
      }
      if (!j.scanning && scanPolls++ < 8) {   // still waiting to start - stand back
        setTimeout(pollScan, 3000);
        return;
      }
      state.scanning = false;
      $('wifi-rescan').disabled = false;
      $('wifi-hint').textContent = j.nets.length
        ? 'Pick a network, enter its password, then press Connect.'
        : 'Nothing found on channels 1, 6 or 11. If your network uses another '
          + 'channel it will not be listed — type its name below instead.';
    })
    .catch(function () {
      // Keep trying: a pass can briefly stall a request even though the sweep is
      // per-channel, and giving up here would leave the list half-built.
      if (scanPolls++ < 20) { setTimeout(pollScan, 1500); return; }
      state.scanning = false;
      $('wifi-rescan').disabled = false;
      toast('Lost contact while scanning');
    });
}

function loadScan() {
  scanPolls = 0;
  // Suppresses the 10 s status poll for the duration - see poll(). Every exit
  // path below and in pollScan() must clear it, or the page stops updating.
  state.scanning = true;
  $('wifi-rescan').disabled = true;
  $('wifi-hint').textContent = 'Searching channels 1, 6 and 11 — the page pauses for a few seconds…';
  // POST starts a sweep; GET only reports. They were the same call, so every
  // poll restarted the scan it was trying to observe.
  req('/api/wifi/scan', { method: 'POST' })
    .then(function (r) { return r.json(); })
    .then(function (j) {
      if (j.ok === false) {
        state.scanning = false;
        $('wifi-rescan').disabled = false;
        $('wifi-hint').textContent = j.msg;
        return;
      }
      setTimeout(pollScan, 4000);   // let the block recover before the first look
    })
    .catch(function () {
      state.scanning = false;
      $('wifi-rescan').disabled = false;
      toast('Could not start the scan');
    });
}

function initWifi() {
  $('wifi-rescan').addEventListener('click', loadScan);
  $('wifi-join').addEventListener('click', function () {
    var ssid = $('wifi-manual').value.trim() || $('wifi-ssid').value;
    if (!ssid) { toast('Pick or type a network name'); return; }

    var body = 'ssid=' + encodeURIComponent(ssid) + '&psk=' + encodeURIComponent($('wifi-psk').value);
    // The header must be exactly this. H4AsyncWebServer compares the whole
    // Content-Type string, so the "; charset=UTF-8" a URLSearchParams body
    // would add makes the device parse no parameters at all.
    req('/api/wifi/connect', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: body
    })
      .then(function (r) { return r.json(); })
      .then(function (j) {
        toast(j.msg);
        if (j.ok) {
          $('wifi-hint').textContent = 'Rebooting and joining "' + ssid + '". '
            + 'This page will stop responding; reconnect to that network to reach the logger again.';
        }
      })
      .catch(function () { toast('Could not send the credentials'); });
  });
}

/* --------------------------------------------------------------- console -- */

function initConsole() {
  var out = $('cmd-out');
  function go() {
    var text = $('cmd').value.trim();
    if (!text) return;
    out.textContent += '> ' + text + '\n';
    // Typed as a command path already, so it is passed through as-is rather
    // than being percent-encoded segment by segment.
    req('/rest/' + text)
      .then(function (r) { return r.json(); })
      .then(function (j) {
        out.textContent += (j.res ? 'error: ' + j.msg : (j.lines || []).join('\n') || 'ok') + '\n\n';
        out.scrollTop = out.scrollHeight;
      })
      .catch(function (e) { out.textContent += 'failed: ' + e.message + '\n\n'; });
    $('cmd').value = '';
  }
  $('cmd-run').addEventListener('click', go);
  $('cmd').addEventListener('keydown', function (e) { if (e.key === 'Enter') go(); });
}

/* ------------------------------------------------------------- transport -- */

/* Server-initiated messages ("Calibration saved", errors from a /rest command
   that finished after the reply went out) used to arrive on the SSE stream. They
   now ride on the status payload as a message plus a counter, so they survive
   without holding a connection open. */
function showServerMessage(s) {
  if (s.msgseq === undefined) return;
  // On the first poll, adopt whatever the device last said without showing it -
  // otherwise every page load replays an old message as though it just happened.
  if (state.msgseq !== undefined && s.msgseq !== state.msgseq && s.msg) toast(s.msg);
  state.msgseq = s.msgseq;
}

function apply(s) {
  state.status = s;
  renderStatus(s);
  renderCal(s);
  renderWifi(s);
  showServerMessage(s);
  if (!state.quantities.length && s.cal.gas) loadCal();
}

function online(yes) {
  $('conn').textContent = yes ? 'live' : 'reconnecting…';
  $('conn').className = yes ? 'ok' : 'warn';
}

/* One status fetch. Separate from the interval so anything that changes the
   device's state can refresh straight away rather than waiting up to 3 s. */
/* The timeout that used to live here is now in req(), which every request on the
   page shares. A poll that is already waiting its turn is not queued again -
   ticks and visibility changes can arrive faster than the device answers, and a
   backlog of identical status requests is worse than skipping one. */
function poll() {
  // Stand aside while a scan is running. pollScan() is already asking the device
  // for the network list, and the scan's own completion needs every byte it can
  // get - it brings the station interface up and then allocates one contiguous
  // bss_info array, which is where the device has been panicking. Adding a status
  // request to that window buys nothing: the radio is off the AP's channel
  // anyway, so the reply would arrive late or not at all.
  if (state.polling || state.scanning) return Promise.resolve();
  state.polling = true;
  return getJSON('/api/status')
    .then(function (s) { online(true); apply(s); })
    .catch(function () { online(false); })
    .then(function () { state.polling = false; });
}

/* Ten seconds, not three.
   Every poll is a fresh TCP connection - the device sends Connection: close
   because it cannot afford keep-alives against a five-socket pool. Polling every
   3 s meant connection churn that never let the heap return to its idle 8.9 KB:
   it sat at 3.4-5.7 KB indefinitely after a page load, until an allocation of
   52 bytes failed and the logger hung. Nothing on this page changes faster than
   the 20 s sample interval anyway.

   Polling also stops while the page is hidden. A phone in a pocket with the tab
   open was holding the logger at its low-water mark for no reason at all. */
/* Poll NOW, then on the interval, and again whenever the page becomes visible.
   All three matter and only the middle one used to exist:

   - without the immediate poll, a freshly loaded page sat on "connecting..."
     for a full interval before it asked the device anything. Ten seconds of
     looking broken on every single load.
   - the interval skips while document.hidden, which is right (a backgrounded
     tab should not hold connections open against a five-socket device) but left
     no way back: a tab hidden when the tick fired simply missed it. Switching
     between two browsers on a phone hides one and shows the other, so the one
     brought forward could sit indefinitely on stale text.
   - so refresh on visibilitychange, which is exactly the moment the numbers
     matter again. */
function startPolling() {
  if (state.poll) return;
  poll();
  state.poll = setInterval(function () {
    if (!document.hidden) poll();
  }, 10000);
  document.addEventListener('visibilitychange', function () {
    if (!document.hidden) poll();
  });
}

function stopPolling() {
  if (state.poll) { clearInterval(state.poll); state.poll = 0; }
}

/* Polling only. There used to be an EventSource on /api/events here, but an SSE
   stream holds a connection open for as long as the page is on screen, and lwIP
   on this device is built with MEMP_NUM_TCP_PCB 5 - so the dashboard was sitting
   on a fifth of the device's entire TCP capacity just to receive an update it
   could ask for. Server-pushed messages now arrive as msg/msgseq in /api/status
   (see showServerMessage), so nothing was lost with the stream. */
function connect() { startPolling(); }

/* ------------------------------------------------------------------ boot -- */

initTabs();
initBanner();
startClock();
initCal();
initWifi();
initConsole();

/* The visibilitychange listener lives in startPolling() and ONLY there. There
   used to be a second, identical one here - startPolling()'s "if (state.poll)
   return" guard stops a duplicate interval but does nothing about duplicate
   listeners, so every return to the tab fired two concurrent status requests at
   a device that can barely afford one.

   And the first request is poll(), not a bare fetch(). The hand-rolled fetch
   that used to be here had no timeout: if the device accepted the connection but
   never answered, the promise never settled, .then(connect) never ran, and
   polling never started - the page sat on "connecting..." for ever with no
   retry. poll() already solved that; the boot path had simply been missed. */
/* And it does not fire immediately. Serving this page is the single most
   expensive thing the device does, and it is not finished when the browser
   finishes parsing: the socket still has to drain and be released. Measured,
   with a phone freshly associated:

     [ap] stations=1 heap=3760 block=3096          <- before the page
     [http] done heap=2928 | low heap=1232 block=448
     [accept] REFUSED heap=864 block=280
     [accept] REFUSED heap=344 block=96            <- 344 bytes free
     [ap] stations=1 heap=736 q=13 REFUSING
     [ap] stations=1 heap=3376 accepting           <- ~2-4 s later, fine again

   Every request in that window is refused, which is exactly what "failed to
   fetch", "no reply from logger" and ERR_CONNECTION_RESET look like from the
   page - and the boot poll, then loadCal() behind it, landed squarely in it.
   The device recovers on its own; it just needs to be left alone to do it.

   The cost is that the header reads "connecting..." for a couple of seconds
   longer on a fresh load. That is a much better trade than a dashboard whose
   first three requests are all refused. */
setTimeout(function () { poll().then(connect); }, 2500);
