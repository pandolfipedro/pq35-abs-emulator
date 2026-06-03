#pragma once

#include <pgmspace.h>

static const char kPortalCss[] PROGMEM = R"raw(
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);color:#e8e8f0;min-height:100vh;padding:24px}
.wrap{max-width:560px;margin:0 auto}
.card{background:rgba(255,255,255,.08);backdrop-filter:blur(12px);border:1px solid rgba(255,255,255,.12);border-radius:16px;padding:24px;margin-bottom:16px}
h1{font-size:1.4rem;margin-bottom:8px}
h2{font-size:1.05rem;margin:20px 0 10px;color:#b8b8d0}
p,li{font-size:.92rem;line-height:1.5;color:#c8c8dc;margin-bottom:8px}
.meta{display:grid;gap:8px;margin:16px 0}
.row{display:flex;justify-content:space-between;padding:8px 12px;background:rgba(0,0,0,.2);border-radius:8px;font-size:.88rem}
.row span:last-child{color:#fff;font-weight:600}
.row-live{border-left:3px solid #667eea}
.nav{display:flex;gap:10px;flex-wrap:wrap;margin-top:16px}
a.btn,button.btn{display:inline-block;padding:10px 18px;border-radius:8px;text-decoration:none;font-size:.9rem;border:none;cursor:pointer;font-weight:600}
.btn-primary{background:linear-gradient(135deg,#667eea,#764ba2);color:#fff}
.btn-secondary{background:rgba(255,255,255,.12);color:#e8e8f0}
label{display:block;font-size:.85rem;color:#a8a8c0;margin:12px 0 4px}
input[type=text],input[type=password],input[type=file]{width:100%;padding:10px 12px;border-radius:8px;border:1px solid rgba(255,255,255,.15);background:rgba(0,0,0,.25);color:#fff;font-size:.9rem}
input:focus{outline:none;border-color:#667eea}
.msg{padding:10px 14px;border-radius:8px;margin-bottom:12px;font-size:.88rem}
.msg-ok{background:rgba(46,204,113,.2);border:1px solid rgba(46,204,113,.4)}
.msg-err{background:rgba(231,76,60,.2);border:1px solid rgba(231,76,60,.4)}
.warn{color:#f39c12;font-size:.85rem;margin-top:8px}
.live-hint{font-size:.78rem;color:#8888a8;margin-top:8px}
.live-stale{color:#e67e22}
#progress{width:100%;height:8px;background:rgba(0,0,0,.3);border-radius:4px;margin:12px 0;overflow:hidden;display:none}
#bar{height:100%;width:0;background:linear-gradient(90deg,#667eea,#764ba2);transition:width .2s}
#status{font-size:.88rem;color:#a8a8c0;min-height:1.2em}
)raw";

static const char kPageHomeHead[] PROGMEM = R"raw(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PQ35 Config</title><style>
)raw";

static const char kPageHomeTail[] PROGMEM = R"raw(
</style></head><body><div class="wrap"><div class="card">
<h1>PQ35 ABS Emulator</h1><p>Portal — <strong>%VARIANT%</strong> build</p>
<h2>System</h2>
<div class="meta">
<div class="row"><span>Version</span><span id="v-sys">%VERSION%</span></div>
<div class="row"><span>Uptime</span><span id="v-uptime">%UPTIME%</span></div>
<div class="row"><span>Free / min heap</span><span id="v-heap">%HEAP%</span></div>
<div class="row"><span>Last reset</span><span id="v-reset">%RESET%</span></div>
<div class="row"><span>Wi-Fi AP</span><span>PQ35-Config (hidden)</span></div>
<div class="row"><span>MCP2515</span><span id="v-mcp">—</span></div>
</div>
<h2>Vehicle (live)</h2>
<div class="meta">
<div class="row row-live"><span>Filtered speed</span><span id="v-spd">—</span></div>
<div class="row row-live"><span>Cluster speed (CAN)</span><span id="v-panel">—</span></div>
<div class="row row-live"><span>Gear (wahl / eng)</span><span id="v-gear">—</span></div>
<div class="row row-live"><span>OBD speed</span><span id="v-obd">—</span></div>
<div class="row row-live"><span>Park / motion</span><span id="v-motion">—</span></div>
<div class="row row-live"><span>Odometer impulses</span><span id="v-odo">—</span></div>
<div class="row row-live"><span>CAN RX buffer</span><span id="v-rx">—</span></div>
<div class="row row-live"><span>TX failures</span><span id="v-txf">—</span></div>
</div>
<p id="live-hint" class="live-hint">Updating...</p>
<div class="nav">
<a class="btn btn-primary" href="/update">Update Firmware</a>
<a class="btn btn-secondary" href="/settings">Settings</a>
</div></div></div>
<script>
function fmtUptime(s){
  var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;
  return (h<10?'0':'')+h+'h '+(m<10?'0':'')+m+'m '+(sec<10?'0':'')+sec+'s';
}
function refresh(){
  fetch('/api/status').then(function(r){return r.json();}).then(function(d){
    document.getElementById('v-uptime').textContent=fmtUptime(d.uptime_s||0);
    document.getElementById('v-heap').textContent=(d.heap_free||0)+' / '+(d.heap_min||0);
    document.getElementById('v-mcp').textContent=d.mcp_ok?'OK':'FAIL';
    document.getElementById('v-spd').textContent=(d.speed_filt||0).toFixed(1)+' km/h';
    document.getElementById('v-panel').textContent=(d.speed_panel||0).toFixed(1)+' km/h';
    document.getElementById('v-gear').textContent=(d.gear_wahl||'?')+' / '+(d.gear_engaged!=null?d.gear_engaged:'?');
    document.getElementById('v-obd').textContent=d.obd_valid?'valid':'waiting';
    document.getElementById('v-motion').textContent=(d.in_park?'Park':'Drive')+' / '+(d.motion_driving?'moving':'stopped');
    document.getElementById('v-odo').textContent=(d.odo_impulses||0)+' ('+(d.imp_per_km||0).toFixed(0)+' imp/km)';
    document.getElementById('v-rx').textContent=d.rx_buf!=null?d.rx_buf:'—';
    document.getElementById('v-txf').textContent='1A0:'+(d.tx_fail_1a0||0)+' 4A0:'+(d.tx_fail_4a0||0)+' 5A0:'+(d.tx_fail_5a0||0);
    var h=document.getElementById('live-hint');
    if((d.status_age_ms||0)>2500){h.className='live-hint live-stale';h.textContent='Stale CAN data ('+d.status_age_ms+' ms)';}
    else{h.className='live-hint';h.textContent='Updates every 1 s · age: '+(d.status_age_ms||0)+' ms';}
  }).catch(function(){document.getElementById('live-hint').textContent='Failed to read /api/status';});
}
refresh();setInterval(refresh,1000);
</script>
</body></html>
)raw";

static const char kPageSettings[] PROGMEM = R"raw(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Settings</title><style>
)raw";

static const char kPageSettingsBody[] PROGMEM = R"raw(
</style></head><body><div class="wrap"><div class="card">
<h1>Settings</h1>
%MSG%
<form method="POST" action="/settings">
<h2>Web login</h2>
<label for="web_user">Web username</label>
<input type="text" id="web_user" name="web_user" value="%WEB_USER%" required>
<label for="web_pass_new">New web password (min. 6 chars, blank = keep)</label>
<input type="password" id="web_pass_new" name="web_pass_new" autocomplete="new-password">
<label for="web_pass_confirm">Confirm new web password</label>
<input type="password" id="web_pass_confirm" name="web_pass_confirm" autocomplete="new-password">
<h2>Wi-Fi AP</h2>
<p>Fixed SSID: <strong>PQ35-Config</strong> (hidden network)</p>
<label for="wifi_pass_new">New Wi-Fi password (min. 6 chars, blank = keep)</label>
<input type="password" id="wifi_pass_new" name="wifi_pass_new" autocomplete="new-password">
<label for="wifi_pass_confirm">Confirm new Wi-Fi password</label>
<input type="password" id="wifi_pass_confirm" name="wifi_pass_confirm" autocomplete="new-password">
<h2>Confirmation</h2>
<label for="current_web_pass">Current web password</label>
<input type="password" id="current_web_pass" name="current_web_pass" required autocomplete="current-password">
<div class="nav">
<button type="submit" class="btn btn-primary">Save</button>
<a class="btn btn-secondary" href="/">Back</a>
</div></form></div></div></body></html>
)raw";

static const char kPageUpdate[] PROGMEM = R"raw(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Update Firmware</title><style>
)raw";

static const char kPageUpdateBody[] PROGMEM = R"raw(
</style></head><body><div class="wrap"><div class="card">
<h1>Update Firmware (local OTA)</h1>
<p>Upload the correct <strong>.bin</strong> for your build (<strong>lite</strong> or <strong>full</strong>).</p>
<p class="warn">Do not power off the ESP32 during upload. The device will reboot when done.</p>
<label for="firmware">.bin file</label>
<input type="file" id="firmware" accept=".bin,application/octet-stream">
<div id="progress"><div id="bar"></div></div>
<p id="status"></p>
<div class="nav">
<button type="button" class="btn btn-primary" id="btn-upload">Upload</button>
<a class="btn btn-secondary" href="/">Back</a>
</div></div></div>
<script>
document.getElementById('btn-upload').onclick=function(){
  var f=document.getElementById('firmware').files[0];
  if(!f){document.getElementById('status').textContent='Select a .bin file';return;}
  var fd=new FormData();fd.append('firmware',f);
  var xhr=new XMLHttpRequest();var bar=document.getElementById('bar');var prog=document.getElementById('progress');
  prog.style.display='block';
  xhr.upload.onprogress=function(e){if(e.lengthComputable)bar.style.width=(100*e.loaded/e.total)+'%';};
  xhr.onload=function(){
    if(xhr.status===200){document.getElementById('status').textContent='Success! Rebooting...';bar.style.width='100%';}
    else{document.getElementById('status').textContent='Failed: '+xhr.responseText;}
  };
  xhr.open('POST','/update');xhr.send(fd);
};
</script>
</body></html>
)raw";
