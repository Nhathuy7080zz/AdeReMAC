// Web UI v4.3: Giao diện Dashboard 2 cột trực quan, độ tương phản cao, hiển thị khung bao vật thể
// sát biên trên lưới siêu mịn 48x48 (2.304 ô) / 24x24 (576 ô) kèm điều khiển ISP Sharpness & VCM Focus.
// Cổng 80 (trang chính, /det, /status, /ctrl, /capture) và cổng 81 (/stream MJPEG) — 2 server riêng
// để luồng video không chặn các yêu cầu điều khiển và ngược lại.
#include "web_ui.h"
#include "app_config.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "frame_hub.h"
#include "detector.h"
#include "cam_ov5640.h"
#include "fpga_link.h"
#include "fomo_micro.h"
#include <lwip/sockets.h>

RuntimeStats g_stats = {};
static httpd_handle_t s_web = nullptr, s_str = nullptr;

static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>AdaReMAC-EdgeVision v4.3 · Fine-Grid FOMO</title>
<style>
:root{--bg:#0b0e14;--panel:#131822;--card:#1a2130;--bdr:#2a344a;--fg:#f0f4f8;--mut:#94a3b8;--acc:#00e676;--acc2:#00b0ff;--warn:#ffb300;--bad:#ff5252}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:13.5px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
header{background:var(--panel);border-bottom:1px solid var(--bdr);padding:10px 18px;display:flex;gap:10px;align-items:center;justify-content:space-between;flex-wrap:wrap}
.brand{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
h1{font-size:16px;font-weight:700;margin:0;color:#fff;letter-spacing:.2px}
.pill{background:var(--card);border:1px solid var(--bdr);border-radius:99px;padding:3px 10px;font-size:12px;color:var(--mut);font-weight:600}
.pill.ok{background:rgba(0,230,118,.15);border-color:var(--acc);color:var(--acc)}
.pill.info{background:rgba(0,176,255,.15);border-color:var(--acc2);color:#70d6ff}
.hdr-btns{display:flex;gap:8px;align-items:center}
button,.btn{background:var(--card);color:var(--fg);border:1px solid var(--bdr);border-radius:7px;padding:6px 12px;font-size:12.5px;font-weight:600;cursor:pointer;transition:.15s;text-decoration:none;display:inline-flex;align-items:center;gap:5px}
button:hover,.btn:hover{border-color:var(--acc2);background:#222c40}
button.pri{background:rgba(0,230,118,.16);border-color:var(--acc);color:var(--acc)}
button.pri:hover{background:rgba(0,230,118,.28)}
.layout{max-width:1420px;margin:12px auto;padding:0 14px;display:grid;grid-template-columns:1fr 390px;gap:14px;align-items:start}
@media(max-width:980px){.layout{grid-template-columns:1fr}}
.stage-card{background:var(--panel);border:1px solid var(--bdr);border-radius:12px;overflow:hidden;box-shadow:0 8px 24px rgba(0,0,0,.35)}
.stage-bar{padding:8px 14px;background:var(--card);border-bottom:1px solid var(--bdr);display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px;font-size:12px;color:var(--mut)}
#wrap{position:relative;width:100%;background:#05070a;min-height:320px;display:flex;align-items:center;justify-content:center}
#v{width:100%;height:auto;max-height:76vh;object-fit:contain;display:block}
#c{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none}
.side{display:flex;flex-direction:column;gap:12px}
.box{background:var(--panel);border:1px solid var(--bdr);border-radius:12px;padding:12px}
.box-title{font-size:12px;font-weight:700;text-transform:uppercase;letter-spacing:.6px;color:var(--mut);margin:0 0 10px;display:flex;justify-content:space-between;align-items:center}
.mgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.mcard{background:var(--card);border:1px solid var(--bdr);border-radius:8px;padding:8px 10px}
.mk{color:var(--mut);font-size:11px}
.mv{font-size:16px;font-weight:700;margin-top:2px;color:#fff}
#objlist{display:flex;flex-direction:column;gap:6px;max-height:135px;overflow-y:auto}
.obj-item{background:rgba(0,230,118,.1);border:1px solid rgba(0,230,118,.4);border-radius:7px;padding:6px 10px;display:flex;justify-content:space-between;align-items:center;font-size:12.5px}
.obj-empty{color:var(--mut);font-size:12.5px;font-style:italic;padding:4px 0}
.ctl-row{display:flex;flex-direction:column;gap:4px;margin-bottom:10px}
.ctl-row:last-child{margin-bottom:0}
.ctl-lbl{display:flex;justify-content:space-between;font-size:12.5px;color:var(--fg);font-weight:500}
.ctl-val{color:var(--acc2);font-weight:700;font-variant-numeric:tabular-nums}
select,input[type=range]{width:100%;background:var(--card);color:var(--fg);border:1px solid var(--bdr);border-radius:6px;padding:6px 8px;font-size:13px}
input[type=range]{padding:0;height:6px;accent-color:var(--acc)}
.chk-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:6px}
.chk{display:flex;align-items:center;gap:6px;font-size:12.5px;background:var(--card);border:1px solid var(--bdr);padding:6px 9px;border-radius:7px;cursor:pointer}
.chk input{accent-color:var(--acc);width:15px;height:15px;margin:0}
</style></head><body>
<header>
 <div class="brand">
  <h1>AdaReMAC-EdgeVision v4.3</h1>
  <span class="pill info" id="be">Backend: -</span>
  <span class="pill info" id="gr">Lưới: 48×48 (2.304 ô)</span>
  <span class="pill" id="cnt">0 đối tượng</span>
 </div>
 <div class="hdr-btns">
  <button class="pri" id="btnAf" title="Quét tự động cuộn dây VCM tìm điểm nét nhất">🎯 Tự lấy nét (AF)</button>
  <a class="btn" href="/capture" download>📷 Chụp ảnh (.jpg)</a>
 </div>
</header>
<div class="layout">
 <div class="stage-card">
  <div class="stage-bar">
   <span id="resInfo">Đang kết nối luồng MJPEG sắc nét...</span>
   <span>Khung bao sát biên Fine-Grid · Bám vết mượt 60Hz</span>
  </div>
  <div id="wrap"><img id="v" alt="Luồng camera OV5640"><canvas id="c"></canvas></div>
 </div>
 <div class="side">
  <div class="box">
   <div class="box-title"><span>Thông số Thời gian thực</span><span id="th" style="color:var(--acc)">-</span></div>
   <div class="mgrid">
    <div class="mcard"><div class="mk">Luồng Stream</div><div class="mv" id="sf">-</div></div>
    <div class="mcard"><div class="mk">Tốc độ AI</div><div class="mv" id="af">-</div></div>
    <div class="mcard"><div class="mk">Giải mã / Suy luận</div><div class="mv" id="ms">-</div></div>
    <div class="mcard"><div class="mk">FPGA (Chu kỳ)</div><div class="mv" id="fc">-</div></div>
    <div class="mcard"><div class="mk">Nhiệt độ ESP32-S3</div><div class="mv" id="tp">-</div></div>
    <div class="mcard"><div class="mk">Cảm biến OV5640</div><div class="mv" id="cm">-</div></div>
   </div>
  </div>
  <div class="box">
   <div class="box-title"><span>Vật thể đang đóng khung</span><span id="cellInfo">2.304 ô/khung</span></div>
   <div id="objlist"><div class="obj-empty">Chưa phát hiện đối tượng trong khung hình</div></div>
  </div>
  <div class="box">
   <div class="box-title"><span>Nhận dạng Lưới Siêu Mịn & Phần cứng AI</span></div>
   <div class="ctl-row">
    <div class="ctl-lbl"><span>Mật độ ô lưới nhận dạng (Fine-Grid)</span><span class="ctl-val" id="gridTxt">48×48 (2.304 ô)</span></div>
    <select id="grid">
     <option value="48" selected>48×48 Siêu mịn (2.304 ô · Gấp 16× · Bám sát biên nhất)</option>
     <option value="24">24×24 Mịn (576 ô · Gấp 4×)</option>
     <option value="12">12×12 Chuẩn FOMO (144 ô)</option>
    </select>
   </div>
   <div class="ctl-row">
    <div class="ctl-lbl"><span>Ngưỡng phát hiện (Logit / Xác suất)</span><span class="ctl-val" id="thrTxt">5 (60%)</span></div>
    <input id="thr" type="range" min="-30" max="40" step="1" value="5">
   </div>
   <div class="ctl-row">
    <div class="ctl-lbl"><span>Backend suy luận</span><span></span></div>
    <select id="bk">
     <option value="auto">Tự động (Ưu tiên FPGA nếu kết nối)</option>
     <option value="cpu">ESP32-S3 CPU (SIMD esp-nn)</option>
     <option value="fpga">Sipeed Tang Nano 20K (SPI)</option>
     <option value="ei">Edge Impulse Library</option>
    </select>
   </div>
   <div class="ctl-row">
    <div class="ctl-lbl"><span>Chế độ bộ nhân xấp xỉ AdaReMAC</span><span></span></div>
    <select id="fm">
     <option value="2">Mode 2 — Chính xác 100% (Exact)</option>
     <option value="1">Mode 1 — Chọn lọc (D2 ≥ thr)</option>
     <option value="0">Mode 0 — Xấp xỉ công suất thấp (+Bù bias)</option>
    </select>
   </div>
   <div class="chk-grid">
    <label class="chk"><input type="checkbox" id="hm" checked> Hiện ô lưới mịn</label>
    <label class="chk"><input type="checkbox" id="lbl" checked> Hiện nhãn & %</label>
    <label class="chk"><input type="checkbox" id="gridlines"> Kẻ lưới toàn khung</label>
    <label class="chk"><input type="checkbox" id="eco"> Ngủ tiết kiệm nhiệt</label>
   </div>
  </div>
  <div class="box">
   <div class="box-title"><span>Độ nét Quang học OV5640 (ISP & Lấy nét VCM)</span></div>
   <div class="ctl-row">
    <div class="ctl-lbl"><span>Độ phân giải Stream</span><span class="ctl-val" id="fsTxt">SVGA 800×600</span></div>
    <select id="fs">
     <option value="hd">HD (1280×720 · Rõ nét cao)</option>
     <option value="xga">XGA (1024×768 · 4:3 sắc nét)</option>
     <option value="svga" selected>SVGA (800×600 · Cân bằng nét & ≥30fps)</option>
     <option value="vga">VGA (640×480 · Nhẹ & mát)</option>
     <option value="qvga">QVGA (320×240 · Wi-Fi yếu)</option>
    </select>
   </div>
   <div class="ctl-row">
    <div class="ctl-lbl"><span>Độ sắc nét cạnh ISP (CIP Sharpness)</span><span class="ctl-val" id="shTxt">Mức 3 / 6</span></div>
    <input id="sharp" type="range" min="0" max="6" step="1" value="3">
   </div>
   <div class="ctl-row">
    <div class="ctl-lbl"><span>Chất lượng nén JPEG (Số nhỏ = Nét hơn)</span><span class="ctl-val" id="qTxt">Q = 8 (Rất nét)</span></div>
    <input id="q" type="range" min="6" max="20" step="1" value="8">
   </div>
   <div class="ctl-row">
    <div class="ctl-lbl"><span>Tiêu cự thấu kính VCM (Lấy nét tay 0..255)</span><span class="ctl-val" id="vcmTxt">140</span></div>
    <input id="vcm" type="range" min="0" max="255" step="1" value="140">
   </div>
  </div>
 </div>
</div>
<script>
const $=id=>document.getElementById(id),v=$('v'),c=$('c'),g=c.getContext('2d');
v.src=location.protocol+'//'+location.hostname+':81/stream';
let det={n:0,o:[],ls:0.08718,g:48},shown=[];
function ctrl(k,val){return fetch('/ctrl?var='+k+'&val='+val)}
function updThrTxt(t,ls){const p=Math.round(100/(1+Math.exp(-t*(ls||0.08718))));$('thrTxt').textContent=t+' ('+p+'%)';$('th').textContent='Ngưỡng '+p+'%'}
$('bk').onchange=e=>ctrl('backend',e.target.value);
$('fm').onchange=e=>ctrl('fpga_mode',e.target.value);
$('grid').onchange=e=>{const gv=e.target.value;ctrl('grid',gv);$('gridTxt').textContent=gv+'×'+gv+' ('+(gv*gv)+' ô)';$('gr').textContent='Lưới: '+gv+'×'+gv+' ('+(gv*gv)+' ô)';$('cellInfo').textContent=(gv*gv)+' ô/khung'};
$('thr').oninput=e=>updThrTxt(Number(e.target.value),det.ls);
$('thr').onchange=e=>ctrl('thr',e.target.value);
$('fs').onchange=e=>{ctrl('fs',e.target.value);$('fsTxt').textContent=e.target.options[e.target.selectedIndex].text.split('·')[0]};
$('sharp').oninput=e=>$('shTxt').textContent='Mức '+e.target.value+' / 6';
$('sharp').onchange=e=>ctrl('sharp',e.target.value);
$('q').oninput=e=>$('qTxt').textContent='Q = '+e.target.value+(e.target.value<=9?' (Rất nét)':e.target.value<=12?' (Chuẩn)':' (Nhẹ)');
$('q').onchange=e=>ctrl('q',e.target.value);
$('vcm').oninput=e=>$('vcmTxt').textContent=e.target.value;
$('vcm').onchange=e=>ctrl('vcm',e.target.value);
$('eco').onchange=e=>ctrl('eco',e.target.checked?1:0);
$('btnAf').onclick=async()=>{const b=$('btnAf');b.textContent='⏳ Đang lấy nét...';await ctrl('af',1);setTimeout(async()=>{await stat();b.textContent='🎯 Tự lấy nét (AF)'},900)};

async function poll(){
 try{
  const wantHeat=$('hm').checked?1:0;
  const r=await fetch('/det?heat='+wantHeat);
  det=await r.json();
  $('af').textContent=det.aifps.toFixed(1)+' fps';
  $('ms').textContent=det.dec.toFixed(1)+' / '+det.inf.toFixed(1)+' ms';
  $('fc').textContent=det.cyc?det.cyc+' ('+(det.cyc/27000).toFixed(1)+' ms)':'CPU Mode';
  $('be').textContent='Backend: '+det.be;
  const nObj=(det.o||[]).length;
  $('cnt').textContent=nObj+' đối tượng';
  $('cnt').className='pill'+(nObj>0?' ok':'');
  if(nObj===0){
   $('objlist').innerHTML='<div class="obj-empty">Chưa phát hiện đối tượng trong khung hình</div>';
  }else{
   $('objlist').innerHTML=det.o.map(o=>`<div class="obj-item"><strong>#${o.id} · Độ tin cậy ${Math.round(o.s*100)}%</strong><span>${o.c||1} ô mịn · ${Math.round(o.w*100)}×${Math.round(o.h*100)}%</span></div>`).join('');
  }
 }catch(e){}
 setTimeout(poll,55);
}

let firstStat=true;
async function stat(){
 try{
  const s=await (await fetch('/status')).json();
  $('sf').textContent=s.sfps.toFixed(1)+' fps';
  $('tp').textContent=s.temp.toFixed(1)+' °C'+(s.hot?' ⚠':'');
  $('tp').style.color=s.hot?'var(--bad)':'#fff';
  $('cm').textContent=(s.cam.standby?'Nghỉ':'Chạy')+' · '+s.cam.fps.toFixed(0)+' fps';
  if(v.naturalWidth>0)$('resInfo').textContent=`Khung hình thực tế: ${v.naturalWidth}×${v.naturalHeight} px · JPEG Q=${s.q} · ISP Sharpness=${s.cam.sharp} · VCM=${s.cam.vcm}`;
  if(firstStat){
   firstStat=false;
   $('thr').value=s.thr;updThrTxt(s.thr,det.ls);
   $('eco').checked=s.eco;$('fm').value=s.fmode;$('grid').value=s.grid;
   $('sharp').value=s.cam.sharp;$('shTxt').textContent='Mức '+s.cam.sharp+' / 6';
   $('q').value=s.q;$('qTxt').textContent='Q = '+s.q+(s.q<=9?' (Rất nét)':'');
   $('vcm').value=s.cam.vcm;$('vcmTxt').textContent=s.cam.vcm;
   const fsMap={5:'qvga',8:'vga',9:'svga',10:'xga',11:'hd'};
   if(fsMap[s.cam.fs])$('fs').value=fsMap[s.cam.fs];
  }else if(document.activeElement!==$('vcm')){
   $('vcm').value=s.cam.vcm;$('vcmTxt').textContent=s.cam.vcm;
  }
 }catch(e){}
 setTimeout(stat,1000);
}

function draw(){
 const rect=v.getBoundingClientRect();
 const w=rect.width,h=rect.height;
 if(w<10||h<10){requestAnimationFrame(draw);return;}
 const dpr=window.devicePixelRatio||1;
 if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){
  c.width=Math.round(w*dpr);c.height=Math.round(h*dpr);
 }
 g.setTransform(dpr,0,0,dpr,0,0);
 g.clearRect(0,0,w,h);

 const G=det.g||48;
 // 1. Kẻ lưới mịn toàn khung nếu người dùng bật
 if($('gridlines').checked){
  g.strokeStyle='rgba(148,163,184,0.16)';
  g.lineWidth=0.75;
  g.beginPath();
  for(let i=1;i<G;i++){
   const gx=i*w/G,gy=i*h/G;
   g.moveTo(gx,0);g.lineTo(gx,h);
   g.moveTo(0,gy);g.lineTo(w,gy);
  }
  g.stroke();
 }

 // 2. Vẽ các ô lưới mịn (Fine-Grid 24x24 / 48x48) bám theo hình dáng vật thể
 if($('hm').checked&&det.fgrid&&det.fgrid.length===576){
  const cw=w/24,ch=h/24;
  for(let i=0;i<576;i++){
   const pct=det.fgrid[i];
   if(pct<28)continue;
   const gx=(i%24)*cw,gy=Math.floor(i/24)*ch;
   const alpha=Math.min(0.45,(pct/100)*0.42);
   g.fillStyle=`rgba(0,230,118,${alpha.toFixed(2)})`;
   g.fillRect(gx+0.5,gy+0.5,cw-1,ch-1);
   if(G===48){
    // Chia đôi mỗi ô 24x24 thành 2x2 ô con 48x48 để nhìn rõ độ mịn 2.304 ô
    g.strokeStyle='rgba(0,255,136,0.32)';
    g.lineWidth=0.6;
    g.strokeRect(gx+0.5,gy+0.5,cw/2-0.5,ch/2-0.5);
    g.strokeRect(gx+cw/2,gy+0.5,cw/2-0.5,ch/2-0.5);
    g.strokeRect(gx+0.5,gy+ch/2,cw/2-0.5,ch/2-0.5);
    g.strokeRect(gx+cw/2,gy+ch/2,cw/2-0.5,ch/2-0.5);
   }else{
    g.strokeStyle='rgba(0,255,136,0.35)';
    g.lineWidth=0.75;
    g.strokeRect(gx+0.5,gy+0.5,cw-1,ch-1);
   }
  }
 }

 // 3. Nội suy 60Hz và vẽ khung bao tương phản cao bám sát biên vật thể [x0, y0, w, h]
 const o=det.o||[];
 shown=o.map(n=>{
  const p=shown.find(s=>s.id===n.id);
  if(!p)return {...n};
  for(const k of['x','y','x0','y0','w','h'])p[k]+=(n[k]-p[k])*0.45;
  p.s=n.s;p.c=n.c;
  return p;
 });

 for(const b of shown){
  const minCell=1/G;
  const bw=Math.max(b.w,minCell)*w;
  const bh=Math.max(b.h,minCell)*h;
  const x=(typeof b.x0==='number'?b.x0*w:(b.x*w-bw/2));
  const y=(typeof b.y0==='number'?b.y0*h:(b.y*h-bh/2));
  const cx=b.x*w,cy=b.y*h;

  // Nền phủ nhẹ trong khung bao
  g.fillStyle='rgba(0,230,118,0.10)';
  g.fillRect(x,y,bw,bh);

  // Viền đen ngoài chống chói trên nền sáng
  g.strokeStyle='rgba(0,0,0,0.85)';
  g.lineWidth=4.5;
  g.strokeRect(x,y,bw,bh);

  // Viền xanh neon chính
  g.strokeStyle='#00e676';
  g.lineWidth=2.2;
  g.strokeRect(x,y,bw,bh);

  // 4 góc ngắm chữ L (Corner Brackets) dày giúp khóa mắt người nhìn ngay lập tức
  const cl=Math.min(18,Math.max(8,Math.min(bw,bh)*0.25));
  g.strokeStyle='#00ff88';
  g.lineWidth=3.8;
  g.beginPath();
  g.moveTo(x,y+cl);g.lineTo(x,y);g.lineTo(x+cl,y);
  g.moveTo(x+bw-cl,y);g.lineTo(x+bw,y);g.lineTo(x+bw,y+cl);
  g.moveTo(x,y+bh-cl);g.lineTo(x,y+bh);g.lineTo(x+cl,y+bh);
  g.moveTo(x+bw-cl,y+bh);g.lineTo(x+bw,y+bh);g.lineTo(x+bw,y+bh-cl);
  g.stroke();

  // Tâm đối tượng (Crosshair + chấm tâm)
  g.strokeStyle='rgba(0,0,0,0.8)';g.lineWidth=3;
  g.beginPath();g.moveTo(cx-7,cy);g.lineTo(cx+7,cy);g.moveTo(cx,cy-7);g.lineTo(cx,cy+7);g.stroke();
  g.strokeStyle='#00e676';g.lineWidth=1.8;
  g.beginPath();g.moveTo(cx-7,cy);g.lineTo(cx+7,cy);g.moveTo(cx,cy-7);g.lineTo(cx,cy+7);g.stroke();

  // Nhãn thông tin độ tương phản cao trên đỉnh khung
  if($('lbl').checked){
   const txt=`#${b.id} · ${Math.round(b.s*100)}% · ${b.c||1} ô`;
   g.font='bold 12.5px -apple-system,Segoe UI,sans-serif';
   const tw=g.measureText(txt).width+14;
   const th=22;
   let lx=Math.max(2,Math.min(w-tw-2,x));
   let ly=(y>=th+4)?(y-th-2):(y+2);
   g.fillStyle='rgba(11,14,20,0.90)';
   g.fillRect(lx,ly,tw,th);
   g.strokeStyle='#00e676';
   g.lineWidth=1.5;
   g.strokeRect(lx,ly,tw,th);
   g.fillStyle='#00ff88';
   g.fillText(txt,lx+7,ly+15.5);
  }
 }
 requestAnimationFrame(draw);
}
poll();stat();draw();
</script></body></html>)HTML";

static esp_err_t index_h(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static const char *bk_name(Backend b) {
    switch (b) { case BK_CPU: return "ESP32-S3 CPU"; case BK_FPGA: return "Tang Nano 20K"; case BK_EI: return "Edge Impulse"; default: return "Auto"; }
}

static esp_err_t det_h(httpd_req_t *req) {
    static DetResult r;
    static char buf[3584];
    det_get(&r);
    char q[16] = {0};
    bool heat = httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK && strstr(q, "heat=1");
    int n = snprintf(buf, sizeof buf,
                     "{\"seq\":%u,\"be\":\"%s\",\"aifps\":%.2f,\"dec\":%.2f,\"inf\":%.2f,\"cyc\":%u,\"ls\":%.6f,\"g\":%u,\"o\":[",
                     (unsigned)r.frame_seq, bk_name(r.backend_used), r.ai_fps, r.decode_ms, r.infer_ms,
                     (unsigned)r.fpga_cycles, fomo_logit_scale(), (unsigned)(r.fine_grid_res ? r.fine_grid_res : 48));
    for (int i = 0; i < r.n && n < (int)sizeof buf - 160; i++) {
        n += snprintf(buf + n, sizeof buf - n,
                      "%s{\"id\":%u,\"x\":%.3f,\"y\":%.3f,\"x0\":%.3f,\"y0\":%.3f,\"w\":%.3f,\"h\":%.3f,\"s\":%.2f,\"c\":%u}",
                      i ? "," : "", r.obj[i].id, r.obj[i].cx, r.obj[i].cy,
                      r.obj[i].x0, r.obj[i].y0, r.obj[i].w, r.obj[i].h, r.obj[i].score, (unsigned)r.obj[i].cells);
    }
    n += snprintf(buf + n, sizeof buf - n, "]");
    if (heat && r.backend_used != BK_EI) {
        n += snprintf(buf + n, sizeof buf - n, ",\"fgrid\":[");
        for (int i = 0; i < FOMO_VIS_GRID * FOMO_VIS_GRID && n < (int)sizeof buf - 16; i++) {
            n += snprintf(buf + n, sizeof buf - n, "%s%u", i ? "," : "", (unsigned)r.fgrid[i]);
        }
        n += snprintf(buf + n, sizeof buf - n, "]");
    }
    n += snprintf(buf + n, sizeof buf - n, "}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t status_h(httpd_req_t *req) {
    char buf[640];
    CamStatus cs = cam_status();
    int n = snprintf(buf, sizeof buf,
        "{\"sfps\":%.1f,\"clients\":%d,\"temp\":%.1f,\"hot\":%s,\"eco\":%s,\"idle\":%s,\"thr\":%d,\"fmode\":%u,\"grid\":%d,\"q\":%d,"
        "\"fpga\":%s,\"ina_ma\":%.1f,\"ina_v\":%.2f,\"heap\":%u,\"psram\":%u,"
        "\"cam\":{\"standby\":%s,\"fps\":%.1f,\"pid\":%u,\"fs\":%d,\"sharp\":%d,\"vcm\":%d,\"r300e\":%d,\"r302c\":%d,\"vco_mhz\":%u}}",
        g_stats.stream_fps, g_stats.stream_clients, g_stats.chip_temp_c, g_stats.hot ? "true" : "false",
        g_stats.eco_enabled ? "true" : "false", g_stats.idle_mode ? "true" : "false", det_get_threshold(),
        det_get_fpga_mode(), det_get_fine_grid(), g_stats.jpeg_quality,
        fpga_present() ? "true" : "false", g_stats.ina_ma, g_stats.ina_v,
        (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(),
        cs.standby ? "true" : "false", g_stats.cam_fps, cs.pid, (int)cs.framesize, cs.sharpness, cs.vcm_step,
        cs.reg_300e, cs.reg_302c, (unsigned)cs.vco_mhz);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t ctrl_h(httpd_req_t *req) {
    char q[96], var[24], val[24];
    if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK ||
        httpd_query_key_value(q, "var", var, sizeof var) != ESP_OK ||
        httpd_query_key_value(q, "val", val, sizeof val) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "var/val");
        return ESP_FAIL;
    }
    if (!strcmp(var, "backend")) {
        det_set_backend(!strcmp(val, "cpu") ? BK_CPU : !strcmp(val, "fpga") ? BK_FPGA : !strcmp(val, "ei") ? BK_EI : BK_AUTO);
    } else if (!strcmp(var, "thr")) {
        det_set_threshold(atoi(val));
    } else if (!strcmp(var, "grid")) {
        det_set_fine_grid(atoi(val));
    } else if (!strcmp(var, "fpga_mode")) {
        det_set_fpga_mode((uint8_t)atoi(val));
    } else if (!strcmp(var, "eco")) {
        g_stats.eco_enabled = atoi(val) != 0;
    } else if (!strcmp(var, "fs")) {
        framesize_t fs = !strcmp(val, "qvga") ? FRAMESIZE_QVGA :
                         !strcmp(val, "vga")  ? FRAMESIZE_VGA  :
                         !strcmp(val, "xga")  ? FRAMESIZE_XGA  :
                         !strcmp(val, "hd")   ? FRAMESIZE_HD   : FRAMESIZE_SVGA;
        cam_set_framesize(fs);
    } else if (!strcmp(var, "q")) {
        sensor_t *s = esp_camera_sensor_get();
        int qv = atoi(val);
        if (s && qv >= 6 && qv <= 30) { s->set_quality(s, qv); g_stats.jpeg_quality = qv; }
    } else if (!strcmp(var, "sharp")) {
        cam_set_sharpness(atoi(val));
    } else if (!strcmp(var, "vcm")) {
        cam_set_vcm_focus(atoi(val));
    } else if (!strcmp(var, "af")) {
        g_stats.af_request = true;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

// Tải khung JPEG mới nhất — dùng để thu ảnh cảnh thật cho ai_fomo/auto_label.py
static esp_err_t capture_h(httpd_req_t *req) {
    HubFrame f;
    if (!hub_acquire_latest(&f, 0)) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no frame"); return ESP_FAIL; }
    char cd[64];
    snprintf(cd, sizeof cd, "attachment; filename=cap_%lu.jpg", (unsigned long)f.seq);
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", cd);
    esp_err_t r = httpd_resp_send(req, (const char *)f.data, f.len);
    hub_release(&f);
    return r;
}

#define BOUNDARY "adaremacframe"
static esp_err_t stream_h(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req), one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    g_stats.stream_clients = g_stats.stream_clients + 1;
    uint32_t last = 0, cnt = 0, t0 = millis();
    esp_err_t res = ESP_OK;
    char hdr[96];
    while (res == ESP_OK) {
        HubFrame f;
        if (!hub_acquire_latest(&f, last)) { vTaskDelay(1); continue; }
        last = f.seq;
        int hl = snprintf(hdr, sizeof hdr, "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)f.len);
        res = httpd_resp_send_chunk(req, hdr, hl);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)f.data, f.len);
        hub_release(&f);
        cnt++;
        uint32_t now = millis();
        if (now - t0 >= 1000) { g_stats.stream_fps = cnt * 1000.0f / (now - t0); cnt = 0; t0 = now; }
    }
    g_stats.stream_clients = g_stats.stream_clients - 1;
    if (g_stats.stream_clients <= 0) { g_stats.stream_clients = 0; g_stats.stream_fps = 0; }
    return res;
}

void web_start() {
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.server_port = 80; c.ctrl_port = 32768; c.core_id = 1; c.max_uri_handlers = 8;
    if (httpd_start(&s_web, &c) == ESP_OK) {
        httpd_uri_t u[] = {
            {"/", HTTP_GET, index_h, nullptr}, {"/det", HTTP_GET, det_h, nullptr},
            {"/status", HTTP_GET, status_h, nullptr}, {"/ctrl", HTTP_GET, ctrl_h, nullptr},
            {"/capture", HTTP_GET, capture_h, nullptr}};
        for (auto &x : u) httpd_register_uri_handler(s_web, &x);
    } else Serial.println("LOI: khong khoi dong duoc web server cong 80 (thieu SRAM?)");
    httpd_config_t c2 = HTTPD_DEFAULT_CONFIG();
    c2.server_port = 81; c2.ctrl_port = 32769; c2.core_id = 1; c2.max_open_sockets = 3;
    if (httpd_start(&s_str, &c2) == ESP_OK) {
        httpd_uri_t s = {"/stream", HTTP_GET, stream_h, nullptr};
        httpd_register_uri_handler(s_str, &s);
    } else Serial.println("LOI: khong khoi dong duoc stream cong 81 (thieu SRAM?)");
}
