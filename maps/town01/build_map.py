from pathlib import Path
import json,html
p=Path(__file__).parent
d=json.loads((p/'map-data.json').read_text())
pts=[v for lane in d['lanes'] for v in lane]
xmin=min(v['x'] for v in pts)-12;xmax=max(v['x'] for v in pts)+12
ymin=min(v['y'] for v in pts)-12;ymax=max(v['y'] for v in pts)+12
W=1000;H=850
scale=min(880/(xmax-xmin),710/(ymax-ymin))
def xy(q):return 65+(q['x']-xmin)*scale,65+(ymax-q['y'])*scale
parts=['<svg xmlns="http://www.w3.org/2000/svg" id="map" viewBox="0 0 1000 850" role="img" aria-label="Town01 ROS coordinate map with reconstructed global route">','<rect width="1000" height="850" fill="#f8fafc"/>']
for x in range(int(xmin//25)*25,int(xmax)+1,25):
 a,b=xy(dict(x=x,y=ymin));_,c=xy(dict(x=x,y=ymax))
 parts.append(f'<path d="M{a},{b}V{c}" stroke="#dce3eb"/><text x="{a}" y="{b+23}" text-anchor="middle" font-size="12">{x}</text>')
for y in range(int(ymin//25)*25,int(ymax)+1,25):
 a,b=xy(dict(x=xmin,y=y));c,_=xy(dict(x=xmax,y=y))
 parts.append(f'<path d="M{a},{b}H{c}" stroke="#dce3eb"/><text x="{a-8}" y="{b+4}" text-anchor="end" font-size="12">{y}</text>')
for lane in d['lanes']:
 coords=' '.join(f'{xy(q)[0]:.2f},{xy(q)[1]:.2f}' for q in lane)
 parts.append(f'<polyline points="{coords}" fill="none" stroke="#a1aab6" stroke-width="{lane[0]["width"]*scale:.2f}" stroke-linecap="round"/>')
route=' '.join(f'{xy(q)[0]:.2f},{xy(q)[1]:.2f}' for q in d['route'])
parts.append(f'<polyline points="{route}" fill="none" stroke="#1760dc" stroke-width="3"/>')
for q in d['route'][::40]:
 a,b=xy(q);parts.append(f'<circle cx="{a}" cy="{b}" r="3" fill="#1760dc"/><text x="{a+6}" y="{b-6}" font-size="12" fill="#164ba7">{q["distance_m"]:.0f} m</text>')
for q in d['vehicles']:
 a,b=xy(q);parts.append(f'<rect x="{a-4}" y="{b-4}" width="8" height="8" fill="#d36a00"/><text x="{a+6}" y="{b+15}" font-size="11" fill="#904500">{html.escape(q["id"])}</text>')
for label,q,color in [('START',d['start'],'#00875a'),('GOAL',d['goal'],'#c02952')]:
 a,b=xy(q);parts.append(f'<circle cx="{a}" cy="{b}" r="7" fill="{color}"/><text x="{a+10}" y="{b-10}" font-size="15" font-weight="bold" fill="{color}">{label}</text>')
parts+=['<circle id="selection" cx="-100" cy="-100" r="7" fill="none" stroke="#121212" stroke-width="2"/>','<text x="500" y="825" text-anchor="middle" font-size="17">ROS x (m) →</text>','<text x="20" y="35" font-size="17">ROS y (m) ↑</text>','</svg>']
svg=''.join(parts)
(p/'town01-coordinate-map.svg').write_text(svg)
template='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Town01 坐标选点地图</title>
<style>body{margin:0;background:#f1f5f9;color:#172033;font:16px system-ui}main{max-width:1450px;margin:auto;padding:22px}h1{font-size:25px;margin:0 0 8px}.layout{display:grid;grid-template-columns:minmax(0,1fr) 330px;gap:20px}.panel{background:white;border-radius:12px;padding:18px}svg{width:100%;height:auto;cursor:crosshair}p{line-height:1.6}label{display:block;margin:14px 0}input,select,textarea,button{font:inherit}input{width:80px}textarea{width:100%;box-sizing:border-box;height:270px;font:13px monospace}button{padding:9px 14px}#status{min-height:55px}small{color:#536176}@media(max-width:850px){.layout{grid-template-columns:1fr}}</style>
<main><h1>Town01 · 障碍车辆坐标选点</h1><p>灰色：可行驶车道　蓝线：默认全局路线重建　橙色：配置中的背景车　绿点：起点　红点：终点</p>
<div class="layout"><div class="panel">SVG</div><div class="panel"><strong>点击地图选择位置</strong>
<label>吸附到 <select id="mode"><option value="road">最近车道中心</option><option value="route">默认全局路线</option></select></label>
<label>沿道路横向偏移 <input id="offset" type="number" step="0.5" value="0"> 米</label>
<small>正值为所选车道行驶方向的左侧，负值为右侧。</small>
<label>车头方向 <select id="direction"><option value="0">沿车道方向</option><option value="180">反向</option><option value="90">左转 90°（横放）</option></select></label>
<p id="status">请选择一个道路位置。</p><textarea id="json" readonly aria-label="车辆出生点 JSON"></textarea>
<button id="copy">复制配置</button><p><small>坐标与 fixed-objects-gui.json 一致。仅生成配置，不会修改仿真或文件。z 使用路面高度 + 0.6 m；首次生成需检查落地与碰撞。车道采样间隔约 1 m，交叉口请核对所选道路和朝向。</small></p>
<p><small>路线由默认起点 (127.4, -195.4) 到 spawn_points[150] 重建；不是运行时 ROS 话题快照。改变起终点后需重新导出。橙色车辆位置来自配置文件，不是实时位置。</small></p>
</div></div></main>
<script>
const D=DATA, params=PARAMS;
const svg=document.getElementById('map'),mode=document.getElementById('mode'),offset=document.getElementById('offset'),direction=document.getElementById('direction');
let selected=null,clickPoint=null;
const lanes=D.lanes.flat();
function choose(){if(!clickPoint)return;const candidates=mode.value==='road'?lanes:D.route;selected=candidates.reduce((a,b)=>Math.hypot(a.x-clickPoint.x,a.y-clickPoint.y)<Math.hypot(b.x-clickPoint.x,b.y-clickPoint.y)?a:b);update();}
function update(){if(!selected)return;let a=selected.yaw*Math.PI/180,off=Number(offset.value)||0;
const x=selected.x-Math.sin(a)*off,y=selected.y+Math.cos(a)*off,yaw=((selected.yaw+Number(direction.value)+180)%360+360)%360-180;
const pose={x:+x.toFixed(3),y:+y.toFixed(3),z:+(selected.z+0.6).toFixed(3),roll:0,pitch:0,yaw:+yaw.toFixed(3)};
document.getElementById('json').value=JSON.stringify({spawn_point:pose},null,2);
document.getElementById('status').textContent='道路 '+selected.road+' / 车道 '+selected.lane+' · x='+pose.x+' m, y='+pose.y+' m, yaw='+pose.yaw+'°';
const mark=document.getElementById('selection');mark.setAttribute('cx',65+(x-params.xmin)*params.scale);mark.setAttribute('cy',65+(params.ymax-y)*params.scale);}
svg.addEventListener('click',e=>{const p=svg.createSVGPoint();p.x=e.clientX;p.y=e.clientY;const q=p.matrixTransform(svg.getScreenCTM().inverse());clickPoint={x:params.xmin+(q.x-65)/params.scale,y:params.ymax-(q.y-65)/params.scale};choose();});
mode.onchange=choose;offset.oninput=update;direction.onchange=update;
document.getElementById('copy').onclick=async()=>{const t=document.getElementById('json');if(!t.value)return;try{await navigator.clipboard.writeText(t.value);document.getElementById('copy').textContent='已复制';}catch(e){t.focus();t.select();document.getElementById('copy').textContent='请按 Ctrl+C 复制';}};
</script></html>'''
template=template.replace('SVG',svg,1).replace('DATA',json.dumps(d,separators=(',',':'))).replace('PARAMS',json.dumps(dict(xmin=xmin,ymax=ymax,scale=scale)))
(p/'town01-map.html').write_text(template)
print('Generated map, interactive selector and coordinate CSVs.')
