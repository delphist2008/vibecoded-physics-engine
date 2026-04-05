const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const ctx = canvas.getContext('2d')!;

let floorY = window.innerHeight - 80;

function resize() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
  floorY = canvas.height - 80;
}
window.addEventListener('resize', resize);
resize();

// Simple physics
interface Vec { x:number; y:number }

function add(a:Vec,b:Vec):Vec { return {x:a.x+b.x,y:a.y+b.y}; }
function sub(a:Vec,b:Vec):Vec { return {x:a.x-b.x,y:a.y-b.y}; }
function mul(a:Vec,s:number):Vec { return {x:a.x*s,y:a.y*s}; }
function len(a:Vec){return Math.hypot(a.x,a.y);}

// core helper math (moved up so other functions can use them)
function dot(a:Vec,b:Vec){return a.x*b.x + a.y*b.y;}
function cross(a:Vec,b:Vec){return a.x*b.y - a.y*b.x;}
function crossScalarVec(s:number, v:Vec){ return { x: -s * v.y, y: s * v.x }; }
function normalize(v:Vec){ const L = Math.hypot(v.x,v.y) || 1; return {x:v.x/L, y:v.y/L}; }
function rotateAround(p:Vec, c:Vec, ang:number){ const s=Math.sin(ang), co=Math.cos(ang); const rx = p.x - c.x, ry = p.y - c.y; return { x: c.x + rx*co - ry*s, y: c.y + rx*s + ry*co }; }
function centroidOf(pts:Vec[]){let x=0,y=0; for(let p of pts){x+=p.x;y+=p.y;} return {x:x/pts.length,y:y/pts.length}; }

// compute polygon signed area (shoelace)
// point-in-polygon (raycast) used for hover and picking
function polygonArea(pts:Vec[]){
  let a = 0;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++){
    a += (pts[j].x * pts[i].y - pts[i].x * pts[j].y);
  }
  return a/2;
}

function pointInPoly(pt:Vec, poly:Vec[]){
  let c = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++){
    const a = poly[i], b = poly[j];
    if (((a.y > pt.y) !== (b.y > pt.y)) && (pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x)) c = !c;
  }
  return c;
}

// small helper to pick a random HSL color for bodies
function randomColor(){
  const h = Math.floor(Math.random() * 360);
  return `hsl(${h} 60% 60%)`;
}

// Polygon body (rigid)
interface RBody {
  id:number;
  // local space vertices relative to pos
  localVerts: Vec[];
  pos: Vec;         // centroid / position in world coords
  vel: Vec;
  mass: number;
  color: string;
  isStatic?: boolean;
  ang: number;        // angle (rad)
  angVel: number;     // angular velocity (rad/s)
  invMass: number;
  inertia: number;
  invInertia: number;
}

// helper: compute world-space verts from body
function getWorldVerts(b: RBody){
  const out: Vec[] = [];
  const s = Math.sin(b.ang), c = Math.cos(b.ang);
  for (let lv of b.localVerts){
    out.push({ x: b.pos.x + lv.x * c - lv.y * s, y: b.pos.y + lv.x * s + lv.y * c });
  }
  return out;
}

function finalizeBody(b: any){
  // if verts were provided in world coords, convert to local
  if ((b as any).verts){
    const worldVerts: Vec[] = (b as any).verts;
    const centroid = centroidOf(worldVerts);
    b.pos = { x: centroid.x, y: centroid.y };
    b.localVerts = worldVerts.map(v=>({ x: v.x - centroid.x, y: v.y - centroid.y }));
    delete (b as any).verts;
  }
  if (!b.pos) b.pos = {x:0,y:0};
  if (!b.localVerts) b.localVerts = [];
  if (!b.vel) b.vel = {x:0,y:0};
  if (!b.mass || b.mass <= 0) b.mass = 1;
  b.invMass = b.isStatic ? 0 : (1 / b.mass);
  // inertia approx from local verts
  let sum = 0;
  for (let lv of b.localVerts){ sum += lv.x*lv.x + lv.y*lv.y; }
  const meanr2 = (b.localVerts.length>0) ? (sum / b.localVerts.length) : 1;
  b.inertia = b.mass * meanr2;
  b.invInertia = b.isStatic || b.inertia===0 ? 0 : 1 / b.inertia;
  if (typeof b.ang !== 'number') b.ang = 0;
  if (typeof b.angVel !== 'number') b.angVel = 0;
}

// update triangle/polygon helpers to work on given vertex arrays
function projectPoly(poly:Vec[], axis:Vec){
  let min = dot(poly[0], axis), max = min;
  for (let i=1;i<poly.length;i++){ const p = dot(poly[i], axis); if (p<min) min=p; if (p>max) max=p; }
  return {min, max};
}

// overlap helper for two vertex arrays
function overlapOnAxis(polyA:Vec[], polyB:Vec[], axis:Vec){
  const pa = projectPoly(polyA, axis);
  const pb = projectPoly(polyB, axis);
  return Math.min(pa.max, pb.max) - Math.max(pa.min, pb.min);
}

// get normalized edge normals (axes) from a vertex array
function getAxesFromVerts(poly: Vec[]){
  const axes: Vec[] = [];
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++){
    const a = poly[j], b = poly[i];
    const edge = { x: b.x - a.x, y: b.y - a.y };
    const nx = -edge.y, ny = edge.x;
    const L = Math.hypot(nx, ny) || 1;
    axes.push({ x: nx / L, y: ny / L });
  }
  return axes;
}

// SAT collision detection using world verts (updated to support concave polygons via ear-clipping triangulation)
function pointInTriangle(p: Vec, a: Vec, b: Vec, c: Vec){
  // barycentric / sign method
  const v0 = { x: c.x - a.x, y: c.y - a.y };
  const v1 = { x: b.x - a.x, y: b.y - a.y };
  const v2 = { x: p.x - a.x, y: p.y - a.y };
  const dot00 = dot(v0, v0);
  const dot01 = dot(v0, v1);
  const dot02 = dot(v0, v2);
  const dot11 = dot(v1, v1);
  const dot12 = dot(v1, v2);
  const invDenom = 1 / (dot00 * dot11 - dot01 * dot01 || 1e-8);
  const u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  const v = (dot00 * dot12 - dot01 * dot02) * invDenom;
  return u >= 0 && v >= 0 && (u + v) <= 1;
}

function ensureCCW(poly: Vec[]){
  if (polygonArea(poly) < 0) return poly.slice().reverse();
  return poly;
}

function earClipTriangulate(polyIn: Vec[]){
  const poly = ensureCCW(polyIn);
  const n = poly.length;
  if (n < 3) return [] as Vec[][];
  if (n === 3) return [[poly[0], poly[1], poly[2]]];
  const verts = poly.map((p, i)=> ({p, i}));
  const triangles: Vec[][] = [];
  let guard = 0;
  while (verts.length >= 3 && guard++ < 1000){
    let earFound = false;
    for (let i=0;i<verts.length;i++){
      const prev = verts[(i-1+verts.length)%verts.length];
      const curr = verts[i];
      const next = verts[(i+1)%verts.length];
      // check convex
      const ax = curr.p.x - prev.p.x, ay = curr.p.y - prev.p.y;
      const bx = next.p.x - curr.p.x, by = next.p.y - curr.p.y;
      const crossZ = ax*by - ay*bx;
      if (crossZ <= 0) continue; // not an ear (requires CCW winding)
      // check no other point inside triangle
      let anyInside = false;
      for (let k=0;k<verts.length;k++){
        if (k===((i-1+verts.length)%verts.length) || k===i || k===((i+1)%verts.length)) continue;
        if (pointInTriangle(verts[k].p, prev.p, curr.p, next.p)){ anyInside = true; break; }
      }
      if (anyInside) continue;
      // cut the ear
      triangles.push([prev.p, curr.p, next.p]);
      verts.splice(i,1);
      earFound = true;
      break;
    }
    if (!earFound) break; // fallback to avoid infinite loop
  }
  // if left with a triangle, add it
  if (verts.length === 3){ triangles.push([verts[0].p, verts[1].p, verts[2].p]); }
  return triangles;
}

function triangleSAT(aVerts: Vec[], bVerts: Vec[]){
  const axes = [...getAxesFromVerts(aVerts), ...getAxesFromVerts(bVerts)];
  let minOverlap = Infinity; let smallestAxis: Vec | null = null;
  for (let axis of axes){
    const o = overlapOnAxis(aVerts, bVerts, axis);
    if (o <= 0) return null;
    if (o < minOverlap){ minOverlap = o; smallestAxis = axis; }
  }
  if (!smallestAxis) return null;
  // contact point approx: average of closest points
  let minProj = Infinity; let minV = bVerts[0];
  for (let v of bVerts){ const p = dot(v, smallestAxis); if (p < minProj){ minProj = p; minV = v; } }
  let maxProj = -Infinity; let maxV = aVerts[0];
  for (let v of aVerts){ const p = dot(v, smallestAxis); if (p > maxProj){ maxProj = p; maxV = v; } }
  const contact = { x: (minV.x + maxV.x)/2, y: (minV.y + maxV.y)/2 };
  // ensure normal points from A to B
  const dir = { x: bVerts[0].x - aVerts[0].x, y: bVerts[0].y - aVerts[0].y };
  if (dot(dir, smallestAxis) < 0) smallestAxis = { x: -smallestAxis.x, y: -smallestAxis.y };
  return { penetration: minOverlap, normal: smallestAxis, contactPoint: contact };
}

function segIntersect(p1:Vec,p2:Vec,q1:Vec,q2:Vec){
  const r = { x: p2.x - p1.x, y: p2.y - p1.y };
  const s = { x: q2.x - q1.x, y: q2.y - q1.y };
  const rxs = cross(r,s);
  const qpx = { x: q1.x - p1.x, y: q1.y - p1.y };
  const qpxr = cross(qpx, r);
  if (Math.abs(rxs) < 1e-9){ return null; } // parallel
  const t = cross(qpx, s) / rxs;
  const u = qpxr / rxs;
  if (t >= -1e-8 && t <= 1+1e-8 && u >= -1e-8 && u <= 1+1e-8){
    return { x: p1.x + t * r.x, y: p1.y + t * r.y };
  }
  return null;
}

function outwardEdgeNormal(a:Vec,b:Vec){
  // for CCW polygon, outward normal = (edge.y, -edge.x) / len
  const ex = b.x - a.x, ey = b.y - a.y;
  const nx = ey, ny = -ex;
  const L = Math.hypot(nx, ny) || 1;
  return { x: nx / L, y: ny / L };
}

function nearestEdgeInfo(poly:Vec[], p:Vec){
  let best = { dist: Infinity, normal: {x:0,y:0}, a: poly[0], b: poly[1] };
  for (let i=0;i<poly.length;i++){
    const a = poly[i]; const b = poly[(i+1)%poly.length];
    const n = outwardEdgeNormal(a,b);
    const d = dot({x: p.x - a.x, y: p.y - a.y}, n); // positive outside, negative inside
    const absd = Math.abs(d);
    if (absd < best.dist){ best.dist = absd; best.normal = n; best.a = a; best.b = b; }
  }
  return best;
}

function polygonCollideSAT(A: RBody, B: RBody){
  const aVerts = ensureCCW(getWorldVerts(A));
  const bVerts = ensureCCW(getWorldVerts(B));
  const contacts: { point:Vec, normal:Vec, penetration:number }[] = [];

  // edge intersections
  for (let i=0;i<aVerts.length;i++){
    const a1 = aVerts[i], a2 = aVerts[(i+1)%aVerts.length];
    for (let j=0;j<bVerts.length;j++){
      const b1 = bVerts[j], b2 = bVerts[(j+1)%bVerts.length];
      const ip = segIntersect(a1,a2,b1,b2);
      if (ip){
        const na = outwardEdgeNormal(a1,a2);
        const nb = outwardEdgeNormal(b1,b2);
        const n = normalize({ x: (na.x + nb.x)/2, y: (na.y + nb.y)/2 });
        contacts.push({ point: ip, normal: n, penetration: 0 });
      }
    }
  }

  // vertices of A inside B
  for (let v of aVerts){
    if (pointInPoly(v, bVerts)){
      const info = nearestEdgeInfo(bVerts, v);
      const penetration = Math.max(0, -dot({x:v.x - info.a.x, y: v.y - info.a.y}, info.normal));
      contacts.push({ point: v, normal: info.normal, penetration });
    }
  }
  // vertices of B inside A
  for (let v of bVerts){
    if (pointInPoly(v, aVerts)){
      const info = nearestEdgeInfo(aVerts, v);
      const penetration = Math.max(0, -dot({x:v.x - info.a.x, y: v.y - info.a.y}, info.normal));
      contacts.push({ point: v, normal: { x: -info.normal.x, y: -info.normal.y }, penetration });
    }
  }

  if (contacts.length === 0) return null;

  // ensure each contact normal points from A to B (avoid flipped normals on concave "inside" edges)
  for (let c of contacts){
    const toB = { x: B.pos.x - c.point.x, y: B.pos.y - c.point.y };
    if (dot(toB, c.normal) < 0) { c.normal.x = -c.normal.x; c.normal.y = -c.normal.y; }
  }

  // weight contacts: intersections small weight, vertex-penetrations weighted by penetration
  let nSum = { x:0, y:0 }, cpSum = { x:0, y:0 }, wSum = 0;
  for (let c of contacts){
    const w = (c.penetration > 0) ? (1 + c.penetration) : 0.5;
    nSum.x += c.normal.x * w; nSum.y += c.normal.y * w;
    cpSum.x += c.point.x * w; cpSum.y += c.point.y * w;
    wSum += w;
  }
  if (wSum === 0) return null;
  const avgN = normalize({ x: nSum.x / wSum, y: nSum.y / wSum });
  const avgCP = { x: cpSum.x / wSum, y: cpSum.y / wSum };

  // compute penetration conservatively: prefer the smallest positive vertex penetration; if none, measure overlap along avg normal
  let minPen = Infinity; let anyPen = false;
  for (let c of contacts){ if (c.penetration > 0){ anyPen = true; if (c.penetration < minPen) minPen = c.penetration; } }
  let avgPen: number;
  if (anyPen && isFinite(minPen)){
    avgPen = Math.max(0.001, minPen);
  } else {
    // project both polys onto avgN and use overlap (safer than arbitrary default)
    const pa = projectPoly(aVerts, avgN);
    const pb = projectPoly(bVerts, avgN);
    const overlap = Math.min(pa.max, pb.max) - Math.max(pa.min, pb.min);
    avgPen = overlap > 0 ? overlap : 0.01;
  }

  // clamp penetration to avoid huge positional corrections that pull bodies through thin walls
  const MAX_PEN = 10; // pixels
  avgPen = Math.min(avgPen, MAX_PEN);

  // ensure normal points from A to B: check direction from contact point to B
  const toB = { x: B.pos.x - avgCP.x, y: B.pos.y - avgCP.y };
  if (dot(toB, avgN) < 0) { avgN.x = -avgN.x; avgN.y = -avgN.y; }

  // nudge contact point slightly along the normal to keep it on the surface side
  const contactPoint = { x: avgCP.x + avgN.x * 1e-3, y: avgCP.y + avgN.y * 1e-3 };

  return { penetration: avgPen, normal: avgN, contactPoint };
}

function applyImpulse(body: RBody, impulse: Vec, contactPoint: Vec){
  body.vel.x += impulse.x * body.invMass;
  body.vel.y += impulse.y * body.invMass;
  const r = { x: contactPoint.x - body.pos.x, y: contactPoint.y - body.pos.y };
  body.angVel += body.invInertia * cross(r, impulse);
}

// world and input state (single canonical copy)
const bodies: RBody[] = [];
// expose a global reference to help detect shadowing/caching issues at runtime
;(window as any).__bodies = bodies;
let nextId = 1;
const gravity = { x: 0, y: 1000 };
let mouseDown = false;
let ctrlDown = false;
let startPos: Vec | null = null;
let currentPos: Vec | null = null;
let draggingBody: RBody | null = null;
let isDragging = false;
// preview creation state
let previewSides = 6;

// Freeform polygon while Ctrl is held
let tempFreeVerts: Vec[] = [];

// lightweight debug: only log when bodies length changes to avoid flooding the console
let lastBodiesCount = -1;

// mouse/keyboard handlers (creation, freeform, dragging)
canvas.addEventListener('mousedown', (e)=>{
  if (e.button !== 0) return;
  const p = {x:e.clientX, y:e.clientY};
  console.debug('mousedown', p, 'ctrl', ctrlDown);
  mouseDown = true;
  startPos = p;
  currentPos = p;

  if (ctrlDown) {
    tempFreeVerts.push(p);
    return;
  }

  // check click on body (top-most first)
  for (let i = bodies.length - 1; i >= 0; i--) {
    const b = bodies[i];
    const wv = getWorldVerts(b);
    if (pointInPoly(p, wv)){
      draggingBody = b;
      isDragging = true;
      // store anchor point in body-local coords so we always apply impulse at the original clicked point
      dragLocalAnchor = worldToLocal(b, p);
      // prevent new-shape creation by clearing startPos
      startPos = null;
      console.debug('start dragging body', b.id);
      return;
    }
  }
});

canvas.addEventListener('mousemove', (e)=>{ currentPos = {x:e.clientX, y:e.clientY}; });

canvas.addEventListener('mouseup', (e)=>{
  if (e.button !== 0) return;
  console.debug('mouseup', { mouseDown, startPos, currentPos, isDragging, ctrlDown });
  mouseDown = false;
  // if we were dragging an existing body, finalize drag and do not create new shape
  if (isDragging) {
    // apply an impulse based on distance from centroid to cursor so the user "throws" the body
    if (draggingBody && currentPos){
      const b = draggingBody;
      // compute contact point where the user initially grabbed the body
      const contact = dragLocalAnchor ? localToWorld(b, dragLocalAnchor) : currentPos;
      // velocity at contact
      const r = { x: contact.x - b.pos.x, y: contact.y - b.pos.y };
      const velAtContact = { x: b.vel.x + (-b.angVel * r.y), y: b.vel.y + (b.angVel * r.x) };
      const dir = { x: currentPos.x - contact.x, y: currentPos.y - contact.y };
      const dist = Math.hypot(dir.x, dir.y);
      if (dist > 1e-3){
        // desired velocity for the contact point proportional to cursor offset
        const k = 6; // strength factor
        const desiredVel = { x: dir.x * k, y: dir.y * k };
        const impulse = { x: (desiredVel.x - velAtContact.x) * (b.mass), y: (desiredVel.y - velAtContact.y) * (b.mass) };
        // apply at the contact point to induce rotation
        applyImpulse(b, impulse, contact);
        console.debug('applied drag impulse to', b.id, 'impulse=', impulse);
      }
    }
    draggingBody = null;
    isDragging = false;
    dragLocalAnchor = null;
    startPos = null;
    console.debug('end dragging');
    return;
  }
  if (!startPos || !currentPos) return;
  if (draggingBody) { draggingBody = null; startPos = null; return; }

  if (ctrlDown){ startPos = null; return; }

  // create regular polygon using previewSides and angle from cursor
  const center = startPos!;
  const dx = currentPos.x - center.x;
  const dy = currentPos.y - center.y;
  const radius = Math.max(10, Math.hypot(dx, dy));
  const angle = Math.atan2(dy, dx);
  const sides = previewSides;
  const verts: Vec[] = [];
  for (let i=0;i<sides;i++){
    const a = i/sides*2*Math.PI + angle;
    verts.push({x:center.x+Math.cos(a)*radius,y:center.y+Math.sin(a)*radius});
  }
  const temp: any = { id: nextId++, verts, vel:{x:0,y:0}, mass: radius*radius/100, color: randomColor(), isStatic:false, ang:0, angVel:0 };
  finalizeBody(temp);
  bodies.push(temp as RBody);
  console.debug('created body', temp.id, 'verts', temp.localVerts.length);
  // diagnostic: show bodies length and whether the global reference matches
  console.debug('after push: bodies.length=', bodies.length, 'global same?', (window as any).__bodies === bodies);
  startPos = null;
});

// wheel to change number of sides when creating a regular polygon
canvas.addEventListener('wheel', (e)=>{
  if (!mouseDown) return; // only while dragging to create
  if (isDragging) return;
  if (ctrlDown) return;
  e.preventDefault();
  const delta = Math.sign(e.deltaY);
  if (delta < 0) previewSides = Math.min(12, previewSides + 1);
  else previewSides = Math.max(3, previewSides - 1);
});

// track ctrl key globally so we can finalize on release
window.addEventListener('keydown', (e)=>{
  if (e.key === 'Control') {
    if (!ctrlDown) {
      ctrlDown = true;
      tempFreeVerts = [];
    }
  }
});
window.addEventListener('keyup', (e)=>{
  if (e.key === 'Control') {
    ctrlDown = false;
    // finalize freeform polygon when Ctrl released
    if (tempFreeVerts.length >= 3) {
      const verts = tempFreeVerts.slice();
      const area = Math.abs(polygonArea(verts));
      const mass = Math.max(1, area/100);
      const temp: any = { id: nextId++, verts, vel:{x:0,y:0}, mass, color: randomColor(), isStatic:false, ang:0, angVel:0 };
      finalizeBody(temp);
      bodies.push(temp as RBody);
      console.debug('created (freeform) body', temp.id, 'verts', temp.localVerts.length);
      console.debug('after push (freeform): bodies.length=', bodies.length, 'global same?', (window as any).__bodies === bodies);
    }
    tempFreeVerts = [];
  }
});

// handle Escape to cancel creation while mouse held
window.addEventListener('keydown', (e)=>{
  if (e.key === 'Escape'){
    // cancel regular preview creation
    if (mouseDown && startPos && !isDragging){
      startPos = null;
      mouseDown = false;
      console.debug('creation canceled (Escape)');
    }
    // cancel freeform creation
    if (ctrlDown && tempFreeVerts.length>0){
      tempFreeVerts = [];
      console.debug('freeform creation canceled (Escape)');
    }
  }
});

// coordinate helpers: convert between world and body-local coords
function localToWorld(b: RBody, local: Vec){
  const s = Math.sin(b.ang), c = Math.cos(b.ang);
  return { x: b.pos.x + local.x * c - local.y * s, y: b.pos.y + local.x * s + local.y * c };
}
function worldToLocal(b: RBody, p: Vec){
  const dx = p.x - b.pos.x, dy = p.y - b.pos.y;
  const s = Math.sin(-b.ang), c = Math.cos(-b.ang);
  return { x: dx * c - dy * s, y: dx * s + dy * c };
}

// dragging anchor in local coords (so the same material point on the body is used while dragging)
let dragLocalAnchor: Vec | null = null;

// debug drawing helper: draw arrow from a to b
function drawArrow(ctx: CanvasRenderingContext2D, a:Vec, b:Vec, color:string='red'){
  const dx = b.x - a.x, dy = b.y - a.y; const L = Math.hypot(dx,dy) || 1;
  const ux = dx / L, uy = dy / L;
  ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 3; ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
  // head
  const hs = Math.min(12, L*0.2);
  const hx = b.x - ux*hs, hy = b.y - uy*hs;
  const left = { x: hx + -uy * (hs*0.5), y: hy + ux * (hs*0.5) };
  const right = { x: hx + uy * (hs*0.5), y: hy + -ux * (hs*0.5) };
  ctx.beginPath(); ctx.moveTo(b.x,b.y); ctx.lineTo(left.x,left.y); ctx.lineTo(right.x,right.y); ctx.closePath(); ctx.fillStyle = color; ctx.fill();
}

// debug toggles
let debugDraw = false;

// selection: hovered body under mouse
let hoveredBody: RBody | null = null;

// delete and debug key handlers
window.addEventListener('keydown', (e)=>{
  if (e.key === 'x' || e.key === 'X'){
    if (hoveredBody){
      const idx = bodies.indexOf(hoveredBody);
      if (idx !== -1){ bodies.splice(idx,1); console.debug('deleted body', hoveredBody.id); hoveredBody = null; }
    }
  }
  if (e.key === 'd' || e.key === 'D'){
    debugDraw = !debugDraw; console.debug('debugDraw=', debugDraw);
  }
});

// main loop state for FPS/HUD
let last = performance.now();
let fps = 0; let frameCount = 0; let fpsLastTime = last;

function step(ts:number){
  // compute frame dt
  const rawDt = Math.min(0.03, (ts - last) / 1000);
  last = ts;

  // simple adaptive substepping
  const maxSubsteps = 8; // allow more substeps for stability
  const target = 1/240; // smaller target step
   let steps = Math.ceil(rawDt / target);
   steps = Math.max(1, Math.min(maxSubsteps, steps));
   const dt = rawDt / steps;

  for (let s=0;s<steps;s++){
    // integrate bodies
    for (let b of bodies){
      if (b.isStatic) continue;
      b.vel.x += 0 * dt; // no horizontal accel
      b.vel.y += 1000 * dt; // gravity
      b.pos.x += b.vel.x * dt;
      b.pos.y += b.vel.y * dt;
      b.ang += b.angVel * dt;

      // floor contacts per-vertex
      const wv = getWorldVerts(b);
      for (let v of wv){
        const pen = v.y - floorY;
        if (pen <= 0) continue;
        const contact = { x: v.x, y: floorY };
        const n = { x:0, y:-1 };
        const r = { x: contact.x - b.pos.x, y: contact.y - b.pos.y };
        const velAtContact = { x: b.vel.x + (-b.angVel * r.y), y: b.vel.y + (b.angVel * r.x) };
        const velAlongNormal = dot(velAtContact, n);
        // positional correction
        const percent = 0.2, slop = 0.01;
        const corr = Math.max(pen - slop, 0) * percent;
        b.pos.y -= corr;
        if (velAlongNormal < 0){
          const raCrossN = cross(r, n);
          const denom = b.invMass + raCrossN*raCrossN * b.invInertia;
          const e = 0.3;
          const j = denom === 0 ? 0 : -(1+e) * velAlongNormal / denom;
          const impulse = { x: n.x * j, y: n.y * j };
          applyImpulse(b, impulse, contact);
          // friction
          const vt = { x: velAtContact.x - velAlongNormal * n.x, y: velAtContact.y - velAlongNormal * n.y };
          const vtLen = Math.hypot(vt.x, vt.y);
          if (vtLen > 1e-6){
            const t = { x: vt.x / vtLen, y: vt.y / vtLen };
            const raCrossT = cross(r, t);
            const denomT = b.invMass + raCrossT*raCrossT * b.invInertia;
            let jt = denomT === 0 ? 0 : -dot(velAtContact, t) / denomT;
            const mu = 0.4;
            if (Math.abs(jt) > Math.abs(j) * mu) jt = Math.sign(jt) * Math.abs(j) * mu;
            applyImpulse(b, { x: t.x * jt, y: t.y * jt }, contact);
          }
        }
      }
    }

    // continuous dragging impulse: while holding mouse on a body, steer it toward the cursor each substep
    if (isDragging && draggingBody && currentPos){
      const b = draggingBody;
      // compute current world-space contact point for the original grabbed material point
      const contact = dragLocalAnchor ? localToWorld(b, dragLocalAnchor) : currentPos;
      const r = { x: contact.x - b.pos.x, y: contact.y - b.pos.y };
      const velAtContact = { x: b.vel.x + (-b.angVel * r.y), y: b.vel.y + (b.angVel * r.x) };
      // desired velocity for the contact point so it follows the cursor
      const dir = { x: currentPos.x - contact.x, y: currentPos.y - contact.y };
      const desiredVel = { x: dir.x * 6, y: dir.y * 6 };
      const dv = { x: desiredVel.x - velAtContact.x, y: desiredVel.y - velAtContact.y };
      const alpha = 0.15; // fraction of velocity error converted into an impulse per substep
      const impulse = { x: dv.x * b.mass * alpha, y: dv.y * b.mass * alpha };
      applyImpulse(b, impulse, contact);
    }

    // body-body collisions: iterative solver (several passes to better resolve manifolds)
    const restitution = 0.2;
    const solverIterations = 4;
    for (let iter=0; iter<solverIterations; iter++){
      for (let i=0;i<bodies.length;i++){
        for (let k=i+1;k<bodies.length;k++){
          const A = bodies[i], B = bodies[k];
          if (A===B) continue;
          const info = polygonCollideSAT(A,B);
          if (!info) continue;
          const n = info.normal; const pen = info.penetration; const p = info.contactPoint;
          // stronger positional correction on first iteration
          const percent = (iter === 0) ? 0.5 : 0.2; const slop = 0.01;
          const invMassSum = A.invMass + B.invMass;
          if (invMassSum > 0){
            const corrMag = Math.max(pen - slop, 0) / invMassSum * percent;
            const corr = { x: n.x * corrMag, y: n.y * corrMag };
            if (!A.isStatic){ A.pos.x -= corr.x * A.invMass; A.pos.y -= corr.y * A.invMass; }
            if (!B.isStatic){ B.pos.x += corr.x * B.invMass; B.pos.y += corr.y * B.invMass; }
          }
          // compute relative velocity at contact
          const rA = { x: p.x - A.pos.x, y: p.y - A.pos.y };
          const rB = { x: p.x - B.pos.x, y: p.y - B.pos.y };
          const vA = { x: A.vel.x + (-A.angVel * rA.y), y: A.vel.y + (A.angVel * rA.x) };
          const vB = { x: B.vel.x + (-B.angVel * rB.y), y: B.vel.y + (B.angVel * rB.x) };
          const rv = { x: vB.x - vA.x, y: vB.y - vA.y };
          const velAlongNormal = dot(rv, n);
          if (velAlongNormal > 0) continue;
          const raCrossN = cross(rA, n);
          const rbCrossN = cross(rB, n);
          const denom2 = A.invMass + B.invMass + raCrossN*raCrossN * A.invInertia + rbCrossN*rbCrossN * B.invInertia;
          const j2 = denom2 === 0 ? 0 : -(1 + restitution) * velAlongNormal / denom2;
          const impulse2 = { x: n.x * j2, y: n.y * j2 };
          if (!A.isStatic) applyImpulse(A, { x: -impulse2.x, y: -impulse2.y }, p);
          if (!B.isStatic) applyImpulse(B, impulse2, p);
        }
      }
    }
  }

  // render
  ctx.clearRect(0,0,canvas.width,canvas.height);
  // background
  ctx.fillStyle = '#aee1ff'; ctx.fillRect(0,0,canvas.width,canvas.height);
  // floor
  ctx.fillStyle = '#6aa84f'; ctx.fillRect(0,floorY,canvas.width,canvas.height-floorY);

  // draw bodies
  const mousePos = currentPos;

  // compute hovered body
  hoveredBody = null;
  if (mousePos){
    for (let i=bodies.length-1;i>=0;i--){ const b=bodies[i]; const wv=getWorldVerts(b); if (pointInPoly(mousePos, wv)){ hoveredBody = b; break;} }
  }

  // diagnostic: only log when the bodies count actually changes
  if (bodies.length !== lastBodiesCount){
    console.debug('render sees bodies.length=', bodies.length, 'global same?', (window as any).__bodies === bodies);
    lastBodiesCount = bodies.length;
  }

  for (let b of bodies){
    const wv = getWorldVerts(b);
    ctx.beginPath();
    for (let i=0;i<wv.length;i++){ const v = wv[i]; if (i===0) ctx.moveTo(v.x,v.y); else ctx.lineTo(v.x,v.y); }
    ctx.closePath();
    ctx.fillStyle = b.color;
    ctx.fill();
    ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(0,0,0,0.2)'; ctx.stroke();
    // hover highlight
    if (mousePos && pointInPoly(mousePos, wv)){
      ctx.lineWidth = 4; ctx.strokeStyle = 'yellow'; ctx.stroke();
    }
    // debug draw: show drag impulse vector when this body is being dragged
    if (debugDraw && isDragging && draggingBody === b && currentPos){
      // compute the impulse that would be applied (same math as in loop)
      const contact = dragLocalAnchor && draggingBody === b ? localToWorld(b, dragLocalAnchor) : { x: b.pos.x, y: b.pos.y };
      const r = { x: contact.x - b.pos.x, y: contact.y - b.pos.y };
      const velAtContact = { x: b.vel.x + (-b.angVel * r.y), y: b.vel.y + (b.angVel * r.x) };
      const dir = { x: currentPos.x - contact.x, y: currentPos.y - contact.y };
      const desiredVel = { x: dir.x * 6, y: dir.y * 6 };
      const dv = { x: desiredVel.x - velAtContact.x, y: desiredVel.y - velAtContact.y };
      const alpha = 0.15;
      const impulse = { x: dv.x * b.mass * alpha, y: dv.y * b.mass * alpha };
      drawArrow(ctx, contact, { x: contact.x + impulse.x, y: contact.y + impulse.y }, 'red');
    }

    // debug: triangulate and draw each body's world-space triangulation when debugDraw is enabled
    if (debugDraw){
      const wv = getWorldVerts(b);
      if (wv.length >= 3){
        const tris = earClipTriangulate(wv);
        ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(255,0,0,0.6)'; ctx.fillStyle = 'rgba(255,0,0,0.06)';
        for (let t of tris){ ctx.beginPath(); ctx.moveTo(t[0].x,t[0].y); ctx.lineTo(t[1].x,t[1].y); ctx.lineTo(t[2].x,t[2].y); ctx.closePath(); ctx.fill(); ctx.stroke(); }
      }
    }
  }

  // regular preview when creating
  if (mouseDown && startPos && currentPos && !ctrlDown){
    const center = startPos; const dx = currentPos.x - startPos.x; const dy = currentPos.y - startPos.y;
    const radius = Math.max(10, Math.hypot(dx, dy)); const angle = Math.atan2(dy, dx); const sides = previewSides;
    ctx.beginPath();
    for (let i=0;i<sides;i++){ const a = i/sides*2*Math.PI + angle; const x = center.x + Math.cos(a)*radius; const y = center.y + Math.sin(a)*radius; if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }
    ctx.closePath(); ctx.fillStyle='rgba(255,255,255,0.5)'; ctx.fill(); ctx.strokeStyle='black'; ctx.stroke();
  }

  // freeform preview
  if (ctrlDown && (tempFreeVerts.length>0 || mousePos)){
    ctx.beginPath();
    for (let i=0;i<tempFreeVerts.length;i++){ const v = tempFreeVerts[i]; if (i===0) ctx.moveTo(v.x,v.y); else ctx.lineTo(v.x,v.y); }
    if (mousePos && tempFreeVerts.length>0) ctx.lineTo(mousePos.x, mousePos.y);
    ctx.strokeStyle = 'rgba(0,0,0,0.8)'; ctx.lineWidth=2; ctx.stroke();
    for (let v of tempFreeVerts){ ctx.beginPath(); ctx.fillStyle='white'; ctx.arc(v.x,v.y,4,0,Math.PI*2); ctx.fill(); ctx.strokeStyle='black'; ctx.stroke(); }

    // debug: triangulate and draw the temporary freeform polygon decomposition
    if (debugDraw && tempFreeVerts.length >= 3){
      const tris = earClipTriangulate(tempFreeVerts);
      ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(255,0,0,0.8)'; ctx.fillStyle = 'rgba(255,0,0,0.08)';
      for (let t of tris){ ctx.beginPath(); ctx.moveTo(t[0].x,t[0].y); ctx.lineTo(t[1].x,t[1].y); ctx.lineTo(t[2].x,t[2].y); ctx.closePath(); ctx.fill(); ctx.stroke(); }
    }
  }

  // HUD
  ctx.fillStyle = 'black'; ctx.font='16px Arial'; ctx.textAlign='left'; ctx.textBaseline='top';
  ctx.fillText(`FPS: ${fps}`, 10, 10);
  ctx.fillText(`Bodies: ${bodies.length}`, 10, 30);
  ctx.fillText(`Mode: ${ctrlDown ? 'Freeform' : 'Regular'}`, 10, 50);
  ctx.fillText(`Preview Sides: ${previewSides}`, 10, 70);

  // debug draw: normals and penetration
  if (debugDraw && hoveredBody){
    const wv = getWorldVerts(hoveredBody);
    ctx.strokeStyle = 'red'; ctx.lineWidth = 2;
    for (let v of wv){
      const info = nearestEdgeInfo(wv, v);
      const normalEnd = { x: v.x + info.normal.x * 20, y: v.y + info.normal.y * 20 };
      drawArrow(ctx, v, normalEnd, 'red');
      ctx.fillStyle = 'red'; ctx.beginPath(); ctx.arc(v.x, v.y, 4, 0, Math.PI*2); ctx.fill();
    }
  }

  // FPS measurement
  frameCount++;
  if (ts - fpsLastTime >= 500){ fps = Math.round(frameCount * 1000 / (ts - fpsLastTime)); frameCount = 0; fpsLastTime = ts; }

  // schedule next frame
  requestAnimationFrame(step);
}

requestAnimationFrame(step);
