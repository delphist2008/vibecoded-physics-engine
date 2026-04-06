import { Vec, dot, cross, normalize, projectPoly, getAxesFromVerts, ensureCCW, pointInPoly, centroidOf } from './math';
import { CONFIG } from './settings';

export interface RBody {
  id:number;
  localVerts: Vec[];
  pos: Vec;
  vel: Vec;
  mass: number;
  color: string;
  isStatic?: boolean;
  ang: number;
  angVel: number;
  invMass: number;
  inertia: number;
  invInertia: number;
}

export function getWorldVerts(b: RBody){
  const out: Vec[] = [];
  const s = Math.sin(b.ang), c = Math.cos(b.ang);
  for (let lv of b.localVerts){ out.push({ x: b.pos.x + lv.x * c - lv.y * s, y: b.pos.y + lv.x * s + lv.y * c }); }
  return out;
}

export function finalizeBody(b: any){
  if ((b as any).verts){ const worldVerts: Vec[] = (b as any).verts; const centroid = centroidOf(worldVerts); b.pos = { x: centroid.x, y: centroid.y }; b.localVerts = worldVerts.map(v=>({ x: v.x - centroid.x, y: v.y - centroid.y })); delete (b as any).verts; }
  if (!b.pos) b.pos = {x:0,y:0}; if (!b.localVerts) b.localVerts = []; if (!b.vel) b.vel = {x:0,y:0}; if (!b.mass || b.mass <= 0) b.mass = 1; b.invMass = b.isStatic ? 0 : (1 / b.mass);
  let sum = 0; for (let lv of b.localVerts){ sum += lv.x*lv.x + lv.y*lv.y; }
  const meanr2 = (b.localVerts.length>0) ? (sum / b.localVerts.length) : 1; b.inertia = b.mass * meanr2; b.invInertia = b.isStatic || b.inertia===0 ? 0 : 1 / b.inertia;
  if (typeof b.ang !== 'number') b.ang = 0; if (typeof b.angVel !== 'number') b.angVel = 0;
}

// reuse many helper functions from original file
function segIntersect(p1:Vec,p2:Vec,q1:Vec,q2:Vec){ const r = { x: p2.x - p1.x, y: p2.y - p1.y }; const s = { x: q2.x - q1.x, y: q2.y - q1.y }; const rxs = cross(r,s); const qpx = { x: q1.x - p1.x, y: q1.y - p1.y }; const qpxr = cross(qpx, r); if (Math.abs(rxs) < 1e-9){ return null; } const t = cross(qpx, s) / rxs; const u = qpxr / rxs; if (t >= -1e-8 && t <= 1+1e-8 && u >= -1e-8 && u <= 1+1e-8){ return { x: p1.x + t * r.x, y: p1.y + t * r.y }; } return null; }
function outwardEdgeNormal(a:Vec,b:Vec){ const ex = b.x - a.x, ey = b.y - a.y; const nx = ey, ny = -ex; const L = Math.hypot(nx, ny) || 1; return { x: nx / L, y: ny / L }; }

function nearestEdgeInfo(poly:Vec[], p:Vec){ let best = { dist: Infinity, normal: {x:0,y:0}, a: poly[0], b: poly[1] }; for (let i=0;i<poly.length;i++){ const a = poly[i]; const b = poly[(i+1)%poly.length]; const n = outwardEdgeNormal(a,b); const d = dot({x: p.x - a.x, y: p.y - a.y}, n); const absd = Math.abs(d); if (absd < best.dist){ best.dist = absd; best.normal = n; best.a = a; best.b = b; } } return best; }

export function polygonCollideSAT(A: RBody, B: RBody){
  const aVerts = ensureCCW(getWorldVerts(A));
  const bVerts = ensureCCW(getWorldVerts(B));
  const contacts: { point:Vec, normal:Vec, penetration:number }[] = [];
  for (let i=0;i<aVerts.length;i++){ const a1 = aVerts[i], a2 = aVerts[(i+1)%aVerts.length]; for (let j=0;j<bVerts.length;j++){ const b1 = bVerts[j], b2 = bVerts[(j+1)%bVerts.length]; const ip = segIntersect(a1,a2,b1,b2); if (ip){ const na = outwardEdgeNormal(a1,a2); const nb = outwardEdgeNormal(b1,b2); const n = normalize({ x: (na.x + nb.x)/2, y: (na.y + nb.y)/2 }); contacts.push({ point: ip, normal: n, penetration: 0 }); } } }
  for (let v of aVerts){ if (pointInPoly(v, bVerts)){ const info = nearestEdgeInfo(bVerts, v); const penetration = Math.max(0, -dot({x:v.x - info.a.x, y: v.y - info.a.y}, info.normal)); contacts.push({ point: v, normal: info.normal, penetration }); } }
  for (let v of bVerts){ if (pointInPoly(v, aVerts)){ const info = nearestEdgeInfo(aVerts, v); const penetration = Math.max(0, -dot({x:v.x - info.a.x, y: v.y - info.a.y}, info.normal)); contacts.push({ point: v, normal: { x: -info.normal.x, y: -info.normal.y }, penetration }); } }
  if (contacts.length === 0) return null;
  for (let c of contacts){ const toB = { x: B.pos.x - c.point.x, y: B.pos.y - c.point.y }; if (dot(toB, c.normal) < 0) { c.normal.x = -c.normal.x; c.normal.y = -c.normal.y; } }
  let nSum = { x:0, y:0 }, cpSum = { x:0, y:0 }, wSum = 0;
  for (let c of contacts){ const w = (c.penetration > 0) ? (1 + c.penetration) : 0.5; nSum.x += c.normal.x * w; nSum.y += c.normal.y * w; cpSum.x += c.point.x * w; cpSum.y += c.point.y * w; wSum += w; }
  if (wSum === 0) return null; const avgN = normalize({ x: nSum.x / wSum, y: nSum.y / wSum }); const avgCP = { x: cpSum.x / wSum, y: cpSum.y / wSum };
  let minPen = Infinity; let anyPen = false; for (let c of contacts){ if (c.penetration > 0){ anyPen = true; if (c.penetration < minPen) minPen = c.penetration; } }
  let avgPen: number; if (anyPen && isFinite(minPen)){ avgPen = Math.max(0.001, minPen); } else { const pa = projectPoly(aVerts, avgN); const pb = projectPoly(bVerts, avgN); const overlap = Math.min(pa.max, pb.max) - Math.max(pa.min, pb.min); avgPen = overlap > 0 ? overlap : 0.01; }
  const MAX_PEN = CONFIG.maxPen; avgPen = Math.min(avgPen, MAX_PEN);
  const toB = { x: B.pos.x - avgCP.x, y: B.pos.y - avgCP.y }; if (dot(toB, avgN) < 0) { avgN.x = -avgN.x; avgN.y = -avgN.y; }
  const contactPoint = { x: avgCP.x + avgN.x * 1e-3, y: avgCP.y + avgN.y * 1e-3 };
  return { penetration: avgPen, normal: avgN, contactPoint };
}

export function applyImpulse(body: RBody, impulse: Vec, contactPoint: Vec){ body.vel.x += impulse.x * body.invMass; body.vel.y += impulse.y * body.invMass; const r = { x: contactPoint.x - body.pos.x, y: contactPoint.y - body.pos.y }; body.angVel += body.invInertia * cross(r, impulse); }
