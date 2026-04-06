export interface Vec { x:number; y:number }
export function add(a:Vec,b:Vec):Vec { return {x:a.x+b.x,y:a.y+b.y}; }
export function sub(a:Vec,b:Vec):Vec { return {x:a.x-b.x,y:a.y-b.y}; }
export function mul(a:Vec,s:number):Vec { return {x:a.x*s,y:a.y*s}; }
export function len(a:Vec){return Math.hypot(a.x,a.y);}
export function dot(a:Vec,b:Vec){return a.x*b.x + a.y*b.y;}
export function cross(a:Vec,b:Vec){return a.x*b.y - a.y*b.x;}
export function crossScalarVec(s:number, v:Vec){ return { x: -s * v.y, y: s * v.x }; }
export function normalize(v:Vec){ const L = Math.hypot(v.x,v.y) || 1; return {x:v.x/L, y:v.y/L}; }
export function rotateAround(p:Vec, c:Vec, ang:number){ const s=Math.sin(ang), co=Math.cos(ang); const rx = p.x - c.x, ry = p.y - c.y; return { x: c.x + rx*co - ry*s, y: c.y + rx*s + ry*co }; }
export function centroidOf(pts:Vec[]){let x=0,y=0; for(let p of pts){x+=p.x;y+=p.y;} return {x:x/pts.length,y:y/pts.length}; }
export function polygonArea(pts:Vec[]){ let a=0; for(let i=0,j=pts.length-1;i<pts.length;j=i++){ a += (pts[j].x * pts[i].y - pts[i].x * pts[j].y);} return a/2; }
export function pointInPoly(pt:Vec, poly:Vec[]){ let c=false; for (let i=0,j=poly.length-1;i<poly.length;j=i++){ const a=poly[i], b=poly[j]; if(((a.y>pt.y)!==(b.y>pt.y)) && (pt.x < (b.x-a.x)*(pt.y-a.y)/(b.y-a.y)+a.x)) c=!c;} return c; }
export function projectPoly(poly:Vec[], axis:Vec){
  let min = dot(poly[0], axis), max = min;
  for (let i=1;i<poly.length;i++){ const p = dot(poly[i], axis); if (p<min) min=p; if (p>max) max=p; }
  return {min, max};
}
export function overlapOnAxis(polyA:Vec[], polyB:Vec[], axis:Vec){ const pa = projectPoly(polyA, axis); const pb = projectPoly(polyB, axis); return Math.min(pa.max, pb.max) - Math.max(pa.min, pb.min); }
export function getAxesFromVerts(poly: Vec[]){ const axes: Vec[] = []; for (let i = 0, j = poly.length - 1; i < poly.length; j = i++){ const a = poly[j], b = poly[i]; const edge = { x: b.x - a.x, y: b.y - a.y }; const nx = -edge.y, ny = edge.x; const L = Math.hypot(nx, ny) || 1; axes.push({ x: nx / L, y: ny / L }); } return axes; }
export function ensureCCW(poly: Vec[]){ if (polygonArea(poly) < 0) return poly.slice().reverse(); return poly; }
// placeholder for physics.getWorldVerts
export function getWorldVerts(b: any){ return b.localVerts ? b.localVerts.map((v:any)=>({ x: v.x + (b.pos?.x||0), y: v.y + (b.pos?.y||0)})) : []; }
export function pointInTriangle(p: Vec, a: Vec, b: Vec, c: Vec){
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

export function earClipTriangulate(polyIn: Vec[]){
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
      const ax = curr.p.x - prev.p.x, ay = curr.p.y - prev.p.y;
      const bx = next.p.x - curr.p.x, by = next.p.y - curr.p.y;
      const crossZ = ax*by - ay*bx;
      if (crossZ <= 0) continue;
      let anyInside = false;
      for (let k=0;k<verts.length;k++){
        if (k===((i-1+verts.length)%verts.length) || k===i || k===((i+1)%verts.length)) continue;
        if (pointInTriangle(verts[k].p, prev.p, curr.p, next.p)){ anyInside = true; break; }
      }
      if (anyInside) continue;
      triangles.push([prev.p, curr.p, next.p]);
      verts.splice(i,1);
      earFound = true;
      break;
    }
    if (!earFound) break;
  }
  if (verts.length === 3) triangles.push([verts[0].p, verts[1].p, verts[2].p]);
  return triangles;
}
