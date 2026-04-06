import * as M from './math';
import * as P from './physics';
import * as R from './render';
import * as S from './saveLoad';
import * as REN from './rendering';
import { CONFIG } from './settings';

// local aliases for commonly used helpers (avoid renaming all references)
const dot = M.dot;
const cross = M.cross;
const normalize = M.normalize;
const polygonArea = M.polygonArea;
const pointInPoly = M.pointInPoly;
const earClipTriangulate = M.earClipTriangulate;
const projectPoly = M.projectPoly;
const getAxesFromVerts = M.getAxesFromVerts;
const ensureCCW = M.ensureCCW;
const centroidOf = M.centroidOf;

// type aliases to match previous single-file names used throughout main.ts
type Vec = M.Vec;
type RBody = P.RBody;

// physics helpers used by main.ts (alias the exports to preserve original names)
const getWorldVerts = P.getWorldVerts;
const finalizeBody = P.finalizeBody;
const applyImpulse = P.applyImpulse;

const randomColor = R.randomColor;

// export/import scene helpers
const exportSceneJSON = S.exportSceneJSON;
const downloadJSON = S.downloadJSON;
const importSceneFromJSON = S.importSceneFromJSON;

const renderSceneFunc = REN.renderScene;

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const ctx = canvas.getContext('2d')!;

let floorY = window.innerHeight - 80;

function resize() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
  // keep floorY in world coords; if you want adaptive, change here
  floorY = canvas.height - 80;
}
window.addEventListener('resize', resize);
resize();

// world and input state (single canonical copy)
const bodies: P.RBody[] = [];
// expose a global reference to help detect shadowing/caching issues at runtime
;(window as any).__bodies = bodies;
let nextId = 1;
const gravity = { x: 0, y: CONFIG.gravity };
let mouseDown = false;
let ctrlDown = false;
let startPos: M.Vec | null = null;
let currentPos: M.Vec | null = null;
let draggingBody: P.RBody | null = null;
let isDragging = false;
// preview creation state
let previewSides = 6;

// Freeform polygon while Ctrl is held
let tempFreeVerts: M.Vec[] = [];

// lightweight debug: only log when bodies length changes to avoid flooding the console
let lastBodiesCount = -1;

// mouse/keyboard handlers (creation, freeform, dragging)
canvas.addEventListener('mousedown', (e)=>{
  if (e.button === 1){ // middle click -> start panning
    isPanning = true;
    panStartScreen = { x: e.clientX, y: e.clientY };
    panStartOffset = { x: viewOffset.x, y: viewOffset.y };
    e.preventDefault();
    return;
  }
  const screenP = { x: e.clientX, y: e.clientY };
  const p = screenToWorld(screenP);
  if (e.button !== 0) return;
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
      dragLocalAnchor = worldToLocal(b, p);
      startPos = null;
      console.debug('start dragging body', b.id);
      return;
    }
  }
});

canvas.addEventListener('mousemove', (e)=>{
  const screenP = { x: e.clientX, y: e.clientY };
  if (isPanning && panStartScreen && panStartOffset){
    viewOffset.x = panStartOffset.x + (screenP.x - panStartScreen.x);
    viewOffset.y = panStartOffset.y + (screenP.y - panStartScreen.y);
    return;
  }
  currentPos = screenToWorld(screenP);
});

canvas.addEventListener('mouseup', (e)=>{
  if (e.button === 1){ isPanning = false; panStartScreen = null; panStartOffset = null; return; }
  const screenP = { x: e.clientX, y: e.clientY };
  currentPos = screenToWorld(screenP);
  if (e.button !== 0) return;
  console.debug('mouseup', { mouseDown, startPos, currentPos, isDragging, ctrlDown });
  mouseDown = false;
  if (isDragging) {
    if (draggingBody && currentPos){
      const b = draggingBody;
      const contact = dragLocalAnchor ? localToWorld(b, dragLocalAnchor) : currentPos;
      const r = { x: contact.x - b.pos.x, y: contact.y - b.pos.y };
      const velAtContact = { x: b.vel.x + (-b.angVel * r.y), y: b.vel.y + (b.angVel * r.x) };
      const dir = { x: currentPos.x - contact.x, y: currentPos.y - contact.y };
      const dist = Math.hypot(dir.x, dir.y);
      if (dist > 1e-3){
        const k = 6;
        const desiredVel = { x: dir.x * k, y: dir.y * k };
        const impulse = { x: (desiredVel.x - velAtContact.x) * (b.mass), y: (desiredVel.y - velAtContact.y) * (b.mass) };
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
  console.debug('after push: bodies.length=', bodies.length, 'global same?', (window as any).__bodies === bodies);
  startPos = null;
});

// wheel: when creating (mouseDown) change sides; otherwise zoom viewport
canvas.addEventListener('wheel', (e)=>{
  if (mouseDown && !isDragging && !ctrlDown){
    e.preventDefault();
    const delta = Math.sign(e.deltaY);
    if (delta < 0) previewSides = Math.min(12, previewSides + 1);
    else previewSides = Math.max(3, previewSides - 1);
    return;
  }
  // zoom when not creating
  if (!mouseDown){
    const screen = { x: e.clientX, y: e.clientY };
    const before = screenToWorld(screen);
    const scaleFactor = Math.exp(-e.deltaY * 0.0015); // smooth
    const newScale = Math.max(0.1, Math.min(5, viewScale * scaleFactor));
    viewScale = newScale;
    // adjust offset so the point under cursor remains under cursor
    const after = before; // world coordinate stays same
    viewOffset.x = screen.x - after.x * viewScale;
    viewOffset.y = screen.y - after.y * viewScale;
  }
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

// debug drawing helper: draw arrow is provided by render module (R.drawArrow)
// local implementation removed to avoid duplicate identifier with import

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

// --- Scene save / load utilities ---
// save/load helpers are provided by saveLoad module; local implementations removed

// hidden file input used for "Load" button and Ctrl+O
const __sceneFileInput = document.createElement('input');
__sceneFileInput.type = 'file';
__sceneFileInput.accept = 'application/json';
__sceneFileInput.style.display = 'none';
__sceneFileInput.addEventListener('change', ()=>{
  const f = __sceneFileInput.files && __sceneFileInput.files[0];
  if (!f) return;
  const reader = new FileReader();
  reader.onload = ()=>{ if (typeof reader.result === 'string') importSceneFromJSON(reader.result); };
  reader.readAsText(f);
});
document.body.appendChild(__sceneFileInput);

// Wire up common button IDs for Save/Load if they exist in the page
const saveIds = ['save','saveBtn','save-button','btnSave','save-scene','saveScene'];
for (const id of saveIds){ const el = document.getElementById(id); if (el) el.addEventListener('click', ()=>{ downloadJSON('scene.json', exportSceneJSON(bodies, nextId)); }); }
const loadIds = ['load','loadBtn','load-button','btnLoad','load-scene','loadScene'];
for (const id of loadIds){ const el = document.getElementById(id); if (el) el.addEventListener('click', ()=>{ __sceneFileInput.value = ''; __sceneFileInput.click(); }); }

// Keyboard shortcuts: Ctrl+S to save, Ctrl+O to open (prevents browser default)
window.addEventListener('keydown', (e)=>{
  if ((e.ctrlKey || e.metaKey) && (e.key === 's' || e.key === 'S')){
    e.preventDefault();
    downloadJSON('scene.json', exportSceneJSON(bodies, nextId));
  }
  if ((e.ctrlKey || e.metaKey) && (e.key === 'o' || e.key === 'O')){
    e.preventDefault();
    __sceneFileInput.value = '';
    __sceneFileInput.click();
  }
});

// alias physics collision function
const polygonCollideSAT = P.polygonCollideSAT;

// viewport / camera (world-to-screen mapping)
let viewOffset: Vec = { x: 0, y: 0 };
let viewScale = 1;
export function screenToWorld(p: Vec){ return { x: (p.x - viewOffset.x) / viewScale, y: (p.y - viewOffset.y) / viewScale }; }
export function worldToScreen(p: Vec){ return { x: p.x * viewScale + viewOffset.x, y: p.y * viewScale + viewOffset.y }; }
// panning state
let isPanning = false;
let panStartScreen: Vec | null = null;
let panStartOffset: Vec | null = null;

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
    const solverIterations = 6;
    for (let iter=0; iter<solverIterations; iter++){
      for (let i=0;i<bodies.length;i++){
        for (let k=i+1;k<bodies.length;k++){
          const A = bodies[i], B = bodies[k];
          if (A===B) continue;
          const info = polygonCollideSAT(A,B);
          if (!info) continue;
          const n = info.normal; const pen = info.penetration; const p = info.contactPoint;
          // stronger positional correction on first iteration
          //          const percent = (iter === 0) ? 0.5 : 0.2; const slop = 0.01;
          // limit positional correction to be milder per iteration
          const percent = (iter === 0) ? 0.25 : 0.12; const slop = 0.01;
          // clamp maximum correction magnitude per-axis to avoid teleport-like moves
          const MAX_CORR_PER_ITER = 2; // pixels
           const invMassSum = A.invMass + B.invMass;
           if (invMassSum > 0){
             let corrMag = Math.max(pen - slop, 0) / invMassSum * percent;
             corrMag = Math.min(corrMag, MAX_CORR_PER_ITER);
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
           // apply normal impulse
           const impulse2 = { x: n.x * j2, y: n.y * j2 };
           if (!A.isStatic) applyImpulse(A, { x: -impulse2.x, y: -impulse2.y }, p);
           if (!B.isStatic) applyImpulse(B, impulse2, p);
           // Coulomb friction: tangential impulse
           const t = { x: -n.y, y: n.x }; // tangent (unit because n is unit)
           const vt = dot(rv, t);
           const raCrossT = cross(rA, t);
           const rbCrossT = cross(rB, t);
           const denomT = A.invMass + B.invMass + raCrossT*raCrossT * A.invInertia + rbCrossT*rbCrossT * B.invInertia;
           let jt = denomT === 0 ? 0 : -vt / denomT;
           const mu = 0.4; // friction coefficient between bodies
           // clamp friction by Coulomb's law (use magnitude of normal impulse)
           const jtMax = Math.abs(j2) * mu;
           if (jt > jtMax) jt = jtMax; if (jt < -jtMax) jt = -jtMax;
           if (Math.abs(jt) > 1e-8){
             if (!A.isStatic) applyImpulse(A, { x: -t.x * jt, y: -t.y * jt }, p);
             if (!B.isStatic) applyImpulse(B, { x: t.x * jt, y: t.y * jt }, p);
           }
         }
       }
     }
  }

  // determine hovered body under mouse (top-most)
  hoveredBody = null;
  if (currentPos){
    for (let i = bodies.length - 1; i >= 0; i--) {
      const b = bodies[i];
      const wv = getWorldVerts(b);
      if (pointInPoly(currentPos, wv)) { hoveredBody = b; break; }
    }
  }

  // render
  ctx.clearRect(0,0,canvas.width,canvas.height);
  // use renderScene for main world rendering
  renderSceneFunc(ctx, canvas, bodies, currentPos ?? { x: 0, y: 0 }, viewOffset, viewScale, { floorY, debugDraw, isDragging, draggingBody, dragLocalAnchor, currentPos, localToWorld, hoveredBody, mouseDown, ctrlDown, startPos, tempFreeVerts, previewSides });

  // HUD (screen-space)
  ctx.fillStyle = 'black'; ctx.font='16px Arial'; ctx.textAlign='left'; ctx.textBaseline='top';
  ctx.fillText(`FPS: ${fps}`, 10, 10);
  ctx.fillText(`Bodies: ${bodies.length}`, 10, 30);
  ctx.fillText(`Mode: ${ctrlDown ? 'Freeform' : 'Regular'}`, 10, 50);
  ctx.fillText(`Preview Sides: ${previewSides}`, 10, 70);
  ctx.fillText(`Scale: ${viewScale.toFixed(2)}`, 10, 90);
  ctx.fillText(`Cam: (${Math.round(viewOffset.x)}, ${Math.round(viewOffset.y)})`, 10, 110);
  // debug: vertex normal arrows removed — they caused incorrect placement during pan/zoom.
  // The drag impulse arrow (for the body being dragged) is still drawn earlier in world-space when debugDraw is enabled.

  // FPS measurement
  frameCount++;
  if (ts - fpsLastTime >= 500){ fps = Math.round(frameCount * 1000 / (ts - fpsLastTime)); frameCount = 0; fpsLastTime = ts; }

  // schedule next frame
  requestAnimationFrame(step);
}

requestAnimationFrame(step);
