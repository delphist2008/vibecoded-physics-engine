import { Vec, earClipTriangulate } from './math';
import { getWorldVerts } from './physics';
import { drawArrow } from './render';

export function renderScene(ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement, bodies: any[], currentPos: Vec, viewOffset: Vec, viewScale: number, params: any){
  // apply view transform
  ctx.save();
  ctx.setTransform(viewScale, 0, 0, viewScale, viewOffset.x, viewOffset.y);
  // background
  const worldLeft = -viewOffset.x / viewScale; const worldTop = -viewOffset.y / viewScale;
  const worldWidth = canvas.width / viewScale; const worldHeight = canvas.height / viewScale;
  const padX = worldWidth * 10; const padY = worldHeight * 10;
  ctx.fillStyle = '#aee1ff'; ctx.fillRect(worldLeft - padX, worldTop - padY, worldWidth + padX * 2, worldHeight + padY * 2);
  ctx.fillStyle = '#6aa84f'; ctx.fillRect(worldLeft - padX, params.floorY, worldWidth + padX * 2, 1e6);

  // draw bodies and optionally triangulation / hover highlight
  for (let b of bodies){
    const wv = getWorldVerts(b);
    ctx.beginPath();
    for (let i=0;i<wv.length;i++){ const v=wv[i]; if (i===0) ctx.moveTo(v.x,v.y); else ctx.lineTo(v.x,v.y); }
    ctx.closePath(); ctx.fillStyle = b.color; ctx.fill(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(0,0,0,0.2)'; ctx.stroke();

    // hover highlight
    if (params.hoveredBody === b){
      ctx.beginPath();
      for (let i=0;i<wv.length;i++){ const v=wv[i]; if (i===0) ctx.moveTo(v.x,v.y); else ctx.lineTo(v.x,v.y); }
      ctx.closePath(); ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,0,0.9)'; ctx.stroke();
    }

    // debug: show triangulation for this body
    if (params.debugDraw){
      try{
        const tris = earClipTriangulate(wv);
        ctx.save(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(0,0,0,0.25)'; ctx.fillStyle = 'rgba(0,0,0,0.03)';
        for (let t of tris){ ctx.beginPath(); ctx.moveTo(t[0].x,t[0].y); ctx.lineTo(t[1].x,t[1].y); ctx.lineTo(t[2].x,t[2].y); ctx.closePath(); ctx.fill(); ctx.stroke(); }
        ctx.restore();
      }catch(e){ /* triangulation best-effort */ }
    }

    // draw drag arrow for the dragged body (rendering module draws a global drag arrow too)
  }

  // creation preview: regular polygon (mouse drag) when not freeform
  if (params.mouseDown && params.startPos && !params.ctrlDown && params.currentPos){
    const center: Vec = params.startPos;
    const dx = params.currentPos.x - center.x, dy = params.currentPos.y - center.y;
    const radius = Math.max(6, Math.hypot(dx, dy));
    const angle = Math.atan2(dy, dx);
    const sides = Math.max(3, Math.min(64, params.previewSides || 6));
    const verts: Vec[] = [];
    ctx.beginPath();
    for (let i=0;i<sides;i++){ const a = i/sides*2*Math.PI + angle; const v = { x: center.x + Math.cos(a)*radius, y: center.y + Math.sin(a)*radius }; verts.push(v); if (i===0) ctx.moveTo(v.x,v.y); else ctx.lineTo(v.x,v.y); }
    ctx.closePath(); ctx.fillStyle = 'rgba(100,100,255,0.12)'; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = 'rgba(60,60,180,0.9)'; ctx.stroke();
    if (params.debugDraw){ try{ const tris = earClipTriangulate(verts); ctx.save(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(0,0,0,0.25)'; for (let t of tris){ ctx.beginPath(); ctx.moveTo(t[0].x,t[0].y); ctx.lineTo(t[1].x,t[1].y); ctx.lineTo(t[2].x,t[2].y); ctx.closePath(); ctx.stroke(); } ctx.restore(); }catch(e){} }
  }

  // freeform preview while holding Ctrl: draw polyline of temp verts and from last to cursor
  if (params.ctrlDown && params.tempFreeVerts && params.tempFreeVerts.length > 0){
    const pts: Vec[] = params.tempFreeVerts;
    ctx.beginPath(); ctx.lineWidth = 2; ctx.strokeStyle = 'rgba(50,150,50,0.95)';
    for (let i=0;i<pts.length;i++){ const v=pts[i]; if (i===0) ctx.moveTo(v.x,v.y); else ctx.lineTo(v.x,v.y); }
    if (params.currentPos) ctx.lineTo(params.currentPos.x, params.currentPos.y);
    ctx.stroke();
    // draw vertices
    for (let v of pts){ ctx.beginPath(); ctx.fillStyle = 'white'; ctx.strokeStyle = 'rgba(0,0,0,0.5)'; ctx.arc(v.x, v.y, 4, 0, Math.PI*2); ctx.fill(); ctx.stroke(); }
    if (params.currentPos){ ctx.beginPath(); ctx.fillStyle = 'rgba(0,0,0,0.6)'; ctx.arc(params.currentPos.x, params.currentPos.y, 4, 0, Math.PI*2); ctx.fill(); }
    if (params.debugDraw && params.tempFreeVerts.length >= 3){ try{ const tris = earClipTriangulate(pts.concat(params.currentPos || [])); ctx.save(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(0,0,0,0.25)'; for (let t of tris){ ctx.beginPath(); ctx.moveTo(t[0].x,t[0].y); ctx.lineTo(t[1].x,t[1].y); ctx.lineTo(t[2].x,t[2].y); ctx.closePath(); ctx.stroke(); } ctx.restore(); }catch(e){} }
  }

  // optionally draw debug drag arrow for the currently dragged body (if any)
  if (params.debugDraw && params.isDragging && params.draggingBody && params.currentPos){
    const b = params.draggingBody;
    const contact = params.dragLocalAnchor && b.localToWorld ? b.localToWorld(params.dragLocalAnchor) : (params.localToWorld ? params.localToWorld(b, params.dragLocalAnchor) : { x: b.pos.x, y: b.pos.y });
    const r = { x: contact.x - b.pos.x, y: contact.y - b.pos.y };
    const velAtContact = { x: b.vel.x + (-b.angVel * r.y), y: b.vel.y + (b.angVel * r.x) };
    const dir = { x: params.currentPos.x - contact.x, y: params.currentPos.y - contact.y };
    const desiredVel = { x: dir.x * 6, y: dir.y * 6 };
    const dv = { x: desiredVel.x - velAtContact.x, y: desiredVel.y - velAtContact.y };
    const alpha = 0.15; const impulse = { x: dv.x * b.mass * alpha, y: dv.y * b.mass * alpha };
    const lenImp = Math.hypot(impulse.x, impulse.y) || 1; const ux = impulse.x / lenImp, uy = impulse.y / lenImp;
    const arrowEnd = { x: contact.x + ux * 80, y: contact.y + uy * 80 };
    drawArrow(ctx, contact, arrowEnd, 'red');
  }

  ctx.restore();
}
