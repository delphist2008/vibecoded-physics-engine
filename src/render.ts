import { Vec, polygonArea } from './math';

export function randomColor(){ const h = Math.floor(Math.random() * 360); return `hsl(${h} 60% 60%)`; }

export function drawArrow(ctx: CanvasRenderingContext2D, a:Vec, b:Vec, color:string='red'){
  const dx = b.x - a.x, dy = b.y - a.y; const L = Math.hypot(dx,dy) || 1;
  const ux = dx / L, uy = dy / L;
  ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 3; ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
  const hs = Math.min(12, L*0.2);
  const hx = b.x - ux*hs, hy = b.y - uy*hs;
  const left = { x: hx + -uy * (hs*0.5), y: hy + ux * (hs*0.5) };
  const right = { x: hx + uy * (hs*0.5), y: hy + -ux * (hs*0.5) };
  ctx.beginPath(); ctx.moveTo(b.x,b.y); ctx.lineTo(left.x,left.y); ctx.lineTo(right.x,right.y); ctx.closePath(); ctx.fillStyle = color; ctx.fill();
}

export function polygonAreaAbs(pts:Vec[]){ return Math.abs(polygonArea(pts)); }
