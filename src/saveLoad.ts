// lightweight, dependency-free save/load helpers
export function exportSceneJSON(bodies: any[], nextId:number){
  const out = bodies.map((b:any) => ({
    localVerts: b.localVerts,
    pos: b.pos,
    vel: b.vel,
    ang: b.ang,
    angVel: b.angVel,
    mass: b.mass,
    color: b.color,
    isStatic: !!b.isStatic
  }));
  return JSON.stringify({ version: 1, nextId, bodies: out }, null, 2);
}

export function downloadJSON(filename: string, data: string){
  const blob = new Blob([data], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = filename; document.body.appendChild(a);
  a.click(); a.remove();
  URL.revokeObjectURL(url);
}

export function importSceneFromJSON(text: string){
  const obj = JSON.parse(text);
  if (!obj || !Array.isArray(obj.bodies)) throw new Error('Invalid scene file');
  return obj;
}
