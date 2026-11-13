// Three.js viewer for the Dyad Jansen walker.
//
// Body pose per frame comes from the Dyad JansenLocomotion solve (docs/data/*.json);
// every leg joint is recomputed here each frame via docs/jansen.js.
//
// Coordinate mapping: the Dyad model is planar -- x forward, y up. The rover is
// that plane extruded across z, with a leg bank on each side. Both banks at a
// station share the crank phase, so the side view IS the simulated model.

// vendor/ lives at the site root and is shared by every demo, so it is one
// level up from a demo folder. Relative (not absolute) because GitHub project
// pages are served from /<repo>/, where "/vendor/..." would miss.
import * as THREE from '../vendor/three/three.module.js';
import { OrbitControls } from '../vendor/three/controls/OrbitControls.js';
import { legJoints, BARS, terrainY, selfCheck } from './jansen.js';

const CASES = { flat: './data/flat.json', terrain: './data/terrain.json' };
const STATION_COLORS = [0x4f9dff, 0xff7a45, 0x3ddc84, 0xffd166, 0xc77dff, 0xff6b9d];

const state = {
  data: null, frame: 0, playing: true, speed: 1,
  bankGap: 18, trail: true, follow: true, t0: performance.now(),
};

// ---------------------------------------------------------------- scene
const container = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);
scene.fog = new THREE.Fog(0x0e1116, 220, 650);

const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 3000);
camera.position.set(-70, 58, 96);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 14, 0);

// lighting: one key with shadows, one fill, plus sky/ground ambient
const key = new THREE.DirectionalLight(0xffffff, 2.6);
key.position.set(60, 120, 70);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
key.shadow.camera.near = 10;
key.shadow.camera.far = 400;
const S = 120;
Object.assign(key.shadow.camera, { left: -S, right: S, top: S, bottom: -S });
key.shadow.bias = -0.0008;
scene.add(key, key.target);
scene.add(new THREE.DirectionalLight(0x88bbff, 0.5).translateX(-80).translateY(40));
scene.add(new THREE.HemisphereLight(0x9ec5ff, 0x202a36, 0.65));

// ---------------------------------------------------------------- ground
const GROUND_LEN = 900, GROUND_W = 150, GSEG = 440;
let groundMesh = null, gridLines = null;

function buildGround(terrain) {
  if (groundMesh) { scene.remove(groundMesh); groundMesh.geometry.dispose(); }
  if (gridLines) { scene.remove(gridLines); gridLines.geometry.dispose(); }

  const geo = new THREE.PlaneGeometry(GROUND_LEN, GROUND_W, GSEG, 2);
  geo.rotateX(-Math.PI / 2);
  const pos = geo.attributes.position;
  for (let i = 0; i < pos.count; i++) pos.setY(i, terrainY(pos.getX(i), terrain));
  geo.computeVertexNormals();

  groundMesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
    color: 0x1b2430, roughness: 0.96, metalness: 0.0,
  }));
  groundMesh.receiveShadow = true;
  scene.add(groundMesh);

  // contour lines along travel, for a readable sense of motion and relief
  const pts = [];
  for (let z = -GROUND_W / 2; z <= GROUND_W / 2 + 0.1; z += 15) {
    for (let i = 0; i < GSEG; i++) {
      const x0 = -GROUND_LEN / 2 + (i * GROUND_LEN) / GSEG;
      const x1 = -GROUND_LEN / 2 + ((i + 1) * GROUND_LEN) / GSEG;
      pts.push(new THREE.Vector3(x0, terrainY(x0, terrain) + 0.05, z),
               new THREE.Vector3(x1, terrainY(x1, terrain) + 0.05, z));
    }
  }
  gridLines = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineBasicMaterial({ color: 0x33506e, transparent: true, opacity: 0.55 }));
  scene.add(gridLines);
}

// ---------------------------------------------------------------- rover
const rover = new THREE.Group();
scene.add(rover);

const barGeo = new THREE.CylinderGeometry(0.42, 0.42, 1, 7);
barGeo.translate(0, 0.5, 0);                       // origin at one end
const jointGeo = new THREE.SphereGeometry(0.62, 12, 10);
const footGeo = new THREE.SphereGeometry(1.0, 14, 12);
const chassisMat = new THREE.MeshStandardMaterial({ color: 0x8e9bab, roughness: 0.45, metalness: 0.65 });
const footMat = new THREE.MeshStandardMaterial({ color: 0xff4d5a, roughness: 0.35, emissive: 0x450d12 });

let legMeshes = [];   // [{bars:[Mesh], joints:[Mesh], foot:Mesh, station, side}]
let chassisBars = [];
const up = new THREE.Vector3(0, 1, 0);

function placeBar(mesh, a, b) {
  const dir = new THREE.Vector3().subVectors(b, a);
  const len = dir.length();
  mesh.position.copy(a);
  mesh.quaternion.setFromUnitVectors(up, dir.normalize());
  mesh.scale.set(1, len, 1);
}

function buildRover(meta) {
  legMeshes.forEach(l => { l.bars.forEach(m => rover.remove(m)); l.joints.forEach(m => rover.remove(m)); rover.remove(l.foot); });
  chassisBars.forEach(m => rover.remove(m));
  legMeshes = []; chassisBars = [];

  for (let i = 0; i < meta.N; i++) {
    const color = STATION_COLORS[i % STATION_COLORS.length];
    const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.4, metalness: 0.35 });
    for (const side of [+1, -1]) {
      const bars = BARS.map(() => {
        const m = new THREE.Mesh(barGeo, mat); m.castShadow = true; rover.add(m); return m;
      });
      const joints = Array.from({ length: 7 }, () => {
        const m = new THREE.Mesh(jointGeo, mat); m.castShadow = true; rover.add(m); return m;
      });
      const foot = new THREE.Mesh(footGeo, footMat); foot.castShadow = true; rover.add(foot);
      legMeshes.push({ bars, joints, foot, station: i, side });
    }
  }
  // chassis: two side rails + a cross member per station
  const nRails = 2 + meta.N;
  for (let i = 0; i < nRails; i++) {
    const m = new THREE.Mesh(barGeo, chassisMat); m.castShadow = true;
    m.scale.set(1.7, 1, 1.7); rover.add(m); chassisBars.push(m);
  }
}

// ---------------------------------------------------------------- trail
const MAXTRAIL = 4000;
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAXTRAIL * 3), 3));
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x4f9dff, transparent: true, opacity: 0.8 }));
trail.frustumCulled = false;
scene.add(trail);
let trailN = 0;

// ---------------------------------------------------------------- frame update
const tmpA = new THREE.Vector3(), tmpB = new THREE.Vector3();

function updateFrame(k) {
  const d = state.data, m = d.meta, f = d.frames;
  const X = f.X[k], Y = f.Y[k], Phi = f.Phi[k], t = f.t[k];
  const cp = Math.cos(Phi), sp = Math.sin(Phi);
  const gap = state.bankGap;

  // body-frame (bx, by) -> world, applying pitch about the COM then translation
  const toWorld = (bx, by, bz) =>
    tmpA.set(X + bx * cp - by * sp, Y + bx * sp + by * cp, bz);

  for (const leg of legMeshes) {
    const i = leg.station;
    const theta = -m.w * t + m.phases[i];
    const J = legJoints(theta, m.holy);
    const hipBx = m.hipX[i] - m.xcom;
    const bz = (leg.side * gap) / 2;

    const W = {};
    for (const nm of ['Oh', 'P', 'T', 'J1', 'J3', 'J2', 'foot']) {
      W[nm] = toWorld(hipBx + J[nm][0], J[nm][1], bz).clone();
    }
    BARS.forEach(([a, b], bi) => placeBar(leg.bars[bi], W[a], W[b]));
    ['Oh', 'P', 'T', 'J1', 'J3', 'J2', 'foot'].forEach((nm, ji) => leg.joints[ji].position.copy(W[nm]));
    leg.foot.position.copy(W.foot);
  }

  // chassis rails along each side + cross members at each station
  const hipFirst = m.hipX[0] - m.xcom, hipLast = m.hipX[m.N - 1] - m.xcom;
  placeBar(chassisBars[0], toWorld(hipFirst, 0, +gap / 2).clone(), toWorld(hipLast, 0, +gap / 2).clone());
  placeBar(chassisBars[1], toWorld(hipFirst, 0, -gap / 2).clone(), toWorld(hipLast, 0, -gap / 2).clone());
  for (let i = 0; i < m.N; i++) {
    const hb = m.hipX[i] - m.xcom;
    placeBar(chassisBars[2 + i], toWorld(hb, 0, +gap / 2).clone(), toWorld(hb, 0, -gap / 2).clone());
  }

  // trail follows the body origin
  if (state.trail) {
    const p = trailGeo.attributes.position;
    if (trailN < MAXTRAIL) {
      p.setXYZ(trailN, X, Y - 1.5, 0); trailN++;
      trailGeo.setDrawRange(0, trailN); p.needsUpdate = true;
    }
  }

  if (state.follow) {
    controls.target.lerp(tmpB.set(X, Y - 2, 0), 0.12);
    key.position.set(X + 60, 120, 70);
    key.target.position.set(X, 0, 0);
    key.target.updateMatrixWorld();
  }

  document.getElementById('readout').innerHTML =
    `t <b>${t.toFixed(2)}</b> s &nbsp; x <b>${X.toFixed(1)}</b> cm &nbsp; ` +
    `speed <b>${(X / Math.max(t, 1e-6)).toFixed(1)}</b> cm/s &nbsp; ` +
    `ride <b>${Y.toFixed(2)}</b> cm &nbsp; pitch <b>${((Phi * 180) / Math.PI).toFixed(1)}</b>&deg;`;
  document.getElementById('scrub').value = k;
}

// ---------------------------------------------------------------- data load
async function load(caseName) {
  const res = await fetch(CASES[caseName]);
  state.data = await res.json();
  const d = state.data;

  const chk = selfCheck(d);
  const el = document.getElementById('check');
  if (chk.ok) {
    el.innerHTML = `leg kinematics verified against Julia &mdash; max error ${chk.worst.toExponential(1)} cm`;
    el.className = 'ok';
  } else {
    el.innerHTML = `JS/Julia leg kinematics DIVERGE at ${chk.detail}: ${chk.worst.toExponential(2)} cm`;
    el.className = 'bad';
  }

  buildGround(d.meta.terrain);
  buildRover(d.meta);
  trailN = 0; trailGeo.setDrawRange(0, 0);
  document.getElementById('scrub').max = d.frames.t.length - 1;
  state.frame = 0;
  updateFrame(0);
}

// ---------------------------------------------------------------- loop + UI
function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h; camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(container);

let acc = 0, last = performance.now();
function tick(now) {
  requestAnimationFrame(tick);
  const dt = (now - last) / 1000; last = now;
  if (state.data && state.playing) {
    acc += dt * state.speed * state.data.meta.fps;
    while (acc >= 1) {
      acc -= 1;
      state.frame = (state.frame + 1) % state.data.frames.t.length;
      if (state.frame === 0) { trailN = 0; trailGeo.setDrawRange(0, 0); }
    }
    updateFrame(state.frame);
  }
  controls.update();
  renderer.render(scene, camera);
}

document.getElementById('play').onclick = (e) => {
  state.playing = !state.playing;
  e.target.textContent = state.playing ? 'pause' : 'play';
};
document.getElementById('scrub').oninput = (e) => {
  state.frame = +e.target.value; state.playing = false;
  document.getElementById('play').textContent = 'play';
  updateFrame(state.frame);
};
document.getElementById('speed').oninput = (e) => {
  state.speed = +e.target.value;
  document.getElementById('speedv').textContent = state.speed.toFixed(2) + '×';
};
document.getElementById('gap').oninput = (e) => {
  state.bankGap = +e.target.value;
  document.getElementById('gapv').textContent = state.bankGap + ' cm';
  if (state.data) updateFrame(state.frame);
};
document.getElementById('follow').onchange = (e) => (state.follow = e.target.checked);
document.getElementById('trailc').onchange = (e) => {
  state.trail = e.target.checked;
  if (!state.trail) { trailN = 0; trailGeo.setDrawRange(0, 0); }
};
for (const r of document.querySelectorAll('input[name=case]')) {
  r.onchange = (e) => load(e.target.value);
}

// ?case=terrain makes a particular scene shareable by URL
const wanted = new URLSearchParams(location.search).get('case');
const initial = wanted && CASES[wanted] ? wanted : 'flat';
document.querySelector(`input[name=case][value="${initial}"]`).checked = true;

resize();
await load(initial);
requestAnimationFrame(tick);
