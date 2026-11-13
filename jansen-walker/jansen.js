// Theo Jansen leg kinematics, ported from src/jansen_kinematics.jl.
//
// This is the "cheap part" of the split described in scripts/export_web.jl:
// the body pose comes from the Dyad solve (shipped as JSON), while every
// leg joint is recomputed here, per frame, so no per-joint data is shipped.
//
// Having two implementations of one mechanism is exactly the kind of silent
// divergence this project has been bitten by, so `selfCheck()` below asserts
// this chain against reference joints computed in Julia and embedded in each
// data file. It runs on load and reports loudly in the UI on mismatch.

// Circle-circle intersection. `br` (+1/-1) picks which of the two roots --
// i.e. the assembly branch. These match the Julia `_circint` branch flags.
export function circint(c1x, c1y, r1, c2x, c2y, r2, br) {
  const dx = c2x - c1x, dy = c2y - c1y;
  const d = Math.hypot(dx, dy);
  if (d > r1 + r2 || d < Math.abs(r1 - r2) || d === 0) return [NaN, NaN];
  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(r1 * r1 - a * a, 0));
  return [c1x + (a * dx) / d + (br * h * dy) / d,
          c1y + (a * dy) / d - (br * h * dx) / d];
}

// All joints of one leg at crank angle theta, in the leg-local frame
// (hip at the origin). Branch flags (-1,-1,1,1,1) select the authentic
// Jansen assembly -- see MILESTONES.md Milestone 5.
export function legJoints(theta, hn) {
  const Oh = [0, 0];
  const P  = [hn.A + hn.m * Math.cos(theta), hn.L + hn.m * Math.sin(theta)];
  const T  = circint(Oh[0], Oh[1], hn.b, P[0], P[1], hn.j, -1);
  const J1 = circint(Oh[0], Oh[1], hn.d, T[0], T[1], hn.e, -1);
  const J3 = circint(Oh[0], Oh[1], hn.c, P[0], P[1], hn.k, 1);
  const J2 = circint(J1[0], J1[1], hn.f, J3[0], J3[1], hn.g, 1);
  const foot = circint(J2[0], J2[1], hn.h, J3[0], J3[1], hn.i, 1);
  return { Oh, P, T, J1, J3, J2, foot };
}

// The eleven bars, as pairs of joint names. Named for the holy numbers they
// realise, so the drawing and the link lengths stay in correspondence.
export const BARS = [
  ['Oh', 'T'],   // B
  ['Oh', 'J3'],  // C
  ['Oh', 'J1'],  // D
  ['T',  'J1'],  // E
  ['J1', 'J2'],  // F
  ['J3', 'J2'],  // G
  ['J2', 'foot'],// H
  ['J3', 'foot'],// I
  ['P',  'T'],   // J
  ['P',  'J3'],  // K
];

// Terrain profile -- identical to the inline expression in the Dyad component
// (dyad/Locomotion.dyad), so the rendered ground is the ground the solver felt.
export function terrainY(x, t) {
  if (!t || !t.amp) return 0;
  return t.amp * (0.6 * Math.sin(t.k1 * x) + 0.4 * Math.sin(t.k2 * x + t.ph2));
}

// Assert this port against joints computed in Julia. Returns {ok, worst, detail}.
export function selfCheck(data, tol = 1e-6) {
  const hn = data.meta.holy;
  let worst = 0, detail = '';
  for (const key of Object.keys(data.reference)) {
    const ref = data.reference[key];
    const got = legJoints(ref.theta, hn);
    for (const name of ['P', 'T', 'J1', 'J3', 'J2', 'foot']) {
      const e = Math.hypot(got[name][0] - ref[name][0], got[name][1] - ref[name][1]);
      if (e > worst) { worst = e; detail = `${key}.${name}`; }
    }
  }
  return { ok: worst <= tol, worst, detail };
}
