# Dyad Simulation Showcase

Interactive, browser-native demos of physical models simulated with
[Dyad](https://juliahub.com/products/dyad) and ModelingToolkit.

**Live:** https://tanmaykm.github.io/DyadShowcase/

Each demo ships the *solved trajectory* as static JSON and replays it with
WebGL. The solver runs ahead of time, not in the browser — so there is no
server, no install, and nothing to spin up.

## Demos

| | |
|---|---|
| [**Theo Jansen walker**](./jansen-walker/) | A twelve-legged strandbeest walking on flat ground or rolling terrain. Body motion from a six-state rigid-body solve with one-sided compliant contact and regularised Coulomb friction. |

## How a demo is built

The split is deliberate — ship the expensive part, recompute the cheap part:

- **Expensive, shipped.** The dynamics. For the walker that is a 6-state solve;
  only the body pose per frame is needed downstream, about 13 KB per case.
- **Cheap, recomputed in-browser.** The linkage geometry. The Jansen leg is a
  chain of five circle intersections, so all eleven bars per leg are rebuilt
  each frame in JavaScript rather than shipped — around 270 KB saved per case.

That means two implementations of one mechanism, which can silently diverge. So
every data file also carries reference joint positions computed by the original
Julia code, and the page asserts against them on load; the result is reported in
the sidebar. A mismatch shows up immediately instead of as a subtly wrong-looking
motion.

## Layout

```
index.html           this landing page
vendor/three/        renderer, shared by every demo (one copy)
<demo-name>/
  index.html         the demo page
  viewer.js          scene and controls
  *.js               model-specific kinematics
  data/*.json        exported trajectories
  preview.png        card image for the landing page
```

Demo pages reference the renderer as `../vendor/three/…`. Relative, not
absolute: a GitHub project site is served from `/<repo>/`, where `/vendor/…`
would resolve to the user root and miss.

## Publishing

This repository is generated. Contents are built in the private model
repository and mirrored here, so edits should be made there and re-published
rather than committed directly.
