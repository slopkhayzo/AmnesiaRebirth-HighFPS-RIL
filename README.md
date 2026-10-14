# AmnesiaRebirth.FPSFix

> [!WARNING]
> This patch has been pretty much entirely been generated using AI; 
> I won't and will never claim to have enough knowledge or expertise to do this 
> kind of reverse-engineering by myself; I've tested till The Oasis at 360hz + 
> briefly tested at 120, 144 and 240 and so far spotted 
> no major issues (on my machine ofc, if you have issues feel free to open an Issue).
> If you're interested and want more (human generated) info, I have a blog post [here](https://slop-blog.enkhayzomachines.net/posts/amnesia-rebirth-high-fps-ril) :)

High-frame-rate presentation fix for the 64-bit Windows build of
*Amnesia: Rebirth*, delivered as an ASI plugin.

The game simulates at a fixed 60 Hz and normally draws at the same rate. The
fix unlocks drawing and presents each frame between the two most recent
simulation states, so motion is smooth at the display's rate while physics,
scripts, AI, animation timing, and audio are unchanged. Everything it changes
for a frame is restored before the simulation runs again.

## What `1.0.0` covers

- Render unlock through the engine's own unlimited-FPS path.
- Camera interpolation on the listener viewport, in both mouse-driven and
  animation-driven (scripted) camera modes.
- Display-rate mouse look: motion is gathered every frame, previewed on the
  rendered camera, and delivered to the game exactly once per tick.
- First-person body root and skeleton, held items, accessories, readable
  inspection props, and script-moved entity roots.
- Newton rigid bodies.
- Skeletal props (`cLuxProp`) and agents (`cLuxAgent`), including agent
  movement.
- Particles (`cParticleEmitter_UserData`).
- Menu, pause-menu, and inventory cursors, plus images and text that follow
  the mouse (dragged items).
- An optional presentation limiter.

Not implemented: Verlet/cloth-like simulations, temporal screen effects,
critters (`cLuxCritter`), and the cursor on in-world terminal screens.

Each layer validates the entry bytes of the game functions it depends on and
stays off if they do not match. Only the GOG build has been tested.

The private `research/` directory (reverse-engineering notes, validation
logs, the working plan) is intentionally excluded from Git.

## Building

Use an x64 MSVC environment with CMake 3.24 or newer:

```text
cmake -S . -B build-x64 -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build-x64 --parallel
ctest --test-dir build-x64 --output-on-failure
```

The ASI and INI are emitted under `build-x64/bin`.

## Packaging

```text
cmake --build build-x64 --target package_plugin_only
cmake --build build-x64 --target package_loader_bundled
```

These write two flat archives to `dist/`:

- `AmnesiaRebirth.FPSFix-<version>-plugin-only.zip`
- `AmnesiaRebirth.FPSFix-<version>-with-Ultimate-ASI-Loader.zip`

The second target exists only when CMake is configured with
`AR_ULTIMATE_ASI_LOADER_DLL` pointing at an official x64 Ultimate ASI Loader
`version.dll`. Both archives carry the same ASI and INI, the end-user README
from `packaging/`, the applicable licenses, and `SHA256SUMS.txt`.

## Logs

`AmnesiaRebirth.FPSFix.log` is written beside the ASI (or under
`%LOCALAPPDATA%\AmnesiaRebirth.FPSFix` if that is not writable) and replaced on
each launch, with the previous session kept as
`AmnesiaRebirth.FPSFix.previous.log`. With `Diagnostics = true` each layer
reports its counters every five seconds.
