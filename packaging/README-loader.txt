AmnesiaRebirth.FPSFix
=====================

High-frame-rate fix for Amnesia: Rebirth (64-bit Windows).

Amnesia: Rebirth simulates the game 60 times a second and normally draws at
the same rate. This fix lets the game draw at your display's rate and smooths
what you see in between simulation steps, without speeding up physics,
scripts, AI, animation, or audio.

What it smooths
---------------
- Camera movement, including scripted and cutscene cameras
- Mouse look, read at the display rate instead of 60 Hz
- The first-person body, arms, held items, and worn accessories
- Props, doors, thrown and carried objects, and notes being read
- Animated and jointed props
- Creature and character animation and movement
- Particles such as fire, smoke, dust, and water
- Menu, pause-menu, and inventory mouse cursors, including dragged items

Requirements
------------
- Amnesia: Rebirth for Windows. Developed and tested on the GOG build only.
  At startup the fix checks the game code it depends on and switches itself
  off, entirely or only for the affected feature, if anything does not match.
- VSync, an external frame limiter, or the optional limiter in the INI. The
  fix does not cap the frame rate by default.
- For the most direct mouse look, turn Smooth Mouse off in the game's input
  options. With it on, the game's own smoothing is kept.

Installation
------------
Place all files beside AmnesiaRebirth.exe. The included version.dll is the
x64 Ultimate ASI Loader v9.7.4, which loads AmnesiaRebirth.FPSFix.asi when the
game starts. If a version.dll is already there, find out what it belongs to
and back it up before replacing it.

Settings
--------
AmnesiaRebirth.FPSFix.ini lists every feature with a short description. Set
Enabled = false under [General] to switch the fix off without removing it.
To cap the frame rate with the fix itself, set PresentationLimiter = true and
TargetFPS to the rate you want.

Not covered
-----------
- Rope and cloth-like simulations and time-based screen effects (image
  trails, blur, fear effects) still update at 60 Hz.
- In scenes where the game animates the camera, such as the plane seat at
  the start, mouse look is smoothed but stays one simulation step behind.
- The cursor on in-world terminal screens is not smoothed.
- The game has not been tested to the end with this version.
- Mods and custom stories are untested.

Troubleshooting
---------------
AmnesiaRebirth.FPSFix.log is written beside the plugin and replaced on each
launch; the previous session is kept as AmnesiaRebirth.FPSFix.previous.log.
If the game folder is not writable, the log goes to
%LOCALAPPDATA%\AmnesiaRebirth.FPSFix instead. If the fix switches something
off, the reason is in the log. Set Diagnostics = true in the INI before
reproducing a problem you want to report.

Removal
-------
Remove AmnesiaRebirth.FPSFix.asi, AmnesiaRebirth.FPSFix.ini,
AmnesiaRebirth.FPSFix.log, AmnesiaRebirth.FPSFix.previous.log, and
version.dll. Do not remove version.dll if another installed plugin depends on
it. The game's own files and save data are not modified.

Misc
-------
For more game fixes, visit https://slop-blog.enkhayzomachines.net/fixes :)
