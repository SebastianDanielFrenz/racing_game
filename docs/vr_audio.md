# VR and spatial audio

The game consumes physics_sim from S:/claude_code/physics_sim, pinned to
ba47951a0834cb62a38700c61de87a155c13c747. This committed master revision
includes the supplied hypercar, terrain boundary fix, Windows spatial backend,
and immutable live-voice source metadata. Pending turbo/audio-bank changes
in the physics owner's working tree are not part of this version.

## Launch

From S:/claude_code/racing_game:

```powershell
.\run.cmd -VR
.\run.cmd -VR -Flat
.\run.cmd -- --audio=godot
```

`-VR` selects Vulkan Mobile and supplies `--xr-mode on` before Godot's
user-argument separator. Configure the headset's OpenXR runtime first.
Normal launches keep OpenXR disabled. If no initialized OpenXR interface is
available, the game retains its desktop camera rigs. An unsupported graphics
driver/runtime may still reject startup before the game script executes.

In VR the driver camera follows the model's socket_driver_eye, including the
model-to-chassis alignment. F9 recenters the seated head pose. FreeCam uses
a tracked free-flight rig and the existing camera movement controls; headset
rotation supplies look direction. The camera director owns rebasing and
terrain streaming for both tracked rigs. The active tracked head transform
also supplies the sound listener. VR controllers and headset rendering were
not verified on hardware in this change.

See [Godot XR setup](https://docs.godotengine.org/en/4.7/tutorials/xr/setting_up_xr.html).

## Audio

The one game GDExtension compiles the library's PsSpatialAudio and VehicleVoice
sources directly. It does not load the physics demo's second extension DLL.
Tyre audio reuses the committed demo's rolling/squeal model and licensed sample
from external/physics_sim/adapters/godot/demo/audio/tyres; the sample's license
remains alongside it in the pinned submodule. Its 24 kHz playback rate is
preserved when supplying the backend's 48 kHz PCM.

Wheel force and angular-speed telemetry is published with the existing frame
snapshot. The main thread is the single producer of live engine voice inputs
from those published engine states; the voice renderer accesses no physics
world. Each intake/exhaust or tyre source is transformed from chassis-local
coordinates to active-listener coordinates. Audio is shut down before world
replacement, and shutdown is idempotent. Streaming freezes mute tyre contact
audio and pause the live voice.

Windows Spatial Sound is attempted automatically. Select Dolby Atmos for
Headphones or home theater on the Windows default multimedia endpoint, with
the appropriate Dolby Access activation/equipment. Windows Sonic and DTS
providers can use the same object API. The game neither installs Dolby
software nor changes Windows settings. Stream activation confirms API
availability, not the selected provider or downstream speaker output.
See [Microsoft Spatial Sound](https://learn.microsoft.com/en-us/windows/win32/coreaudio/spatial-sound).

If device opening fails or the stream is lost, each source uses Godot 3D audio.
`--audio=godot` forces this fallback; `--m1-audio=godot` is also accepted for
demo parity. `--no-tyre-audio` disables tyre generation. Restart after changing
the Windows default endpoint to retry native output.

The committed car_hyper uses its authored 1 MW torque-map engine and has no
live cycle voice. Its tyre audio is active. Vehicles with a supplied simulated
engine/cycle/fuel definition can use the live voice interface; the presentation
test exercises car_sedan_pipes. Pending hypercar turbo/live-voice data remains
owned by the physics library and is not substituted with invented engine audio.

## Verification

```powershell
.\tools\test_presentation.ps1
```

This checks rig placement/orientation/rebasing/free flight, invalid spatial
format/channel rejection, live engine source positions, finite nonzero PCM,
and repeated shutdown. The real-world hypercar smoke also exercises relocation
and world/mode switches. On this machine the endpoint returned 0x80004001 and
the game successfully used Godot fallback. Audible Atmos direction, speaker
mapping, and real headset operation still require compatible hardware checks.

A Vulkan Mobile real-terrain screenshot rendered the car and dashboard.
The hidden-window probe emitted Vulkan surface-capability errors during
startup, so this capture does not establish clean headset rendering.
