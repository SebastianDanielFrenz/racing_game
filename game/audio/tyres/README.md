# Tyre audio

- `asphalt_roll_provisional.wav` - PROVISIONAL, AI-generated (owner, 2026-10-18). Sounds like distant traffic roll at a highway
  rest stop, not a close single-tyre recording. Original name `tyre_rolling_on_asph_#1-1792354739356.wav` (stereo 16-bit 48 kHz,
  6.48 s). Mono-mixed and loop-crossfaded at load; played as ONE car-level rolling layer on sealed surfaces
  (`data/controls/tyre_sound.json`, `roll_source` = recording | procedural). The recording speed is unknown ([AS] 50 km/h).
  The procedural per-wheel roll stays in the code as fallback and for unsealed surfaces.

2026-10-19: the recording is no longer the whole layer. Its level follows data/controls/tyre_sound.json `roll` (35 dB/decade, no cap below 500 km/h) and its playback rate follows (v/50 km/h)^0.35, so it gets louder and rises in pitch with wheel speed; a procedural hiss and a tread tone (frequency = tread speed / pitch) ride on top. Wind noise (data/controls/wind_noise.json) is a separate layer driven by airspeed.
