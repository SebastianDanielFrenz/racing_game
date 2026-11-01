# One design language, for racing_game and the farm game

Owner's words: "Create a unified design language for racing_game and farming sim where the color palette is interchangable."

This language lives in racing_game and the farm game takes it from there. A later Unreal port reuses `data/ui/` and
`rg/ui_theme.h` unchanged and rewrites only the thin Godot layer.

## How to use it

1. In a UI script, `const UiTheme := preload("res://scripts/ui/ui_theme.gd")`. Plain controls need nothing: they already follow the palette.
2. Name a role, never a colour: `UiTheme.text(label, &"warning_text")`, `UiTheme.style(&"surface_panel", &"accent")`, `UiTheme.role(&"gauge_redline")` where you paint in `_draw`.
3. Read sizes from tokens: `UiTheme.token(&"space.m")`, `UiTheme.px(&"type.body", height)` where the root's UI scale does not reach; build screens from `game/scripts/ui/`: `ui_panel`, `ui_gauge`, `ui_tab_strip`, `ui_image_tile`, `ui_key_prompt`.
4. A `_draw` that caches calls `UiTheme.follow(self)` in `_ready` or puts `UiTheme.generation()` in its signature; a `ColorRect` or a stylebox property uses `UiTheme.bind`.
5. A game picks its palette in `data/ui/game.json`; the player changes it under Settings, Interface, at runtime. A new palette is one JSON file in `data/ui/palettes/`, and a new role goes in `data/ui/roles.json` and every palette, with the surfaces it stands on.

## What lives where

| File | What it holds |
|---|---|
| `data/ui/roles.json` | The role contract: every role, what it means, which role pairs must contrast and by how much, what the surfaces stand on. |
| `data/ui/palettes/*.json` | One palette each: every role gets a colour, and nothing else. `racing`, `farm`, `high_contrast`. |
| `data/ui/tokens.json` | The palette-independent half: type scale, spacing grid, radii, strokes, panel treatment, gauge anatomy, tab, tile, key cap, icon sizes, motion timing, state values. |
| `data/ui/game.json` | Which palette this game starts in, and the menu order. |
| `data/ui/colour_allowlist.json` | The scripts and lines that may hold a literal colour, each with its reason. |
| `core/include/rg/ui_theme.h`, `core/src/ui_theme.cpp` | Parser and validator, engine-neutral. |
| `godot_ext/src/rg_ui_theme.*` | Hands the checked data to GDScript as `RgUiTheme`. |
| `game/scripts/ui/ui_theme.gd` | The Godot side of a swap: role and token lookups, repainting the engine's Theme, bindings. |
| `game/scripts/ui/ui_*.gd` | The five components. |
| `tests/unit/test_ui_theme.cpp` | `[ui_theme]`: every shipped palette passes, a broken one is refused with words, the scan for hard-coded colours. |
| `game/scripts/ui_palette_shots.gd` | `--ui-palette-shots <dir>`: walks the real screens of one running game, swaps the palette on each, writes a PNG per screen and palette. |

World colours are not UI and stay where they are: traffic, trucks, terrain, car bodies, lamps, weather in the scene. So does
anything a picture shows as content: a car thumbnail, a paint swatch.

## Decisions

### UI code names roles, a palette gives every role a colour

Chosen: a closed list of 106 roles with one meaning each. Screens name a role; a palette is a flat map from role to colour.

- Roles by job (`surface_panel`, `warning_text`, `gauge_redline`), the choice. Pro: any palette works on any screen, the validator can reason about pairs, the list says what each colour is for. Contra: a new look of an existing thing can need a new role, so the list has to be looked after.
- A small abstract ramp (`primary`, `surface-1` to `surface-5`, `on-primary`). Pro: fewer names, usual in web design systems. Contra: a screen picks a step by eye, so two palettes need the same lightness order to look right, and "the gauge's red zone" has no name to check. A palette then is not interchangeable in the sense the owner asked for.
- One colour per screen, per game. Pro: the least work today. Contra: that is the state this replaces, and it makes the swap impossible.

### Interchangeable means checked

Chosen: a palette that breaks the contract is refused with words, and the game keeps running on the others. The refusal names the role, the ratio, what it was judged on and what was required.

- A check at load that refuses, the choice. Pro: a player or a mod cannot make a screen unreadable, and the words say how to fix the file. Contra: a palette author has to get every pair right, and the racing palette needed small corrections.
- A warning only. Pro: nothing is ever turned away. Contra: "any palette works on any screen" is then only hoped for.
- Checking shots by eye. Pro: judges the whole picture. Contra: does not scale to a palette somebody writes later, and nobody runs it on a player's machine.

### How contrast is judged

Text 4.5:1, large text and graphics 3:1 [C WCAG 2.2 SC 1.4.3 and 1.4.11]. Marks that repeat a reading and carry nothing of their own (minor ticks, off lamps, grid lines) need 1.5:1 [AS].

A translucent surface is judged over the scene that can be behind it: `bright_field`, `bright_sky` and `night` [AS]; the worst of the three counts. These are in `roles.json`, so a game with brighter scenes can add its own.

`sits_on` (decision of this work): a surface that is always drawn on another one is judged on that one, not on the scene. A button in a panel never has the sky directly behind it.

- `sits_on`, the choice. Pro: judges what the player sees, and it is one short map in the contract. Contra: when a screen later puts a button straight on the scene, the check would be too kind; the contract line is then wrong, not the check.
- Every translucent surface over every scene. Pro: simplest, no extra rule. Contra: refused today's racing look 37 times, nearly all for surfaces inside opaque panels, which would have forced thick buttons for no gain.
- Forbid translucent surfaces. Pro: the check gets trivial. Contra: panels over the running game are part of racing's look.

### Colour-blind safety

The brief: do not rely on red against green alone. Two rules:

- Signals that carry no word (lamps, pedal bars, the redline against the ticks) must differ in lightness by 18 L* [AS] in every palette. A badge or a status line says what it means in words and does not need this.
- A palette that claims `colour_blind_safe` must also keep its signal pairs apart under simulated protan, deutan and tritan vision [C matrices Machado, Oliveira, Fernandes 2009, transcribed from memory, no download], at 14 Delta E76 [AS]. `high_contrast` claims it.

Alternatives: requiring the simulated check of every palette (pro: stronger; contra: `racing` is red against green by history and would need a redesign, which the brief rules out), and shape-only signals (pro: independent of colour; contra: rebuilds racing's gauges, which keep their layout).

### Tokens live apart from palettes

Sizes, spacing, strokes, gauge anatomy, motion and state values are one file that no palette touches, so a palette can only change colour. Numbers are design pixels at 1080 lines tall; `ui_scale.gd`'s root scale applies them to Controls, and `UiTheme.px` scales them for `_draw` code and for a viewport the root scale does not reach, such as a headset view. Alternatives: tokens inside each palette (pro: a palette could change the feel; contra: two games drift apart, which is what this replaces) and CSS-like per-screen values (pro: free; contra: no language at all).

Motion tokens have no consumer yet: racing's screens do not animate, and the farm screens will be the first to read them.

### The Godot layer repaints the engine's default Theme in place

Chosen: `ThemeDB.get_default_theme()` is mutated in place, every stock colour mapped to its role by its stock value, and `emit_changed()` repaints every Control.

- The default Theme in place, the choice. Pro: every Control in the game follows with no script change, also those on a CanvasLayer or in a SubViewport; a swap rebuilds nothing. Contra: reaches past the usual pattern, and a stock value that is not in the map keeps its stock colour (which is why the shots are checked).
- A `Theme` resource on each root Control. Pro: the Godot idiom. Contra: stops at CanvasLayers and SubViewports, and this game has many of each.

Colours outside the Theme use `bind` (a property follows the role until its object is freed) and `follow` (a `_draw` redraws on a swap).

### Each game picks its palette in data, the player changes it at runtime

`data/ui/game.json` names the default; the setting `interface.palette` swaps at once, no restart, and `main.gd` applies it through `UiTheme.set_palette`. Alternative: a palette picked at export time (pro: smaller; contra: no colour-blind option for a player).

### Nothing moves in racing_game

racing keeps its screens and layouts and takes only the language. The farm screens build on the five components. `tach_gauge.gd` and `digital_cluster.gd` stay as they are in shape; `ui_gauge.gd` is the same anatomy as a reusable Control, without racing's lamps and pedal bars.

### Components come from the owner's sketches

`docs/farm/ui/hud_sketch.jpg` and `menu_sketch.jpg` on the farm branch ask for a panel with key prompts, a clock box, a round dial with a gear inset, icon tabs with the selected one highlighted, and a scrolling grid of image-only tiles. Hence `ui_panel`, `ui_key_prompt`, `ui_gauge`, `ui_tab_strip`, `ui_image_tile`. A clock box is a `ui_panel` on `surface_widget`.

The selected tab is a filled accent shape with a line under it, so it reads without hue. Focus is a ring in `focus_ring` for keyboard and gamepad, hover a brighter fill for the mouse; tabs and tiles are Buttons, so `ui_left`, `ui_right` and `ui_accept` work with no extra code.

Icons are white line drawings on transparent, `icon.stroke` wide at `icon.medium`, tinted by the component. No icon set is shipped yet.

### The scan for hard-coded colours

`[ui_theme]` fails on any `Color(...)`, `Color.NAME`, `Color8(...)`, `[color=` or `[bgcolor=` in a script under `game/scripts` that is not a test, a shot, or on the allow-list. Every allow-list entry carries its reason, and an entry that no longer matches anything fails too, so the list cannot rot. `Color.TRANSPARENT` passes: it is no colour.

Alternatives: a CI grep (pro: no C++; contra: `tools/ci.ps1` is one more thing to change, and a Catch2 case runs with the rest of `[ui_theme]`), and a Godot-side check at startup (pro: sees real values; contra: finds them only on the screens a run reaches).

## Small corrections to today's racing look

`racing` reproduces today's values, read off the scripts at c40aeea. These are the changes, each asked for by the contrast check or by merging near-duplicates.

Contrast corrections:

| Role | Was | Now |
|---|---|---|
| `surface_hud` | alpha 0.55 | alpha 0.82 |
| `surface_map` | alpha 0.70 | alpha 0.82 |
| `surface_popup_hover` | 0.40 grey | 0.37 grey |
| `gauge_redline` | (.85,.15,.10,.90) | (1,.25,.16,.95) |
| `mode_racing` | (.92,.16,.18) | (1,.30,.28) |
| `cluster_blinker_off` | 0.15 grey | 0.18 grey |
| `badge_good` | (.16,.60,.25,.85) | (.11,.46,.19,.85) |
| `pedal_brake` | (.95,.25,.20,.95) | (1,.30,.22,.95) |
| `chart_share_electric_4` | (.169,.478,.263) | (.20,.55,.30) |
| `chart_grid` | (.30,.33,.38,.45) | (.34,.37,.42,.60) |

Merged near-duplicates:

| Role | Took over |
|---|---|
| `surface_panel` | side panels alpha 0.82, car browser panel alpha 0.90, now 0.85 |
| `surface_scrim` | the shell's shade (.02,.025,.035,.78) and the controls dim (0,0,0,.72) |
| `warning` | cockpit amber (1,.65,.15), fuel low (1,.55,.15), EPB `Color.ORANGE` |
| `good` | the blinker's `Color.GREEN` |
| `danger_text` | advice danger (.95,.32,.28), car browser reject (1,.55,.3) |
| `warning_text` | advice warning (.98,.72,.22) |
| `info_text` | advice info (.62,.78,.95) |
| `accent` | the configurator's focus ring (1,.55,.2) |
| `text_muted` | dyno legend, weather captions, loading footer |
| `surface_widget` | weather menu at alpha 0.95, now opaque like the box |
| `state.disabled_alpha` | the three disabled alphas 0.50, 0.55, 0.65, now 0.55 |

`state.auto_pedal_fraction` 0.37 keeps the clutch bar's faint look under the auto clutch, `state.readout_tint` 0.55 keeps the drive-mode readout's tint: both read off the scripts [M].
