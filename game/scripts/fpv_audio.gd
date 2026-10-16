extends Node
# game/scripts/fpv_audio.gd - the FPV drone's rotor buzz. The synthesis is rg::FpvRotorVoice (RgFpvSound); this node
# only feeds an AudioStreamGenerator with it while the drone flies, a base hum at idle, higher and louder with the
# drone's activity (acceleration, deceleration, turning).

const RATE := 48000
const BUFFER_S := 0.1

var simulation: Node
var master_volume: float = 1.0
var muted: bool = false

var _sound: RgFpvSound
var _player: AudioStreamPlayer
var _capacity: int = 0

func _ready() -> void:
	_sound = RgFpvSound.new()
	var err: String = _sound.load_config(ProjectSettings.globalize_path("res://../data/controls/fpv_drone.json"))
	if err != "":
		push_warning("fpv_audio: " + err)
		_sound = null
		return
	var stream := AudioStreamGenerator.new()
	stream.mix_rate = RATE
	stream.buffer_length = 0.25
	_player = AudioStreamPlayer.new()
	_player.stream = stream
	_player.volume_db = -8.0
	add_child(_player)

func _process(_delta: float) -> void:
	if _sound == null or simulation == null:
		return
	var state: Dictionary = simulation.get_fpv_state()
	if state.is_empty() or muted:
		if _player.playing:
			_player.stop()
			_capacity = 0
		return
	if not _player.playing:
		_player.play()
	_sound.set_activity(float(state["activity"]))
	var playback := _player.get_stream_playback() as AudioStreamGeneratorPlayback
	if playback == null:
		return
	var available := playback.get_frames_available()
	_capacity = maxi(_capacity, available)
	var needed := mini(available, maxi(0, int(RATE * BUFFER_S) - (_capacity - available)))
	if needed <= 0:
		return
	var samples: PackedFloat32Array = _sound.render(needed, master_volume)
	playback.push_buffer(RgTyreSound.channel_to_stereo(samples, 1, 0))
