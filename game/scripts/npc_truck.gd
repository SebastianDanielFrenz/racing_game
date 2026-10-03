extends Node3D
var simulation: Node
var managed_externally := false
var wheels: Array[Node3D] = []
var spin := 0.0
var model := Node3D.new()

func material(color: Color) -> StandardMaterial3D:
 var m := StandardMaterial3D.new()
 m.albedo_color = color
 m.roughness = 0.75
 return m

func box(at: Vector3, size: Vector3, color: Color) -> void:
 var mesh := BoxMesh.new()
 mesh.size = size
 var item := MeshInstance3D.new()
 item.mesh = mesh
 item.material_override = material(color)
 item.position = at
 model.add_child(item)

func _ready() -> void:
 model.basis = Basis(Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(1, 0, 0))
 add_child(model)
 # Godot local +Z forward, +X left; native pose conversion supplies orientation.
 box(Vector3(0, 0, -1.25), Vector3(2.5, 3.1, 10.5), Color(0.82, 0.84, 0.88))
 box(Vector3(0, -0.3, 5.15), Vector3(2.5, 2.4, 2.7), Color(0.12, 0.3, 0.64))
 box(Vector3(0, 0.2, 6.52), Vector3(2.15, 0.8, 0.035), Color(0.08, 0.16, 0.22))
 box(Vector3(0, -1.25, 0), Vector3(2.25, 0.35, 12.8), Color(0.1, 0.12, 0.14))
 for x in [-1.28, 1.28]:
  for z in [-5.1, -3.9, 3.8, 5.4]:
   var wheel := MeshInstance3D.new()
   var cylinder := CylinderMesh.new()
   cylinder.top_radius = 0.5
   cylinder.bottom_radius = 0.5
   cylinder.height = 0.3
   wheel.mesh = cylinder
   wheel.material_override = material(Color(0.035, 0.035, 0.04))
   wheel.rotation.z = PI / 2
   var hub := Node3D.new()
   hub.position = Vector3(x, -1.7, z)
   hub.add_child(wheel)
   model.add_child(hub)
   wheels.append(hub)
 for x in [-0.95, 0.95]:
  box(Vector3(x, -0.8, -6.52), Vector3(0.3, 0.2, 0.035), Color(0.9, 0.035, 0.025))
  box(Vector3(x, -0.8, 6.52), Vector3(0.35, 0.2, 0.035), Color(1, 0.95, 0.75))

func _process(delta: float) -> void:
 if managed_externally or simulation == null:
  return
 var state: Dictionary = simulation.get_npc_truck_state()
 visible = bool(state.get("active", false))
 if not visible:
  return
 global_transform = simulation.get_body_transform("npc_truck")
 spin += float(state.get("speed_kph", 0)) / 3.6 / 0.5 * delta
 for wheel in wheels:
  wheel.rotation.x = spin
