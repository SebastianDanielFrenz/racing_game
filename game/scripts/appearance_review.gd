extends SceneTree
# Render the native fixture export with the shipping material and mesh bindings.
var uploader: Node
func _initialize() -> void:
    run.call_deferred()
func mesh_from_vertices(vertices: Array) -> ArrayMesh:
    var groups: Array = []
    for material in range(12):
        groups.append([PackedVector3Array(),PackedVector3Array(),PackedColorArray(),PackedVector2Array()])
    for i in range(0,vertices.size(),3):
        for j in [i,i+2,i+1]:
            var v: Array = vertices[j]
            var g: Array = groups[int(v[4])]
            g[0].append(Vector3(-v[0][1],v[0][2],-v[0][0]))
            g[1].append(Vector3(-v[1][1],v[1][2],-v[1][0]))
            g[2].append(Color(v[2][0],v[2][1],v[2][2]))
            g[3].append(Vector2(v[3][0],v[3][1]))
    var surfaces: Array = []
    for material in range(groups.size()):
        if not groups[material][0].is_empty():
            surfaces.append({"material":material,"arrays":groups[material]})
    return uploader._mesh(surfaces)
func stage(container: Control, title: String, bridge: bool) -> Node3D:
    var cell := VBoxContainer.new()
    cell.custom_minimum_size = Vector2(315 if not bridge else 420,300 if not bridge else 230)
    container.add_child(cell)
    var label := Label.new()
    label.text = title
    label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
    cell.add_child(label)
    var holder := SubViewportContainer.new()
    holder.stretch = true
    holder.size_flags_vertical = Control.SIZE_EXPAND_FILL
    holder.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    cell.add_child(holder)
    var viewport := SubViewport.new()
    viewport.size = Vector2i(315 if not bridge else 420,270 if not bridge else 200)
    viewport.own_world_3d = true
    viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
    holder.add_child(viewport)
    var world := Node3D.new()
    viewport.add_child(world)
    var environment := Environment.new()
    environment.background_mode = Environment.BG_COLOR
    environment.background_color = Color(.40,.47,.53)
    environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
    environment.ambient_light_color = Color.WHITE
    environment.ambient_light_energy = .7
    var we := WorldEnvironment.new()
    we.environment = environment
    world.add_child(we)
    var sun := DirectionalLight3D.new()
    sun.rotation_degrees = Vector3(-50,-35,0)
    sun.light_energy = 1.6
    world.add_child(sun)
    var ground := MeshInstance3D.new()
    var box := BoxMesh.new()
    box.size = Vector3(25,.15,25)
    ground.mesh = box
    ground.position = Vector3(-4,-.13,-6)
    var mat := StandardMaterial3D.new()
    mat.albedo_color = Color(.31,.34,.29)
    mat.roughness = 1
    ground.material_override = mat
    world.add_child(ground)
    var camera := Camera3D.new()
    camera.projection = Camera3D.PROJECTION_ORTHOGONAL
    camera.size = (23 if title.begins_with("apartments") else 20) if not bridge else 15
    world.add_child(camera)
    var target := Vector3(-4,7 if title.begins_with("apartments") else 5,-6) if not bridge else Vector3(0,.5,-6)
    camera.look_at_from_position(target+Vector3(19,10,23),target)
    return world
func run() -> void:
    var fixture := OS.get_environment("RG_APPEARANCE_FIXTURE")
    var output := OS.get_environment("RG_APPEARANCE_IMAGE")
    if fixture.is_empty() or output.is_empty():
        push_error("Set RG_APPEARANCE_FIXTURE and RG_APPEARANCE_IMAGE")
        quit(1)
        return
    var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(fixture))
    uploader = preload("res://scripts/building_stream.gd").new()
    var panel := PanelContainer.new()
    panel.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
    root.add_child(panel)
    var rows := VBoxContainer.new()
    panel.add_child(rows)
    var heading := Label.new()
    heading.text = "Native facade archetypes and safety railings — inferred appearance fixtures"
    rows.add_child(heading)
    var grid := GridContainer.new()
    grid.columns = 4
    rows.add_child(grid)
    for building: Dictionary in data.buildings:
        var world := stage(grid,str(building.name),false)
        var instance := MeshInstance3D.new()
        instance.mesh = mesh_from_vertices(building.vertices)
        world.add_child(instance)
    var railing_row := HBoxContainer.new()
    rows.add_child(railing_row)
    for railing: Dictionary in data.railings:
        var world := stage(railing_row,str(railing.name)+" safety barrier",true)
        var arrays: Array = []
        arrays.resize(Mesh.ARRAY_MAX)
        var points := PackedVector3Array()
        var normals := PackedVector3Array()
        var indices := PackedInt32Array()
        for i in range(0,railing.positions.size(),3):
            points.append(Vector3(-railing.positions[i+1],railing.positions[i+2],-railing.positions[i]))
            normals.append(Vector3(-railing.normals[i+1],railing.normals[i+2],-railing.normals[i]))
        for i in range(0,railing.indices.size(),3):
            indices.append_array([int(railing.indices[i]),int(railing.indices[i+2]),int(railing.indices[i+1])])
        arrays[Mesh.ARRAY_VERTEX]=points
        arrays[Mesh.ARRAY_NORMAL]=normals
        arrays[Mesh.ARRAY_INDEX]=indices
        var mesh := ArrayMesh.new()
        mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES,arrays)
        var instance := MeshInstance3D.new()
        instance.mesh = mesh
        var material := StandardMaterial3D.new()
        material.albedo_color = Color(.65,.67,.65)
        material.roughness = .8
        instance.material_override = material
        world.add_child(instance)
    for i in range(12):
        await process_frame
    await RenderingServer.frame_post_draw
    var error := root.get_texture().get_image().save_png(output)
    print("APPEARANCE_REVIEW image=%s status=%d" % [output,error])
    uploader.free()
    quit(0 if error == OK else 1)

