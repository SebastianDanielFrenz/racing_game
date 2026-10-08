"""Prepare CFD sweep cases and convert solver body-axis loads into runtime maps.
This tool does not run CFD and never fabricates missing results.
"""
import argparse
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path

AXES = ("yaw_deg", "pitch_deg", "speed_m_s", "ride_height_m", "wing_offset_deg", "wing_lift_m")
LOADS = ("fx_n", "fy_n", "fz_n", "mx_nm", "my_nm", "mz_nm")

def finite(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result

def configuration(path):
    config = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if config.get("format") != "rg.aero-bake/1":
        raise ValueError("expected rg.aero-bake/1 configuration")
    axes = config.get("axes", {})
    if any(name not in axes for name in AXES[:3]) or any(name not in AXES for name in axes):
        raise ValueError("axes require yaw_deg,pitch_deg,speed_m_s; optional ride_height_m,wing_offset_deg,wing_lift_m")
    names = [name for name in AXES if name in axes]
    for name in names:
        values = [finite(v, name) for v in axes[name]]
        if not values or any(a >= b for a, b in zip(values, values[1:])):
            raise ValueError(f"{name} must be nonempty, strictly increasing")
        axes[name] = values
    if axes["yaw_deg"][0] != -180 or axes["yaw_deg"][-1] != 180:
        raise ValueError("yaw must include -180 and +180")
    if axes["pitch_deg"][0] != -90 or axes["pitch_deg"][-1] != 90:
        raise ValueError("pitch must cover -90 to +90")
    if axes["speed_m_s"][0] <= 0:
        raise ValueError("solver speeds must be positive; runtime zero-speed force is zero")
    if "ride_height_m" in axes and axes["ride_height_m"][0] < 0:
        raise ValueError("ride height must be nonnegative")
    if "wing_lift_m" in axes and axes["wing_lift_m"][0] < 0:
        raise ValueError("wing lift must be nonnegative")
    for name in ("reference_area_m2", "reference_length_m"):
        config[name] = finite(config[name], name)
        if config[name] <= 0:
            raise ValueError(f"{name} must be positive")
    point = config.get("reference_point_local_m", [0, 0, 0])
    if len(point) != 3:
        raise ValueError("reference point requires three body-axis coordinates")
    config["reference_point_local_m"] = [finite(v, "reference point") for v in point]
    return config, names

def cases(config, names):
    return itertools.product(*(config["axes"][name] for name in names))

def geometry_manifest(config, config_path):
    files = []
    for relative in config.get("geometry_files", []):
        path = (Path(config_path).resolve().parent / relative).resolve()
        files.append({"path": relative, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    if not files:
        raise ValueError("geometry_files is required for provenance")
    return files

def prepare(config_path, output):
    config, names = configuration(config_path)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "cases.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["case_id", *names, "vx_m_s", "vy_m_s", "vz_m_s"])
        for i, values in enumerate(cases(config, names)):
            case = dict(zip(names, values))
            yaw, pitch = math.radians(case["yaw_deg"]), math.radians(case["pitch_deg"])
            speed = case["speed_m_s"]
            # Relative vehicle velocity THROUGH air, not inlet wind direction.
            velocity = [speed*math.cos(pitch)*math.cos(yaw), speed*math.cos(pitch)*math.sin(yaw), speed*math.sin(pitch)]
            writer.writerow([i, *values, *velocity])
    manifest = {"format": "rg.cfd-run/1", "config": config, "geometry": geometry_manifest(config, config_path),
                "case_count": math.prod(len(config["axes"][name]) for name in names),
                "status": "prepared_not_solved", "convention": "ISO body +x forward,+y left,+z up; moments about reference_point_local_m"}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return manifest

def convert(config_path, results_path, output, validated=False):
    config, names = configuration(config_path)
    solver = config.get("solver", {})
    if solver.get("name", "unselected") == "unselected" or solver.get("version", "unselected") == "unselected":
        raise ValueError("record actual solver name and version before importing results")
    rows = {}
    with Path(results_path).open(newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        required = set(names) | set(LOADS) | {"density_kg_m3"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"results CSV requires {sorted(required)}")
        for number, row in enumerate(reader, 2):
            key = tuple(finite(row[name], name) for name in names)
            if any(value not in config["axes"][name] for name, value in zip(names, key)):
                raise ValueError(f"line {number}: case outside sweep axes")
            if key in rows:
                raise ValueError(f"line {number}: duplicate case {key}")
            rho = finite(row["density_kg_m3"], "density")
            if rho <= 0:
                raise ValueError("density must be positive")
            speed = key[names.index("speed_m_s")]
            scale = .5*rho*speed*speed*config["reference_area_m2"]
            loads = [finite(row[name], name) for name in LOADS]
            rows[key] = [load/scale for load in loads[:3]] + [load/(scale*config["reference_length_m"]) for load in loads[3:]]
    ordered = list(cases(config, names))
    missing = [key for key in ordered if key not in rows]
    if missing:
        raise ValueError(f"missing {len(missing)} solver cases; first {missing[0]}")
    # Periodic seam is measured twice, not invented or averaged silently.
    for key in ordered:
        if key[0] == -180:
            opposite = (180.0, *key[1:])
            if any(a != b for a,b in zip(rows[key], rows[opposite])):
                raise ValueError(f"yaw seam mismatch at {key}; reconcile solver convergence first")
    # At +/-90 pitch all yaw labels are the same physical airflow vector.
    for key in ordered:
        if abs(key[1]) == 90:
            canonical = (-180.0, *key[1:])
            if rows[key] != rows[canonical]:
                raise ValueError(f"pitch pole mismatch at {key}; reuse the same converged physical case")
    provenance = dict(config.get("provenance", {}))
    provenance.update({"validated": bool(validated), "solver": solver, "geometry": geometry_manifest(config, config_path),
                       "results_sha256": hashlib.sha256(Path(results_path).read_bytes()).hexdigest()})
    geometry = provenance["geometry"]
    digest = geometry[0]["sha256"] if len(geometry) == 1 else hashlib.sha256("\n".join(item["sha256"] for item in geometry).encode()).hexdigest()
    result = {"schema": "physics_sim.aero_coefficients/1", "frame": "ISO_BODY_X_FORWARD_Y_LEFT_Z_UP",
              "units": "SI_DEGREES_DIMENSIONLESS_COEFFICIENTS", "axes": config["axes"],
              "reference": {"area_m2":config["reference_area_m2"],"lengths_m":[config["reference_length_m"]]*3,"point_local_m":config["reference_point_local_m"]},
              "coefficients": [rows[key] for key in ordered],
              "provenance":{"geometry_sha256":digest,"solver":str(solver["name"])+" "+str(solver["version"]),"source":str(Path(results_path).name)}}
    # Review evidence stays separate from the strict physics JSON schema.
    provenance["geometry_sha256"] = digest
    encoded = json.dumps(result, indent=2, allow_nan=False)+"\n"
    import os
    for item in provenance["geometry"]:
        original = (Path(config_path).resolve().parent / item["path"]).resolve()
        item["path"] = os.path.relpath(original,Path(output).resolve().parent)
    provenance["map_sha256"] = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    Path(output).write_bytes(encoded.encode("utf-8"))
    Path(str(output)+".provenance.json").write_text(json.dumps(provenance,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("config"); prep.add_argument("output")
    conv = sub.add_parser("convert")
    conv.add_argument("config"); conv.add_argument("results"); conv.add_argument("output")
    conv.add_argument("--validated", action="store_true", help="attest completed convergence/geometry review; not an automatic certification")
    args = parser.parse_args()
    try:
        result = prepare(args.config, args.output) if args.command == "prepare" else convert(args.config, args.results, args.output, args.validated)
    except (ValueError, KeyError, OSError, json.JSONDecodeError) as error:
        parser.exit(2, f"aero bake: {error}\n")
    print("Prepared solver cases" if args.command == "prepare" else "Converted solver coefficients", len(result.get("coefficients", [])) or result.get("case_count", 0))
if __name__ == "__main__":
    main()
