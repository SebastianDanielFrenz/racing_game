"""Minimal GLB (binary glTF 2.0) reader/writer with numpy, no third-party glTF library."""
import json
import struct

import numpy as np

_CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_NC = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT2": 4, "MAT3": 9, "MAT4": 16}


def read(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, version, length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2:
        raise ValueError(f"{path}: not a glTF 2 binary")
    off = 12
    js, bin_ = None, b""
    while off < length:
        clen, ctype = struct.unpack_from("<II", data, off)
        chunk = data[off + 8: off + 8 + clen]
        if ctype == 0x4E4F534A:
            js = json.loads(chunk.decode("utf-8"))
        elif ctype == 0x004E4942:
            bin_ = bytes(chunk)
        off += 8 + clen
    return js, bin_


def write(path, js, bin_):
    j = json.dumps(js, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    j += b" " * ((4 - len(j) % 4) % 4)
    b = bin_ + b"\0" * ((4 - len(bin_) % 4) % 4)
    total = 12 + 8 + len(j) + (8 + len(b) if b else 0)
    with open(path, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total))
        f.write(struct.pack("<II", len(j), 0x4E4F534A))
        f.write(j)
        if b:
            f.write(struct.pack("<II", len(b), 0x004E4942))
            f.write(b)


def accessor(js, bin_, idx):
    a = js["accessors"][idx]
    n = _NC[a["type"]]
    dt = np.dtype(_CT[a["componentType"]])
    count = a["count"]
    if "bufferView" not in a:
        out = np.zeros((count, n), dt)
    else:
        bv = js["bufferViews"][a["bufferView"]]
        base = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        stride = bv.get("byteStride", 0) or dt.itemsize * n
        if stride == dt.itemsize * n:
            out = np.frombuffer(bin_, dt, count * n, base).reshape(count, n).copy()
        else:
            raw = np.frombuffer(bin_, np.uint8, (count - 1) * stride + dt.itemsize * n, base)
            out = np.lib.stride_tricks.as_strided(raw, (count, dt.itemsize * n), (stride, 1)).copy().view(dt).reshape(count, n)
    if a.get("normalized") and dt != np.float32:
        out = out.astype(np.float32) / np.iinfo(dt).max
    if "sparse" in a:
        raise NotImplementedError("sparse accessors")
    return out


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def local_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"], float).reshape(4, 4).T
    m = np.eye(4)
    s = np.array(node.get("scale", [1, 1, 1]), float)
    r = quat_to_mat(node.get("rotation", [0, 0, 0, 1]))
    m[:3, :3] = r * s
    m[:3, 3] = node.get("translation", [0, 0, 0])
    return m


def world_matrices(js):
    """node index -> (world 4x4, parent index or None) for every node reachable from the default scene."""
    parent = {}
    for i, n in enumerate(js.get("nodes", [])):
        for c in n.get("children", []):
            parent[c] = i
    out = {}
    scene = js.get("scenes", [{}])[js.get("scene", 0)]
    stack = [(r, np.eye(4)) for r in scene.get("nodes", [])]
    while stack:
        i, pm = stack.pop()
        m = pm @ local_matrix(js["nodes"][i])
        out[i] = m
        for c in js["nodes"][i].get("children", []):
            stack.append((c, m))
    return out, parent
