"""Exterior triangle bookkeeping for the CFD flow selection sidecar.

The generator knows which triangles are the outer surface the air sees: it
records, per (node, material) primitive, the index ranges emitted between
checkpoints. `mark` adds everything emitted since the last checkpoint to the
selection; `skip` moves the checkpoint without selecting (inner skins, cabin,
engine). The result is physics_sim.visual_flow_selection/1, the format the
hypercar's car_hyper.flow.json uses, pinned to the GLB's sha256 by build.py.
"""
from assetgen.vehicles.hyper_shell import clean_degenerate


class FlowRecorder:
    def __init__(self, model):
        self.model = model
        self.ranges = []      # (key, first_index, index_count, label)
        self.labels = []
        self.excluded = []
        self._snap = self._counts()

    def _counts(self):
        return {key: len(p.idx) for key, p in self.model.mb.prims.items()}

    def _close(self):
        removed = clean_degenerate(self.model.mb)
        if removed and self._touches_marked(removed):
            raise ValueError("degenerate cleanup reached already-selected triangles")

    def _touches_marked(self, _removed):
        # Selected ranges are only recorded after a cleanup, so later cleanups can only
        # remove triangles emitted after the checkpoint; verify instead of assume.
        for key, start, count, _ in self.ranges:
            if len(self.model.mb.prims[key].idx) < start + count:
                return True
        return False

    def mark_all_since_start(self, label):
        self.mark(label)

    def mark(self, label):
        self._close()
        now = self._counts()
        for key, n in now.items():
            s = self._snap.get(key, 0)
            if n > s:
                self.ranges.append((key, s, n - s, label))
        self.labels.append(label)
        self._snap = now

    def skip(self, label):
        self._close()
        self._snap = self._counts()
        self.excluded.append(label)

    def selection(self, limitations):
        m = self.model
        merged = {}
        order = []
        for key, start, count, _ in self.ranges:
            if key not in merged:
                merged[key] = []
                order.append(key)
            runs = merged[key]
            if runs and runs[-1][0] + runs[-1][1] == start:
                runs[-1][1] += count
            else:
                runs.append([start, count])
        ranges = []
        for key in order:
            for start, count in merged[key]:
                ranges.append({"node": key[0] or m.id, "material": key[1],
                               "first_triangle": start // 3, "triangle_count": count // 3})
        return {
            "format": "physics_sim.visual_flow_selection/1",
            "frame": "ISO x forward,y left,z up; rig ground origin, not chassis COM",
            "status": "visual_surface_selection_not_fluid_solid",
            "cfd_ready": False,
            "ranges": ranges,
            "selected": self.labels,
            "excluded": "; ".join(self.excluded),
            "limitations": limitations,
        }
