"""CarBody whose every material region is a closed thin shell.

Same visible surface as assetgen's CarBody; the difference is what sits behind
it. Each node's skin (body, every door, the hood, the hatch, the glass) gets an
inner skin 18 mm inside it with stitched perimeter walls, using the hypercar's
own shell helper (assetgen.vehicles.hyper_shell.solidify, imported read-only).
Draped patches (lamps, grilles, plates) get a backing and a skirt, so they are
closed too. Wheel arches and intentional openings stay open; nothing is capped
across a mouth.

The exterior triangle ranges (the outward visual surface before the inner
skins) are recorded for the CFD flow selection (cargen.flow).
"""
from . import ASSETGEN  # noqa: F401  (sys.path setup)
from assetgen.carbody import CarBody
from assetgen import mesh as M
from assetgen.vehicles.hyper_shell import skin_part, clean_degenerate

SHELL_THICKNESS = 0.018
PANEL_INSET = 0.001
SEAM_HALF = 0.004      # half width of a shut line (8 mm gap)
SEAM_CLEAR = 0.022     # regular stations closer than this to a seam are dropped
SEAM_BAND = 0.006      # |face centre x - seam| below this is the shut line


class ClosedCarBody(CarBody):
    def __init__(self, model, *, flow, moving_parts=(), seams=(), **kw):
        """seams: x positions of vertical shut lines. Each gets a station pair at x +- SEAM_HALF (any regular
        station within SEAM_CLEAR of it is dropped), so a narrow gap band |c.x - seam| < SEAM_BAND selects one
        8 mm column of faces, whatever the regular station step."""
        kw.setdefault("step", 0.03)
        kw.setdefault("around", 112)
        super().__init__(model, **kw)
        self.flow = flow
        self.moving_parts = set(moving_parts)
        self.seams = sorted(seams)

    def build(self, classify_lower=None, classify_green=None):
        import assetgen.carbody as cb
        orig = cb.stations
        seams = self.seams

        def stations(x0, x1, step, end_r):
            xs = [x for x in orig(x0, x1, step, end_r)
                  if all(abs(x - q) >= SEAM_CLEAR for q in seams) or x in (x0, x1)]
            for q in seams:
                for x in (q - SEAM_HALF, q + SEAM_HALF):
                    if x0 < x < x1:
                        xs.append(round(x, 6))
            return sorted(set(xs))

        cb.stations = stations
        try:
            super().build(classify_lower, classify_green)
        finally:
            cb.stations = orig

    @staticmethod
    def on_seam(x, q):
        return abs(x - q) < SEAM_BAND

    def _inner_skins(self):
        mb = self.model.mb
        clean_degenerate(mb)
        self.flow.mark_all_since_start("body outer skin")
        for part in list(mb.parts()):
            skin_part(mb, part, thickness=SHELL_THICKNESS,
                      inset=PANEL_INSET if part in self.moving_parts else 0.0)
        self.flow.skip("inner skins and perimeter returns")

    def patch(self, outline, mat, face, part="", offset=0.004, rings=3, skirt_mat=None, skirt=0.004):
        """Drape a lens/grille/plate like CarBody.patch, with a backing so it is closed."""
        mb = self.model.mb

        def place(a, b, off):
            if face in ("front", "rear"):
                e = 1 if face == "front" else -1
                return (self.surface_x(a, b, e) + e * off, a, b)
            side = 1 if face == "left" else -1
            return (a, side * (self.surface_y(a, b) + off), b)

        want = {"front": (1, 0, 0), "rear": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0)}[face]
        if M.dot(M.newell([place(a, b, 0) for a, b in outline]), want) < 0:
            outline = list(reversed(outline))
        ca = sum(p[0] for p in outline) / len(outline)
        cb = sum(p[1] for p in outline) / len(outline)

        def rows(off):
            return [[place(ca + (a - ca) * k / rings, cb + (b - cb) * k / rings, off) for a, b in outline]
                    for k in range(rings + 1)]

        front = rows(offset)
        recs = mb.grid(front, mat, part, closed_v=True)
        back = rows(offset - skirt - 0.004)
        mb.grid([back[-1], front[-1]], skirt_mat or mat, part, closed_v=True, flip=True)
        mb.grid(back, skirt_mat or mat, part, closed_v=True, flip=True)
        clean_degenerate(mb)
        return recs
