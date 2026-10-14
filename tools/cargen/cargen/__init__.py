"""racing_game car generator (tools/cargen).

Builds on physics_sim's stdlib asset generator (external/physics_sim/tools/assetgen),
which is imported read-only from the submodule: nothing here writes into it.
Output goes to racing_game's own data/models/<id>/.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
ASSETGEN = os.path.join(REPO, "external", "physics_sim", "tools", "assetgen")
if ASSETGEN not in sys.path:
    sys.path.insert(0, ASSETGEN)
