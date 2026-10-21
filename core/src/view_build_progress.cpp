#include "rg/view_build_progress.h"

#include "g2m/world/view_streamer.h"

#include <utility>
#include <vector>

namespace rg {

void attach_view_build_progress(TerrainViewSource& source, const std::shared_ptr<ViewBuildProgress>& progress) {
    if (!progress) return;
    g2m::world::ChunkColourFn inner = std::move(source.colour);
    source.colour = [inner = std::move(inner), progress](g2m::world::ViewChunk& chunk) {
        if (inner) inner(chunk);
        progress->done.fetch_add(1, std::memory_order_relaxed);
    };
}

std::uint32_t announce_view_build_total(const TerrainViewSource& source, double x, double y,
                                        ViewBuildProgress& progress) {
    std::vector<g2m::mesh::ChunkKey> keys;
    g2m::world::select_view_keys(x, y, source.params, source.e0, source.n0, keys);
    const auto n = static_cast<std::uint32_t>(keys.size());
    progress.total.store(n, std::memory_order_relaxed);
    return n;
}

} // namespace rg
