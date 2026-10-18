#include "realtimecsg/recovered_brush_edge_cleanup.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace realtimecsg::recovered {
namespace {
struct OwnedGroup {
    BrushEdgeGroupView view;
    BrushEdgeTriangleList source;
    BrushEdgeConnectionList connection;
    std::vector<BrushTriangleEdge> refs, links;
    std::vector<std::uint32_t> edges;
};
// Scratch records retain original buffers with their groups through swaps.
// Commit checks all capacities before writing any caller-owned state.
struct Work {
    std::vector<TriangleRecord> triangles;
    std::vector<OwnedGroup> groups, removed;
    Work(const TriangleMeshView& mesh, const BrushEdgeGroupView* views,
         const BrushEdgeTriangleList* sources, const BrushEdgeConnectionList* connections, int count) {
        if (count < 0 || (count && (!views || !sources || !connections)) || mesh.triangleCount < 0
            || mesh.triangleCount > mesh.triangleCapacity || mesh.triangleCount > (1<<30)
            || (!mesh.triangles && mesh.triangleCount)) throw std::invalid_argument("Invalid brush-edge views");
        if (mesh.triangleCount) triangles.assign(mesh.triangles,mesh.triangles+mesh.triangleCount);
        groups.reserve(count); removed.reserve(count);
        for (int i = 0; i < count; ++i) {
            const auto& a = views[i]; const auto& b = sources[i]; const auto& c = connections[i];
            if (a.surfaceRefCount < 0 || a.surfaceRefCount > a.surfaceRefCapacity || (!a.surfaceRefs && a.surfaceRefCapacity)
                || b.count < 0 || b.count > b.capacity || (!b.edges && b.capacity)
                || c.count < 0 || c.count > c.capacity || (!c.connections && c.capacity))
                throw std::invalid_argument("Invalid brush-edge storage");
            OwnedGroup group{a,b,c,{},{},{}};
            if (a.surfaceRefCount) group.refs.assign(a.surfaceRefs,a.surfaceRefs+a.surfaceRefCount);
            if (b.count) group.edges.assign(b.edges,b.edges+b.count);
            if (c.count) group.links.assign(c.connections,c.connections+c.count);
            groups.push_back(std::move(group));
        }
    }
    void pop() { removed.push_back(std::move(groups.back())); groups.pop_back(); }
    int merge(std::array<int,2>& pair) {
        std::vector<int> indices(pair.begin(),pair.end());
        const int result = mergeMany(indices);
        std::copy_n(indices.begin(),2,pair.begin());
        return result;
    }
    int mergeMany(std::vector<int>& indices) {
        const int size = static_cast<int>(groups.size());
        if (indices.empty()) throw std::invalid_argument("Merge index array must be nonempty");
        for (int id : indices) if (id < -1 || id >= size) throw std::invalid_argument("Invalid merge group index");
        std::sort(indices.begin(),indices.end(),[](int a,int b) { return a > b; });
        if (indices[0] == -1) return -1;
        int active = static_cast<int>(indices.size());
        while (indices[active-1] == -1) --active;
        if (indices[0] == indices[active-1]) return indices[0];
        std::reverse(indices.begin(),indices.begin()+active);
        // Native removes duplicates by filling from the active tail, restarting
        // the search after each replacement. Preserve unused index-array bytes.
        for (int i = 0; i < active; ++i) {
            int j = i+1;
            while (j < active) {
                if (indices[j] == indices[i]) { indices[j] = indices[--active]; j = i+1; }
                else ++j;
            }
        }
        const int root = indices[0];
        std::vector<int> mapping(static_cast<std::size_t>(size));
        for (int i = 0; i < size; ++i) mapping[i] = i;
        for (int i = 1; i < active; ++i) {
            const int other = indices[i];
            auto& a = groups[root]; auto& b = groups[other];
            for (const auto& ref : a.refs) for (auto& candidate : b.refs)
                if (ref.brushIndex == candidate.brushIndex && ref.edge == candidate.edge) {
                    candidate = {-1,kInvalidTriangleEdge}; break;
                }
            for (const auto& ref : b.refs) if (ref.brushIndex != -1) a.refs.push_back(ref);
            b.refs.clear(); mapping[other] = root;
        }
        for (int i = 1; i < active; ++i) {
            int pending = active;
            while (active > i) {
                const int candidate = indices[--active];
                if (candidate != static_cast<int>(groups.size())-1) break;
                mapping[candidate] = -1; pop(); --pending;
            }
            if (pending == i) break;
            const int other = indices[i];
            if (other >= static_cast<int>(groups.size()))
                throw std::invalid_argument("Native removal order would access a destroyed group");
            mapping[groups.size()-1] = other;
            std::swap(groups[other],groups.back()); pop();
            active = pending;
        }
        for (auto& triangle : triangles) for (auto& id : triangle.edgeSource) {
            if (id == -1) continue;
            id = id >= 0 && id < size ? mapping[id] : -1;
        }
        return root;
    }
    std::uint32_t twin(int p, unsigned c) const {
        const auto edge = triangles[p].twins[c];
        if ((edge&3u) == 3u || edge>>2 >= triangles.size()
            || triangles[edge>>2].twins[edge&3u] != (static_cast<unsigned>(p)*4+c))
            throw std::invalid_argument("Cleanup needs reciprocal tagged twins");
        return edge;
    }
    void cleanup() {
        for (int p = 0; p < static_cast<int>(triangles.size()); ++p) for (unsigned c = 0; c < 3; ++c) {
            const int id = triangles[p].edgeSource[c];
            if (id == -1) continue;
            const auto e = twin(p,c); const int other = triangles[e>>2].edgeSource[e&3u];
            if (id == other) continue;
            int replacement = id;
            if (other != -1) {
                triangles[p].edgeSource[c] = triangles[e>>2].edgeSource[e&3u] = -1;
                std::array<int,2> pair{{id,other}};
                replacement = merge(pair);
            }
            triangles[p].edgeSource[c] = triangles[e>>2].edgeSource[e&3u] = replacement;
        }
        for (int p = 0; p < static_cast<int>(triangles.size()); ++p) {
            if (!(triangles[p].flags&8u)) continue;
            for (unsigned c = 0; c < 3; ++c) if (triangles[p].edgeSource[c] != -1) {
                const auto e = twin(p,c);
                if (triangles[e>>2].flags&8u) triangles[p].edgeSource[c] = -1;
            }
        }
        const int size = static_cast<int>(groups.size());
        std::vector<int> counts(static_cast<std::size_t>(size),0), mapping(static_cast<std::size_t>(size));
        for (int i = 0; i < size; ++i) mapping[i] = i;
        for (auto& triangle : triangles) for (auto& id : triangle.edgeSource) {
            if (id < 0 || id >= size) id = -1;
            else {
                if (counts[id] == std::numeric_limits<int>::max()) throw std::length_error("Too many source handles");
                ++counts[id];
            }
        }
        for (int i = 0; i < static_cast<int>(groups.size()); ++i)
            while (i < static_cast<int>(groups.size()) && counts[i] == 0) {
                const int last = static_cast<int>(groups.size())-1;
                mapping[i] = -1; mapping[last] = i;
                std::swap(groups[i],groups[last]); std::swap(counts[i],counts[last]); pop();
            }
        for (int p = 0; p < static_cast<int>(triangles.size()); ++p) for (unsigned c = 0; c < 3; ++c) {
            auto& id = triangles[p].edgeSource[c];
            if (id == -1) continue;
            id = mapping[id];
            if (id != -1) groups[id].edges.push_back(static_cast<unsigned>(p)*4+c);
        }
    }
    void commit(TriangleMeshView& mesh, BrushEdgeGroupView* views, BrushEdgeTriangleList* sources,
                BrushEdgeConnectionList* connections, int& count) const {
        for (const auto& group : groups)
            if (group.refs.size() > static_cast<std::size_t>(group.view.surfaceRefCapacity)
                || group.edges.size() > static_cast<std::size_t>(group.source.capacity)
                || group.links.size() > static_cast<std::size_t>(group.connection.capacity))
                throw std::length_error("Brush-edge buffer capacity exhausted");
        for (std::size_t i = 0; i < groups.size(); ++i) {
            const auto& g = groups[i];
            if (!g.refs.empty()) std::memcpy(g.view.surfaceRefs,g.refs.data(),g.refs.size()*sizeof(BrushTriangleEdge));
            if (!g.edges.empty()) std::memcpy(g.source.edges,g.edges.data(),g.edges.size()*sizeof(std::uint32_t));
            if (!g.links.empty()) std::memcpy(g.connection.connections,g.links.data(),g.links.size()*sizeof(BrushTriangleEdge));
            views[i] = g.view; views[i].surfaceRefCount = static_cast<int>(g.refs.size());
            sources[i] = g.source; sources[i].count = static_cast<int>(g.edges.size());
            connections[i] = g.connection; connections[i].count = static_cast<int>(g.links.size());
        }
        for (std::size_t i = 0; i < removed.size(); ++i) {
            const int slot = count-1-static_cast<int>(i); const auto& g = removed[i];
            views[slot] = g.view; views[slot].surfaceRefCount = 0;
            sources[slot] = g.source; sources[slot].count = 0;
            connections[slot] = g.connection; connections[slot].count = 0;
        }
        if (!triangles.empty()) std::memcpy(mesh.triangles,triangles.data(),triangles.size()*sizeof(TriangleRecord));
        count = static_cast<int>(groups.size());
    }
};
}
void cleanUpBrushEdges(TriangleMeshView& mesh, BrushEdgeGroupView* groups,
    BrushEdgeTriangleList* sources, BrushEdgeConnectionList* connections, int& count) {
    Work work(mesh,groups,sources,connections,count);
    work.cleanup(); work.commit(mesh,groups,sources,connections,count);
}
int mergeBrushIndexPair(TriangleMeshView& mesh, BrushEdgeGroupView* groups,
    BrushEdgeTriangleList* sources, BrushEdgeConnectionList* connections, int& count, int* indices) {
    if (!indices) throw std::invalid_argument("Missing merge pair");
    Work work(mesh,groups,sources,connections,count);
    std::array<int,2> pair{{indices[0],indices[1]}};
    const int result = work.merge(pair);
    work.commit(mesh,groups,sources,connections,count);
    indices[0] = pair[0]; indices[1] = pair[1]; return result;
}
int mergeBrushIndices(TriangleMeshView& mesh, BrushEdgeGroupView* groups,
    BrushEdgeTriangleList* sources, BrushEdgeConnectionList* connections, int& count, int* indices, int indexCount) {
    if (!indices || indexCount <= 0) throw std::invalid_argument("Missing merge indices");
    Work work(mesh,groups,sources,connections,count);
    std::vector<int> copy(indices,indices+indexCount);
    const int result = work.mergeMany(copy);
    work.commit(mesh,groups,sources,connections,count);
    std::copy(copy.begin(),copy.end(),indices); return result;
}
}
extern "C" std::int32_t rcsg_recovered_cleanup_brush_edges(realtimecsg::recovered::TriangleMeshView* mesh,
    realtimecsg::recovered::BrushEdgeGroupView* groups, realtimecsg::recovered::BrushEdgeTriangleList* sources,
    realtimecsg::recovered::BrushEdgeConnectionList* connections, std::int32_t* count) {
    if (!mesh || !count) return -1;
    try { realtimecsg::recovered::cleanUpBrushEdges(*mesh,groups,sources,connections,*count); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
extern "C" std::int32_t rcsg_recovered_merge_brush_pair(realtimecsg::recovered::TriangleMeshView* mesh,
    realtimecsg::recovered::BrushEdgeGroupView* groups, realtimecsg::recovered::BrushEdgeTriangleList* sources,
    realtimecsg::recovered::BrushEdgeConnectionList* connections, std::int32_t* count, std::int32_t* indices, std::int32_t* result) {
    if (!mesh || !count || !indices || !result) return -1;
    try { *result = realtimecsg::recovered::mergeBrushIndexPair(*mesh,groups,sources,connections,*count,indices); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
extern "C" std::int32_t rcsg_recovered_merge_brush_indices(realtimecsg::recovered::TriangleMeshView* mesh,
    realtimecsg::recovered::BrushEdgeGroupView* groups, realtimecsg::recovered::BrushEdgeTriangleList* sources,
    realtimecsg::recovered::BrushEdgeConnectionList* connections, std::int32_t* count,
    std::int32_t* indices, std::int32_t indexCount, std::int32_t* result) {
    if (!mesh || !count || !indices || !result) return -1;
    try { *result = realtimecsg::recovered::mergeBrushIndices(*mesh,groups,sources,connections,*count,indices,indexCount); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
