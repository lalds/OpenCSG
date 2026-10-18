#include "realtimecsg/recovered_categorization.h"

#include <algorithm>
#include <array>
#include <climits>
#include <functional>
#include <map>
#include <new>
#include <set>
#include <stdexcept>
#include <tuple>

namespace realtimecsg::recovered {

CategoryIndex combineCategories(CSGOperation operation, CategoryIndex left, CategoryIndex right) {
    const auto a = static_cast<unsigned>(left), b = static_cast<unsigned>(right);
    if (a > 3 || b > 3) throw std::invalid_argument("Invalid category");
    switch (operation) {
    case CSGOperation::Additive: return static_cast<CategoryIndex>(a & b);
    case CSGOperation::Subtractive: return static_cast<CategoryIndex>(a | ((~b) & 3u));
    case CSGOperation::Intersecting: return static_cast<CategoryIndex>(a | b);
    }
    throw std::invalid_argument("Invalid CSG operation");
}

namespace {
struct Decision { int position; std::array<int, 4> next; };

// Four-way ordered decision diagrams. References 0..3 are terminal categories.
// Canonicalization and memoized apply avoid enumerating all 4^N assignments.
struct Diagram {
    std::vector<Decision> decisions;
    std::map<std::array<int, 5>, int> unique;
    std::map<std::tuple<unsigned, int, int>, int> applied;

    int intern(int position, const std::array<int, 4>& next) {
        if (std::all_of(next.begin(), next.end(), [&](int n) { return n == next[0]; })) return next[0];
        const std::array<int, 5> key{position, next[0], next[1], next[2], next[3]};
        const auto old = unique.find(key);
        if (old != unique.end()) return old->second;
        if (decisions.size() >= 32763) throw std::length_error("Categorization state limit");
        const int reference = static_cast<int>(decisions.size()) + 4;
        decisions.push_back({position, next});
        unique.emplace(key, reference);
        return reference;
    }

    int combine(CSGOperation operation, int left, int right) {
        if (left < 4 && right < 4)
            return static_cast<int>(combineCategories(operation, static_cast<CategoryIndex>(left),
                                                       static_cast<CategoryIndex>(right)));
        const auto key = std::make_tuple(static_cast<unsigned>(operation), left, right);
        const auto cached = applied.find(key);
        if (cached != applied.end()) return cached->second;
        const int a = left < 4 ? INT_MAX : decisions[left - 4].position;
        const int b = right < 4 ? INT_MAX : decisions[right - 4].position;
        const int position = std::min(a, b);
        std::array<int, 4> children{};
        // Copy branches before recursion: intern() can reallocate decisions.
        const auto leftNext = left < 4 ? std::array<int, 4>{left, left, left, left} : decisions[left - 4].next;
        const auto rightNext = right < 4 ? std::array<int, 4>{right, right, right, right} : decisions[right - 4].next;
        for (int c = 0; c < 4; ++c)
            children[c] = combine(operation, a == position ? leftNext[c] : left,
                                   b == position ? rightNext[c] : right);
        const int result = intern(position, children);
        applied.emplace(key, result);
        return result;
    }
};
} // namespace

CategorizationProgram compileCategorization(const CategorizationNode* nodes, int nodeCount,
                                            const int* children, int childCount, int target,
                                            const std::uint8_t* hints, int root) {
    if (!nodes || nodeCount <= 0 || nodeCount > 4096 || childCount < 0
        || (childCount && !children) || target < 0 || target >= nodeCount
        || root < 0 || root >= nodeCount || nodes[root].isBrush || !nodes[target].isBrush)
        throw std::invalid_argument("Invalid categorization input");
    std::vector<int> color(nodeCount, 0), order, position(nodeCount, -1);
    std::function<void(int, int)> visit = [&](int index, int depth) {
        if (index < 0 || index >= nodeCount || depth > 256 || color[index])
            throw std::invalid_argument("Invalid tree: cycle, shared child, index or depth");
        color[index] = 1;
        const auto& node = nodes[index];
        if (static_cast<unsigned>(node.operation) > 2 || node.isBrush > 1 || node.infinite > 1
            || (hints && hints[index] > 3)) throw std::invalid_argument("Invalid node flags");
        if (node.isBrush) {
            if (node.childCount != 0) throw std::invalid_argument("Brush has children");
            position[index] = static_cast<int>(order.size());
            order.push_back(index);
        } else {
            if (node.infinite || node.firstChild < 0 || node.childCount <= 0
                || node.firstChild > childCount || node.childCount > childCount - node.firstChild)
                throw std::invalid_argument("Empty or invalid branch not yet supported");
            if (hints && hints[index] != 1)
                throw std::invalid_argument("Resolved branch hints not yet supported");
            bool additive = false;
            for (int i = 0; i < node.childCount; ++i) {
                const int child = children[node.firstChild + i];
                visit(child, depth + 1);
                additive |= nodes[child].operation == CSGOperation::Additive;
            }
            if (!additive) throw std::invalid_argument("Branch without additive child not yet supported");
        }
        color[index] = 2;
    };
    visit(root, 0);
    if (!color[target]) throw std::invalid_argument("Target is outside the tree");
    if (nodes[target].infinite) return {{}, 1, 4};

    Diagram categories;
    std::function<int(int)> build = [&](int index) {
        const auto& node = nodes[index];
        if (node.isBrush) {
            if (node.infinite) return 0; // An infinite brush occupies both sides.
            if (index == target) return 1; // Subject's own oriented surface.
            const unsigned hint = hints ? hints[index] : 1;
            if (hint == 0) return 3;
            if (hint == 2) return 0;
            // Recovered leaf handling at 0x257f0/0x25aec: coincident surfaces
            // depend on whether the subject is earlier or later in tree order.
            const int aligned = position[index] < position[target] ? 3 : 0;
            return categories.intern(position[index], {0, aligned, 0, 3});
        }
        int accumulator = 3; // Empty space before the first additive child.
        bool started = false;
        for (int i = 0; i < node.childCount; ++i) {
            const int child = children[node.firstChild + i];
            if (!started && nodes[child].operation != CSGOperation::Additive) continue;
            started = true;
            // The original bit-0x40 path replaces the accumulator with Inside
            // independently of the stored operation (0x2599e). Preserve that
            // observed special case rather than assuming ordinary Boolean algebra.
            accumulator = nodes[child].isBrush && nodes[child].infinite ? 0
                          : categories.combine(nodes[child].operation, accumulator, build(child));
        }
        return accumulator;
    };
    const int categoryRoot = build(root);

    Diagram visible;
    std::map<int, int> converted;
    std::function<int(int)> convert = [&](int reference) {
        if (reference < 4) return std::array<int, 4>{1, 2, 3, 1}[reference];
        const auto cached = converted.find(reference);
        if (cached != converted.end()) return cached->second;
        const auto decision = categories.decisions[reference - 4];
        std::array<int, 4> next{};
        for (int i = 0; i < 4; ++i) next[i] = convert(decision.next[i]);
        const int result = visible.intern(decision.position, next);
        converted.emplace(reference, result);
        return result;
    };
    const int visibleRoot = convert(categoryRoot);
    if (visibleRoot < 4) return {{}, static_cast<std::uint16_t>(visibleRoot), 4};

    CategorizationProgram program{{}, 0, 4};
    std::map<int, std::uint16_t> stateIds{{visibleRoot, 0}};
    std::set<int> emitted;
    std::function<std::uint16_t(int)> emit = [&](int reference) -> std::uint16_t {
        if (reference < 4) return static_cast<std::uint16_t>(reference);
        auto found = stateIds.find(reference);
        if (found == stateIds.end()) {
            if (program.stateCount >= 32767) throw std::length_error("Categorization state limit");
            found = stateIds.emplace(reference, static_cast<std::uint16_t>(program.stateCount++)).first;
        }
        const auto id = found->second;
        if (!emitted.insert(reference).second) return id;
        const auto decision = visible.decisions[reference - 4];
        CategoryTransition row{order[decision.position], id, {}, 0};
        for (int i = 0; i < 4; ++i) row.next[i] = emit(decision.next[i]);
        program.rows.push_back(row);
        return id;
    };
    emit(visibleRoot);
    std::sort(program.rows.begin(), program.rows.end(), [&](const auto& a, const auto& b) {
        return std::tie(position[a.brushIndex], a.state) < std::tie(position[b.brushIndex], b.state);
    });
    return program;
}

SurfaceState evaluateCategorization(const CategoryTransition* rows, int rowCount,
                                    std::uint16_t state, const std::uint8_t* categories, int count) {
    if (rowCount < 0 || (rowCount && !rows) || count < 0 || (count && !categories))
        throw std::invalid_argument("Invalid category program");
    for (int i = 0; i < rowCount;) {
        const int brush = rows[i].brushIndex;
        if (brush < 0 || brush >= count || categories[brush] > 3)
            throw std::invalid_argument("Missing brush category");
        int end = i + 1;
        while (end < rowCount && rows[end].brushIndex == brush) ++end;
        for (int j = i; j < end; ++j) {
            if (rows[j].state == state) {
                state = rows[j].next[categories[brush]];
                break;
            }
        }
        i = end;
    }
    if (state < 1 || state > 3) throw std::invalid_argument("Nonterminal category state");
    return static_cast<SurfaceState>(state);
}
} // namespace realtimecsg::recovered

extern "C" std::int32_t rcsg_recovered_combine_categories(std::uint8_t operation,
                                                          std::uint8_t a, std::uint8_t b) {
    try {
        return static_cast<int>(realtimecsg::recovered::combineCategories(
            static_cast<realtimecsg::recovered::CSGOperation>(operation),
            static_cast<realtimecsg::recovered::CategoryIndex>(a),
            static_cast<realtimecsg::recovered::CategoryIndex>(b)));
    } catch (const std::invalid_argument&) { return -1; }
}

extern "C" std::int32_t rcsg_recovered_compile_categories(
    const realtimecsg::recovered::CategorizationNode* nodes, int count,
    const int* children, int childrenCount, int target, const std::uint8_t* hints, int root,
    realtimecsg::recovered::CategoryTransition* output, int capacity,
    std::uint16_t* initialState, int* stateCount) {
    try {
        if (capacity < 0 || !initialState || !stateCount) return -1;
        const auto program = realtimecsg::recovered::compileCategorization(
            nodes, count, children, childrenCount, target, hints, root);
        *initialState = program.initialState;
        *stateCount = program.stateCount;
        if (!output) return capacity == 0 ? static_cast<int>(program.rows.size()) : -1;
        if (program.rows.size() > static_cast<std::size_t>(capacity)) return -2;
        std::copy(program.rows.begin(), program.rows.end(), output);
        return static_cast<int>(program.rows.size());
    } catch (const std::invalid_argument&) { return -1; }
      catch (const std::length_error&) { return -3; }
      catch (const std::bad_alloc&) { return -3; }
}

extern "C" std::int32_t rcsg_recovered_evaluate_categories(
    const realtimecsg::recovered::CategoryTransition* rows, int count, std::uint16_t initial,
    const std::uint8_t* categories, int categoryCount) {
    try { return static_cast<int>(realtimecsg::recovered::evaluateCategorization(rows, count, initial, categories, categoryCount)); }
    catch (const std::invalid_argument&) { return -1; }
}
