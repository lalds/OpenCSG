#pragma once

#include "realtimecsg/recovered_geometry.h"
#include <vector>

namespace realtimecsg::recovered {

enum class CSGOperation : std::uint8_t { Additive = 0, Subtractive = 1, Intersecting = 2 };
enum class SurfaceState : std::uint16_t { Hidden = 1, Aligned = 2, ReverseAligned = 3 };

// Portable research input, NOT the original CSGNode memory layout. Branch child
// lists refer to indices in a separate flat array; brush nodes have no children.
struct CategorizationNode {
    std::int32_t firstChild;
    std::int32_t childCount;
    CSGOperation operation;
    std::uint8_t isBrush;
    std::uint8_t infinite;
    std::uint8_t reserved;
};
static_assert(sizeof(CategorizationNode) == 12);

// Recovered transition record shape: node +0, state +4, four successors +6.
struct CategoryTransition {
    std::int32_t brushIndex;
    std::uint16_t state;
    std::uint16_t next[4];
    std::uint16_t reserved;
};
static_assert(sizeof(CategoryTransition) == 16);

struct CategorizationProgram {
    std::vector<CategoryTransition> rows;
    std::uint16_t initialState;
    std::int32_t stateCount;
};

// Operation matrices recovered from csg_operations at Mach-O 0x3c750.
CategoryIndex combineCategories(CSGOperation operation, CategoryIndex left, CategoryIndex right);

// Builds a reduced decision program from recovered category/priority rules. This
// is a new compiler, not a line-for-line reconstruction of GenerateCategorizationTable.
// Current scope: valid trees, at least one additive child per branch, unresolved
// branch hints. Leaf hints: 0 outside, 2 inside, 1/3 unresolved; target is a surface.
// State IDs and row order need not match the native optimizer, but outcomes must.
CategorizationProgram compileCategorization(const CategorizationNode* nodes, std::int32_t nodeCount,
                                            const std::int32_t* children, std::int32_t childCount,
                                            std::int32_t target, const std::uint8_t* hints = nullptr,
                                            std::int32_t root = 0);
SurfaceState evaluateCategorization(const CategoryTransition* rows, std::int32_t rowCount,
                                    std::uint16_t initialState, const std::uint8_t* categories,
                                    std::int32_t categoryCount);

} // namespace realtimecsg::recovered

extern "C" {
// Null output with capacity zero queries the number of rows. -1 invalid/unsupported
// input, -2 insufficient capacity, -3 state limit/allocation failure.
std::int32_t rcsg_recovered_compile_categories(
    const realtimecsg::recovered::CategorizationNode*, std::int32_t,
    const std::int32_t*, std::int32_t, std::int32_t, const std::uint8_t*, std::int32_t,
    realtimecsg::recovered::CategoryTransition*, std::int32_t, std::uint16_t*, std::int32_t*);
std::int32_t rcsg_recovered_evaluate_categories(
    const realtimecsg::recovered::CategoryTransition*, std::int32_t, std::uint16_t,
    const std::uint8_t*, std::int32_t);
std::int32_t rcsg_recovered_combine_categories(std::uint8_t, std::uint8_t, std::uint8_t);
}
