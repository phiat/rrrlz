#!/bin/bash
# Build LLVM optimization comparison for all C sources
# Produces: clean IR (.ll), optimized IR, assembly (.s), and binaries at O1-O3/Os/Oz
#
# Usage:
#   bash build_all.sh              # build all (hello.c, dijkstra, astar)
#   bash build_all.sh dijkstra     # build only dijkstra
#   bash build_all.sh astar        # build only astar
#   bash build_all.sh hello        # build only hello

set -e

build_one() {
    local src="$1"
    local dir="$(dirname "$src")"
    local base="$(basename "$src" .c)"
    local prefix="${dir}/${base}"

    echo ""
    echo "============================================"
    echo "  Building: $src"
    echo "============================================"

    echo ""
    echo "=== Step 0: Unoptimized IR (clean, no optnone/noinline) ==="
    clang -S -emit-llvm -O1 -Xclang -disable-llvm-optzns "$src" -o "${prefix}_O0.ll"
    echo "  -> ${prefix}_O0.ll"

    for OPT in O1 O2 O3 Os Oz; do
        echo ""
        echo "=== Optimizing IR at -${OPT} ==="

        # Optimize IR with opt
        opt -S -${OPT} "${prefix}_O0.ll" -o "${prefix}_opt_${OPT}.ll"
        echo "  -> ${prefix}_opt_${OPT}.ll"

        # Generate assembly from optimized IR (PIC for PIE linking)
        llc -relocation-model=pic "${prefix}_opt_${OPT}.ll" -o "${prefix}_opt_${OPT}.s"
        echo "  -> ${prefix}_opt_${OPT}.s"

        # Build final binary from assembly
        clang "${prefix}_opt_${OPT}.s" -o "${prefix}_opt_${OPT}"
        echo "  -> ${prefix}_opt_${OPT}"
    done

    echo ""
    echo "=== Direct clang builds for comparison ==="
    for OPT in O0 O1 O2 O3 Os Oz; do
        clang -${OPT} "$src" -o "${prefix}_clang_${OPT}"
        echo "  -> ${prefix}_clang_${OPT}"
    done
}

# Determine what to build
TARGETS=()
if [ $# -eq 0 ]; then
    TARGETS=("hello/hello.c" "dijkstra/dijkstra.c" "astar/astar.c")
else
    for arg in "$@"; do
        case "$arg" in
            hello)      TARGETS+=("hello/hello.c") ;;
            dijkstra)   TARGETS+=("dijkstra/dijkstra.c") ;;
            astar)      TARGETS+=("astar/astar.c") ;;
            *)          TARGETS+=("$arg") ;;
        esac
    done
fi

# Build each target
for target in "${TARGETS[@]}"; do
    build_one "$target"
done

# Size comparison table
echo ""
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║           SIZE COMPARISON TABLE                  ║"
echo "╚══════════════════════════════════════════════════╝"

for target in "${TARGETS[@]}"; do
    dir="$(dirname "$target")"
    base="$(basename "$target" .c)"
    prefix="${dir}/${base}"

    echo ""
    echo "--- ${target} ---"
    echo ""
    printf "  %-8s %10s %10s %10s\n" "Level" "IR" "Assembly" "Binary"
    printf "  %-8s %10s %10s %10s\n" "-----" "--" "--------" "------"
    for OPT in O1 O2 O3 Os Oz; do
        ir_size=$(wc -c < "${prefix}_opt_${OPT}.ll" 2>/dev/null || echo "?")
        asm_size=$(wc -c < "${prefix}_opt_${OPT}.s" 2>/dev/null || echo "?")
        bin_size=$(wc -c < "${prefix}_opt_${OPT}" 2>/dev/null || echo "?")
        printf "  %-8s %10s %10s %10s\n" "$OPT" "$ir_size" "$asm_size" "$bin_size"
    done
done

echo ""
echo "=== Done. ==="
echo "Compare IR:       diff dijkstra/dijkstra_opt_O1.ll dijkstra/dijkstra_opt_O3.ll"
echo "Compare assembly: diff astar/astar_opt_O1.s astar/astar_opt_O3.s"
echo "Run:              ./dijkstra/dijkstra_opt_O1  or  ./astar/astar_opt_O1"
