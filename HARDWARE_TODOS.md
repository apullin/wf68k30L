# Hardware TODOs

Deferred hardware-compatibility work against full MC68030 behavior.

## HW-001: MMU support
- Status: in progress
- Scope: implement MMU register model and instruction behavior (`PFLUSH`, `PLOAD`, `PMOVE`, `PTEST`), address translation, and related exceptions.
- Why: real MC68030 includes on-chip MMU; current core omits it.
- Plan: see `docs/MMU_PLAN.md` for staged implementation/tasks.

## HW-002: Instruction/data cache support
- Status: in progress
- Scope: implement cache control/state (`CACR`, `CAAR`) and cache-visible bus behavior where architecturally required.
- Why: real MC68030 includes instruction and data caches; current core runs without them.
- Phase plan / progress:
  - Phase 1 (done): MOVEC-visible `CACR`/`CAAR` register surface semantics.
  - Phase 2 (done): minimal instruction-cache fetch path.
    - Direct-mapped lookup/fill model on opcode fetches.
    - `CACR.EI` enables lookup; `CACR.FI` inhibits miss fills.
    - `CACR.CI/CEI` perform immediate invalidation actions (all / selected entry via `CAAR[7:2]`).
  - Phase 3 (completed, long-aligned surface model): data-cache behavior.
    - Read miss fill + subsequent hit service for aligned longword accesses.
    - Write-through hit update and `WA` write-miss allocation for aligned longword accesses.
    - `CACR.CD/CED` invalidate internal data-cache validity state.
  - Phase 4 (completed, surface model): cache/MMU cache-inhibit integration.
    - TT `CI` inhibits cache lookup/fill/allocation on matching accesses.
    - Focused tests cover both I-cache refill inhibition and D-cache WA-allocation inhibition under `CI`.
    - `CIOUT` external signaling behavior remains deferred.
  - Phase 5 (pending): closer 68030 fill policy (line/entry behavior, burst-aware model).

## HW-003: Coprocessor interface support
- Status: deferred
- Scope: add coprocessor instruction/exception interface (`cpBcc`, `cpDBcc`, `cpGEN`, `cpRESTORE`, `cpSAVE`, `cpScc`, `cpTRAPcc`) and F-line/coprocessor exception behavior.
- Why: MC68030 supports external coprocessors; current core omits this interface.

## HW-004: SSW-accurate RTE/bus-fault restoration
- Status: deferred
- Scope: model Special Status Word (SSW) semantics for RTE return paths that recover from defective bus-error stack frames.
- Why: current implementation documents RTE operation without full SSW restoration semantics.
