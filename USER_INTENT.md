# User Intent (PI_DiskCutting)

- If there are unexpected file changes while the Coding Agent is working, it is the user who is editing the codebase alongside the Coding Agent, and the Coding Agent should (a) think to understand what the user is doing, or (b) ask the user if unsure about anything.

---

## 1. Communication style

- Write in plain, highly readable language; lead with context (what/why) before details.
- Prefer simple phrasing over jargon, but do not sacrifice clarity for simplicity. Avoid dense wording, invented terms, or unnecessary abstraction language.
- Describe behavior with concrete outcomes (returns, raises, valid/invalid inputs); no abstract phrasing or smarmy/overly clever wording or coding.
- Code must be written simply and clearly — even when behavior is complex — and must be skimmable by a human reader.
- For clarifying questions, be explicit about the exact field/behavior in question and present concrete options/examples so the user does not have to infer unstated assumptions.

---

## 2. Scope: this repo

**PI_DiskCutting** prepares **laser disk** cutting paths from **CAD (DXF)**. Outputs feed **DMS** and real hardware.

- **Primary script:** `import ezdxf.py` — contours, interpolation (splines/polylines), CSV columns, optional **`origin_xy_mm`** in `*_pvt.meta.json`.
- **Downstream:** Vivodyne **DMS** loads PVT and meta — see `Subsystems/Laser/PVT_BUFFER_LOAD_RECIPE.md` and `PvtCadMetaJson.cs` in the DMS repo for buffer/chip-map behavior.
- **Safety / correctness:** treat motion-related defaults (velocity, acceleration, units mm) and frame semantics (delta vs absolute, CAD origin in meta) as contractual; do not retune without explicit user agreement.

---

## 3. HC2 / Datacore archive

The long **USER_INTENT** below was ported from another codebase (KUKA, SceneBridge, Docker HC2, etc.). It is **not normative** for this Python repo. Full text:

- [`docs/archive/USER_INTENT_HC2_datacore_port.md`](docs/archive/USER_INTENT_HC2_datacore_port.md)

Use only for cross-repo or historical comparison.

---

## 4. PI_DiskCutting-specific intent

_Add durable invariants: e.g. column naming, meta schema, when to use delta CSV vs absolute, ezdxf flattening choices._

- *(Add bullets as stable rules emerge.)*
