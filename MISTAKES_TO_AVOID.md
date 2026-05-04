# Mistakes To Avoid (PI_DiskCutting)

## Rule

On every user-corrected mistake, add one concise entry as a **LESSON LEARNED** that avoids this type of mistake in the future. Generalize to a **class** of mistake; do not log one-off trivia. New **repo-specific** lessons go in **§ PI_DiskCutting lessons** below.

---

## Scope

This repository is **PI_DiskCutting** — **Python** tooling (notably `import ezdxf.py`) that turns **DXF** geometry into **PVT CSV** and sidecar **`*.pvt.meta.json`** (CAD origin / headers) for consumption by **DMS** (`LaserService`, `PvtCadMetaJson`). It is not the HC2 / Datacore / ROS monorepo.

A preserved long-form lessons doc from another codebase (FrameStore, ROS, pytest hygiene in that stack, etc.) lives here for reference only:

- [`docs/archive/MISTAKES_TO_AVOID_HC2_datacore_port.md`](docs/archive/MISTAKES_TO_AVOID_HC2_datacore_port.md)

For defaults that affect real cutting, stay aligned with the **DMS** laser/PVT recipe and hardware expectations; ask before changing exported scales, velocity/acceleration defaults, or coordinate conventions.

---

## Habits that still apply here

- **Fail fast** on bad DXF inputs, missing layers/entities, or inconsistent units — do not emit partial PVT that looks valid.
- **No silent “fixes”** for geometry or motion parameters that downstream treats as truth for hardware.
- **Explicit typing and validation** at boundaries (CSV columns, meta JSON schema, `origin_xy_mm`).

---

## PI_DiskCutting lessons

_Add concise, reusable lessons for DXF import, contour generation, delta vs absolute CSV, and meta JSON._

- *(none yet — append below as patterns emerge.)*
