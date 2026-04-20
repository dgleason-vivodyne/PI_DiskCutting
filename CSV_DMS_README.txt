DXF-derived PVT CSV format (PI_DiskCutting: import ezdxf.py -> generate_csv_from_points)
======================================================================================

Canonical tooling for this pipeline lives in the PI_DiskCutting repository.

Column order (required by DMS SendPvtToXMccWithIoInjectionAsync):
  1  Time (s)
  2  Horizontal position (mm)   -- relative / delta pattern per row as generated
  3  Horizontal velocity (mm/s)
  4  Vertical position (mm)
  5  Vertical velocity (mm/s)

Optional column (Python exporter):
  6  Cut (1=cut 0=travel)     -- 1 = laser cutting pass, 0 = travel/rapid between contours

DMS behavior (LaserService.cs):
  - Lines are split on commas; the first five numeric fields drive PointRelative PVT.
  - Rows that fail numeric parse are skipped (header row skipped automatically).
  - Extra columns after column 5 are currently IGNORED. Travel vs cut is not yet
    mapped to laser IO (AOM on/off); future work can use column 6 or text lines
    like "Turn Laser Off" between travel and cut segments.

Text directives (already supported in DMS CSV):
  - "Turn Laser On" / "Turn Laser Off" (case-insensitive substring match)
  - "Set Laser Voltage To", <volts>

Generating files:
  - Run import ezdxf.py from PI_DiskCutting; overlap and DXF path are prompted in the terminal.
  - Output path: <dxf_basename>_pvt.csv next to the resolved DXF unless you change the script.
