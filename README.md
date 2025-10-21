# Testing-Hall-Sensor / N32G430 Motor Control

Short description
- This repository contains firmware source code for an N32G430-based brushless motor control project (NZ_MotorControl_2shunt_Sense). It includes application code (`App/`), device drivers and CMSIS (`Driver/`), motor control algorithms and FOC code (`Foc/`), and Keil MDK project files under `RVMDK/`.

What you'll find
- `App/` - application-level code (main, drivers wrappers)
- `Driver/` - CMSIS and peripheral driver sources
- `Foc/` - field-oriented control algorithms, observers, and supporting code
- `RVMDK/` - Keil µVision project, build outputs, and tool-specific files

Quick build notes
- This project is set up for Keil µVision (MDK-ARM). To build:
  1. Open `RVMDK/RVMDKN32G430/NZ_MotorControl_2shunt_Sense.uvprojx` in Keil µVision.
  2. Select the desired target and toolchain settings.
  3. Build the project (Project -> Build target). Outputs are placed in the `Obj/` folder under the Keil project.

Notes about git and this repository
- The repository contains IDE project files and generated build artifacts. These files should not be committed to GitHub. A `.gitignore` has been added to exclude common build artifacts, Keil user files, JLink logs, and typical IDE folders.

If you already committed build outputs
1. Add the `.gitignore` (already added).
2. Remove tracked build artifacts from git while keeping them locally:

   git rm --cached -r RVMDK/**/Obj
   git rm --cached RVMDK/**/JLinkLog.txt
   git commit -m "Remove generated build artifacts from repository"

Helpful tips
- Keep third-party libraries (CMSIS, vendor drivers) under `Driver/` if you want reproducible builds. If you prefer, move prebuilt binaries to a separate `third_party/` folder and list them in README.
- For remote collaboration, avoid pushing `*.uvoptx` or `*.user` files — they contain local/debug settings.

Contact / Next steps
- If you'd like, I can:
  - Remove already-tracked build artifacts (run the git rm --cached commands above).
  - Add a CONTRIBUTING.md with build environment and required toolchain versions.
  - Create a small script to export/clean build artifacts.

  congratulations
