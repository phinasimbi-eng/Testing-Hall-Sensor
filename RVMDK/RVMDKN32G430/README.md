# NZ_MotorControl_2shunt_Sense (N32G430)

This repository contains a Keil/MDK µVision project for a brushless motor control demo targeting the Nationstech N32G430 MCU.

Contents
- `NZ_MotorControl_2shunt_Sense.uvprojx` - Keil µVision project file
- `Obj/` - Build artifacts (object files, linker outputs, .axf, .hex)
- `List/` - Generated listings and startup files
- `lib/` - Prebuilt libraries used by the project
- `Driver/`, `App/`, `Foc/`, etc. - Source folders referenced by the project (located relative to the project file)

Quick notes
- This project is built with Keil µVision (ARMCC/ARM-ADS toolchain configured in the project file).
- Output artifacts (found in `Obj/`) include: `NZ_MotorControl_2shunt.axf`, `NZ_MotorControl_2shunt.hex`, `*.o`, `*.d`, and many intermediate listing files.

.gitignore
A `.gitignore` file has been added to avoid committing build artifacts, IDE/workspace files, logs, and temporary files. If you want to track `lib/` or other folders, remove them from `.gitignore`.

Build (Keil µVision)
1. Open `NZ_MotorControl_2shunt_Sense.uvprojx` in Keil µVision.
2. Make sure the Device Pack for N32G430 is installed and the toolchain matches the project settings.
3. Build (Project -> Build target) — outputs go to the `Obj/` folder.

Recommended Git workflow and cleanup
If you haven't pushed build artifacts yet, simply commit the `.gitignore` (already added here). If you have committed build outputs and want to stop tracking them, run the following in the repository root (PowerShell):

```powershell
# commit .gitignore first (if not committed)
git add .gitignore
git commit -m "chore: add .gitignore to exclude build artifacts and user files"

# remove previously tracked files that are now ignored, but keep them on disk
git rm -r --cached Obj
git rm --cached NZ_MotorControl_2shunt.axf
# repeat for any other committed artifacts you want to stop tracking

git add .
git commit -m "chore: remove build artifacts from tracking"

# push to remote
git push
```

If you need to purge large files from history use the `git filter-repo` or `bfg` tool — ask for guidance and I can provide exact commands.

Notes & next steps
- If `lib/` contains prebuilt libraries you want versioned, remove it from `.gitignore`.
- If you want, I can run the git cleanup commands for you now (I will not change remote branches). Tell me to proceed.
- I can also add a small CONTRIBUTING or BUILD script if you want to support CLI-based builds or CI.

Contact
If something looks off in paths or you want a different layout, tell me what to change and I'll update the README and `.gitignore` accordingly.
