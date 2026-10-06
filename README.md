# Analog MEMS Electronic Stethoscope

University engineering project: the design and development of an electronic stethoscope based on an analog MEMS microphone.

## Project Overview

This project develops an electronic stethoscope that captures body sounds (heart and lung sounds) with an analog MEMS acoustic sensor and processes them electronically. The repository holds the complete project: electronics, mechanical design, firmware, and documentation.

The system covers:

- **MEMS acoustic sensing**: an analog MEMS microphone captures auscultation sounds.
- **Analog front-end**: amplification and biasing of the MEMS sensor output.
- **Signal conditioning**: analog filtering to isolate the frequency bands of interest and prepare the signal for digitization.
- **Embedded processing**: an ESP32 microcontroller handles audio acquisition (ADC sampling) and digital filtering.
- **BLE communication** (where applicable): wireless transfer of audio or data to an external device.
- **PCB design**: schematic capture, PCB layout, BOM, and manufacturing outputs.
- **Mechanical enclosure**: SolidWorks-designed housing, chest piece, and assembly.
- **Firmware**: embedded software for acquisition, filtering, and communication.

> Detailed specifications (sampling rate, filter bands, component selection, etc.) will be documented in `docs/` and in each subsystem folder as the design matures.

## Team

| Member | Responsibility | GitHub |
|---|---|---|
| Pirathi | Firmware / Embedded Systems | [pirathi2002](https://github.com/pirathi2002) |
| Dayananthan | Mechanical Design | [Dayananthan2021](https://github.com/Dayananthan2021) |
| Thamilezai Ananthakumar | PCB / Electronics Design | [ThamilezaiAnanthakumar](https://github.com/ThamilezaiAnanthakumar) |

## Repository Structure

```
analog-mems-electronic-stethoscope/
├── docs/                   Project documentation
│   ├── literature-review/  Papers, notes, and literature survey material
│   ├── presentations/      Slides for each evaluation stage
│   │   ├── literature-review/
│   │   ├── circuit-design/
│   │   ├── mid-evaluation/
│   │   └── final/
│   └── reports/            Progress, monthly, and final reports
│
├── mechanical/             Mechanical design (owner: Dayananthan)
│   ├── solidworks/         SolidWorks parts (.SLDPRT) and assemblies (.SLDASM)
│   ├── drawings/           Engineering drawings (.SLDDRW, PDF exports)
│   └── documentation/      Design notes, material choices, assembly instructions
│
├── electronics/            Electronics design (owner: Thamilezai)
│   ├── schematic/          Schematic source files
│   ├── pcb/                PCB layout source files
│   ├── bom/                Bill of materials
│   ├── gerber/             Released Gerber / drill files
│   └── datasheets/         Component datasheets
│
├── firmware/               Embedded software (owner: Pirathi)
│   ├── src/                Source files (.c)
│   ├── include/            Header files (.h)
│   ├── config/             Configuration (e.g. STM32CubeMX .ioc, linker scripts)
│   └── README.md           Firmware build and usage notes
│
├── manufacturing/          Fabrication-ready outputs
│   ├── mechanical/         STL/STEP files for 3D printing or machining
│   └── electronics/        PCB fabrication and assembly packages
│
└── images/                 Photos, renders, and diagrams
    ├── mechanical/
    ├── pcb/
    └── system/
```

## Development Workflow

`main` is the **stable integration branch**. Nobody develops directly on `main`; all changes go through Pull Requests from personal branches.

| Branch | Owner | Works inside |
|---|---|---|
| `firmware/pirathi` | Pirathi | `firmware/` |
| `mechanical/dayananthan` | Dayananthan | `mechanical/` |
| `pcb/thamilezai` | Thamilezai | `electronics/` |

Shared folders (`docs/`, `images/`, `manufacturing/`) may be updated from any branch. Coordinate with the team to avoid editing the same file at the same time.

```
                    ┌─────────────────────┐
                    │        main         │
                    │   Stable Project    │
                    └──────────┬──────────┘
                               ↑
                         Pull Requests
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
  firmware/pirathi    mechanical/dayananthan      pcb/thamilezai
          │                    │                    │
      Firmware             SolidWorks              PCB
      Embedded             Mechanical          Electronics
      Software               Design               Design
```

### Step by step

1. **Pull the latest `main`**
   ```bash
   git checkout main
   git pull origin main
   ```
2. **Switch to your personal branch and bring it up to date with `main`**
   ```bash
   git checkout firmware/pirathi        # or mechanical/dayananthan, pcb/thamilezai
   git merge main
   ```
   If a merge conflict occurs, **stop** and discuss it with the owner of the conflicting file before resolving it.
3. **Make your changes** inside your own folder.
4. **Commit**
   ```bash
   git status
   git add firmware/                    # or mechanical/, electronics/
   git commit -m "Add <short description>"
   ```
5. **Push your branch**
   ```bash
   git push origin firmware/pirathi
   ```
6. **Create a Pull Request** on GitHub: `your-branch → main`.
7. **Review**: at least one other team member reviews the PR.
8. **Merge** into `main` once the review is approved.

### Rules

- Never commit directly to `main`.
- Always pull `main` and merge it into your branch before starting new work.
- Do not modify or overwrite another member's files without discussing it first.
- Never use `git push --force` on shared branches.
- Write clear commit messages that describe *what* changed and *why*.

## Large Files

Large binary files (SolidWorks models, PCB design archives, presentations, PDFs, images) are stored with [Git LFS](https://git-lfs.com/). Install it once per machine before cloning:

```bash
git lfs install
```

The tracked file types are listed in `.gitattributes`.
