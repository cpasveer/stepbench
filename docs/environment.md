# The environment record

Every run carries a record of what it was made with: the model, the CAD
tool and its add-ons, the Python packages, the benchmark's own version, the
tasks, and the machine. A result means little if nobody can say what
produced it; a model that passes with FreeCAD 1.1 and a fastener add-on has
not shown the same thing as one that passes with bare build123d.

The record is **captured by the machine at the start of the run**, not
typed in afterwards. What a program can read for itself, it reads. Only what
it cannot read (which model was called and how, a connector running
elsewhere, the agent loop) is passed in by the harness that starts the run,
and is kept apart and labelled as given.

## Making it

The harness runs this once, at the start of the run, in the environment the
agent will work in:

```
uv run jointbench env runs/opus-model-track \
  --model claude-opus-5-5 --provider anthropic --track model --tool build123d \
  --agent-loop example-harness=1.2.0 --task runs/opus-model-track/facing-l1
```

It writes `runs/opus-model-track/environment.json`. In the official box the
launcher runs the same probe inside the box before the agent gets control,
and keeps the record outside, where the agent cannot change it. Because the
box holds no grader, the probe there has to be a standalone script with no
grading code in it; today it is part of the package (to do for the box).

| field | how it is filled |
|---|---|
| **model**: exact id, provider (direct API, OpenRouter, ...) | given by the harness (`--model`, `--provider`) |
| run date and time | the machine's clock, UTC |
| **track** | given (`--track`) |
| **CAD tool** declared | given (`--tool`) |
| CAD programs found: path, exact version, every add-on or workbench with its version | **read**: FreeCAD is started headless and reports its version and its `Mod` folder; OpenSCAD and SolidCraft report `--version` |
| a program the benchmark cannot find or read (Rhino, Onshape, ...) | given (`--tool-version NAME=VERSION`) |
| connector or MCP server, with version | given (`--mcp NAME=VERSION`) |
| **Python packages**: build123d, OCP and every other, with versions | **read**: every installed package (the same as a `pip freeze`) |
| agent loop, with version | given (`--agent-loop NAME=VERSION`) |
| benchmark version and exact source (git commit, and whether it was modified) | **read** |
| tasks: id, seed, generator version, task set | **read** from each task's `task.json` (`--task`) |
| machine: operating system and version, processor, Python | **read** |
| **box image and its digest** (official runs) | **read** from the box's launcher (`JOINTBENCH_BOX_IMAGE`, `JOINTBENCH_BOX_DIGEST`) |

Not recorded, on purpose: the machine's name and the user's name. Paths in
the user's home folder are written with `~`. They add nothing to repeating a
run and would publish personal details.

## Declared tool against the STEP file

Every graded attempt also compares the declared tool (`verify --tool`) with
what the STEP file's own header says wrote it, and records one of:

- **consistent**: the header names the declared tool (build123d writes
  `build123d`, FreeCAD writes `FreeCAD`; OpenSCAD's export goes through
  FreeCAD here, so `FreeCAD` is consistent with `openscad`);
- **inconsistent**: the header names a different known tool;
- **cannot tell**: the header names no known tool.

This is a cross-check, not proof: a header is text, and anyone can write
anything in it. It catches mistakes and makes a false claim visible, and it
never changes a verdict.

## Reproducing an official run

Official runs happen in a sealed box (see [participate.md](participate.md)).
Each box is a published container image, identified by its digest: a
fingerprint of its exact contents. The record of an official run names that
digest. Anyone can download the image with that digest, which is the same
box to the last file, and run the same tasks in it. What cannot be
fixed this way is the model itself, which runs at its provider and answers
differently from one call to the next.

## Example: model track, build123d

Shortened: the package list has some sixty entries.

```json
{
  "format": 1,
  "captured": "2026-11-14T08:15:02+00:00",
  "given_by_harness": {
    "model": "claude-opus-5-5",
    "provider": "anthropic",
    "track": "model",
    "tool": "build123d",
    "tool_versions": {},
    "mcp_servers": {},
    "agent_loop": {"example-harness": "1.2.0"}
  },
  "measured": {
    "benchmark": {"name": "jointbench", "version": "0.1.0",
                  "commit": "39eaa9234f69eac6e514bb0ff9a349aed7b4c26e", "modified": false},
    "machine": {"os": "Linux", "os_version": "6.8.0", "platform": "Linux-6.8.0-x86_64-with-glibc2.39",
                "cpu": "x86_64", "cpus": 8, "python": "3.12.11", "python_implementation": "CPython",
                "box": {"image": "stepbench/box-model:v1",
                        "digest": "sha256:<digest of the published image>"}},
    "cad_programs": {},
    "python_packages": {"build123d": "0.13.0", "cadquery-ocp-novtk": "...", "numpy": "...", "...": "..."},
    "tasks": [{"id": "facing-l1", "seed": null, "level": 1, "generator_version": 5,
               "format_version": 1, "task_set": null}]
  }
}
```

In an official run the seed is `null`: the copy the agent works on has it
removed, and the grader outside keeps it.

## Example: tool track, FreeCAD over MCP

The FreeCAD part is what the command read on a test machine; the model and
the MCP server are given by the harness.

```json
{
  "format": 1,
  "captured": "2026-11-14T08:22:40+00:00",
  "given_by_harness": {
    "model": "claude-sonnet-5-5",
    "provider": "openrouter",
    "track": "tool",
    "tool": "freecad",
    "tool_versions": {},
    "mcp_servers": {"freecad-mcp": "0.4.1"},
    "agent_loop": {"example-harness": "1.2.0"}
  },
  "measured": {
    "benchmark": {"name": "jointbench", "version": "0.1.0", "commit": "...", "modified": false},
    "machine": {"os": "Darwin", "os_version": "24.6.0", "platform": "macOS-15.6-arm64-arm-64bit",
                "cpu": "arm64", "cpus": 12, "python": "3.12.11", "python_implementation": "CPython",
                "box": null},
    "cad_programs": {
      "freecad": {
        "path": "/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd",
        "version": "1.1.3",
        "build": "20260725 (Git shallow) Unknown",
        "addons": {"FreeCADMCP": null, "fasteners": "0.5.64"}
      }
    },
    "python_packages": {"build123d": "0.13.0", "...": "..."},
    "tasks": [{"id": "facing-l1", "seed": 0, "level": 1, "generator_version": 5,
               "format_version": 1, "task_set": null}]
  }
}
```

An add-on with `null` has no version in its `package.xml`; the record
shows that rather than guessing.
