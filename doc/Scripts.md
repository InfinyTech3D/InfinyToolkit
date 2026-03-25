# Scripts Folder Contents

The Scripts folder has a collection of python scripts to provide basic utilities.

## runBenchmarks_script.py

```
SOFA Scene Benchmarking Tool

Usage:
    python benchmark.py -config benchmark.json
    python benchmark.py -config benchmark.json -n_tests 10 -timeout 180

Config file format (JSON):
    {
        "sofa_exe":   "runSofa",
        "iterations": 110,
        "n_tests":    5,
        "warmup":     2,
        "timeout":    120,
        "output":     "log.benchmark",
        "cases": [
            { "name": "baseline",   "scene": "scene_baseline.scn" },
            { "name": "refactored", "scene": "scene_refactored.scn"     }
        ]
    }

    - `sofa_exe`: command to run (to choose SOFA version).
        - Default: `runSofa`
    - `iterations`: # of time steps to run in batch mode. 
    - `n_tests`: # of tests to run. Mean & std of FPS are computed. 
    - `warmup`: # of times to do a dry run to avoid false reports due to caching.
        - Default: 2
    - `timeout`: # seconds to run before killing process. For stale runs that crashed.
    - `output`: Name of the file to report results.
        - Default: "log.benchmark"
    - `cases`: Which scene files to use


All top-level config keys can be overridden individually via CLI arguments.
```
