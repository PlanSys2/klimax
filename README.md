# Klimax (κλῖμαξ)

**An HTN planner for HDDL 2.1: hierarchical, numeric and temporal planning, usable as a library
and as an executable.**

> Work in progress. Nothing plans yet: this repository holds the skeleton, the semantics Klimax
> will implement and the plan of the work.

Klimax (Greek for *ladder*) reads [HDDL](https://doi.org/10.1609/aaai.v34i06.6542), the language
of the hierarchical tracks of the International Planning Competition, and its proposed extension
HDDL 2.1, which adds numeric fluents, durative actions and methods, and temporal constraints
between tasks. It is developed within the [PlanSys2](https://github.com/PlanSys2) organization,
next to [Kairos](https://github.com/PlanSys2/kairos), and is built on Kairos' modules (parser,
grounder, heuristics, temporal network, validator).

## Documents

- [docs/semantics.md](docs/semantics.md): the language Klimax accepts and what a solution is.
- [docs/plan.md](docs/plan.md): state of the art, design and phases.
- [docs/baseline.md](docs/baseline.md): the existing HTN planners on the IPC 2023 benchmark.
- [docs/results.md](docs/results.md): Klimax against them, on total order.

## Using

Klimax plans for totally ordered HDDL 1.0 problems. Partial order, numbers and time are not
planned yet.

```
klimax DOMAIN PROBLEM [--timeout S]    # the plan and its decomposition, in the IPC format
klimax --check DOMAIN PROBLEM          # read HDDL 1.0 and report every error
klimax --verify DOMAIN PROBLEM PLAN    # check a plan in the format of the IPC HTN tracks
```

## Building

Klimax needs Kairos installed with its internal modules (`kairos::core`):

```bash
cmake -S kairos -B build-kairos -DKAIROS_INSTALL_CORE=ON -DCMAKE_INSTALL_PREFIX=$PWD/prefix
cmake --build build-kairos -j && cmake --install build-kairos

cmake -S klimax -B build-klimax -DCMAKE_PREFIX_PATH=$PWD/prefix
cmake --build build-klimax -j
ctest --test-dir build-klimax --output-on-failure
./build-klimax/src/cli/klimax --version
```

In a colcon workspace: `colcon build --cmake-args -DKAIROS_INSTALL_CORE=ON`.

## License

Apache-2.0.
