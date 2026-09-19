# Monte Carlo harness (headless Webots, local Docker or Griffith HPC)

Runs many copies of the project world without a display and records each
outcome. The course project (worlds, textures, config) is only read, never modified;
`make_bench_project.py` builds a separate copy with a referee supervisor added.

## Why the extra steps

- R2025a worlds load PROTOs, textures and even motor sounds from
  raw.githubusercontent.com, and neither the official image nor the .deb ships
  them. The HPC has no outbound network, and a failed download makes Webots
  hang and segfault. `docker/vendor_assets.py` mirrors everything the project
  world needs, and the Dockerfile rewrites the remaining URLs to `webots://`.
- Webots only tries ports 1234-1244 by default, so more than 11 instances per
  machine fail. `hpc_run.sh` gives every instance its own `--port`.
- Under Apptainer, Webots must write into its install tree while expanding PROTO
  templates. Always pass `--writable-tmpfs`.

## Local (Docker)

    cd tom/docker && docker build -t webots-mc:R2025a . && cd ../..
    python3 tom/make_bench_project.py <path-to-3006ict-webots-project> /tmp/wbproj
    mkdir -p /tmp/wbres
    docker run --rm --user $(id -u):$(id -g) -v /tmp/wbproj:/proj -v /tmp/wbres:/results \
      -v $PWD/tom/hpc_run.sh:/hpc_run.sh webots-mc:R2025a \
      bash /hpc_run.sh 4 60 bench_A /results local

`hpc_run.sh N SIM_SECONDS WORLD OUTDIR TAG` starts N instances. Worlds are
`bench_{A,B,C}` (full fidelity) and `bench_{A,B,C}_lite` (flat sky and floor,
about 2x faster, but camera images differ from the assessed world).

## Griffith HPC (PBS + Apptainer, CPU only, `workq`)

    docker save webots-mc:R2025a | gzip -1 > webots-mc.tar.gz
    scp webots-mc.tar.gz tom/hpc_run.sh tom/pbs/*.pbs \
        s<ID>@clogin1.rcs.griffith.edu.au:/scratch/s<ID>/webots_mc/
    # also copy the project built by make_bench_project.py to .../webots_mc/proj
    qsub build_sif.pbs     # builds webots-mc.sif on a compute node and runs a smoke test
    qsub array.pbs         # 100 x 1-core subjobs

Request memory carefully: other users hold most of the RAM on the `cn` nodes,
so small subjobs (1 core, 1 GB) start immediately while one large 24 GB job can
wait indefinitely.

## Measured throughput (2026-09-19)

| Setting | Result |
|---|---|
| HPC, 1 core per instance | RTF 0.96 (min 0.71, max 1.10) |
| Webots startup | about 27 s per run |
| Memory | about 0.53 GB RSS per instance |
| HPC array of 100 x 1 core | 48 ran at once (bound by free RAM), 100/100 OK, 0 errors |
| Aggregate | 23.7 simulated s per wall s, about 1400 runs of 60 s per hour |
| Local 20-core desktop, 20 instances | 9.4 sim s per wall s; lite world 19.0 |
