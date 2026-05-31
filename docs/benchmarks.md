# Benchmarks

Benchmark scripts live in `src/benchmark`.
Repository-level launch helpers live in `scripts/`.

Dense obstacle simulation:

```bash
scripts/run_dense_simulation.sh --count 160 --min_dist 1.8 --seed 41
```

DWA and TEB baselines:

```bash
scripts/run_dwa_simulation.sh
scripts/run_teb_simulation.sh
```

MPC experiments:

```bash
scripts/run_mpc_experiments.sh baseline
scripts/run_mpc_experiments.sh no_ff
scripts/run_mpc_experiments.sh no_delay
scripts/run_mpc_experiments.sh analyze
```

Python regression checks:

```bash
python3 -m pytest src/benchmark src/nlp_commander/tests
```
