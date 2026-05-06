# scenario-extraction

Extract scene graphs from driving video data using a modified [RoadScene2Vec](roadscene2vec/), then specify and query scenarios using LTL formulas via [SceneFlowLang](SceneFlowLang/). Ground-truth evaluation is provided for the Argoverse 2 dataset.

## Repository structure

```
scenario-extraction/
├── SceneFlowLang/          # LTL model checker & scenario extraction runner
│   └── my_results/         # Extraction outputs (SG_tracks, SG_tracks_kinematics, SG_tracks_kinematics_map)
├── scenarios/              # GT annotation processing & evaluation scripts
│   └── aps/                # Per-scenario ground-truth annotation packages (APs)
├── roadscene2vec/          # Scene graph extraction from raw sensor data
├── sg_processing/          # Track estimation & map pre-processing utilities
├── paper/                  # Supporting scripts used in the paper
└── replicate_data/         # Replication scripts — reproduce paper tables from pre-computed results
```

## Replication packages

The `replicate_data/` directory contains self-contained scripts for reproducing results from the paper. The replication study is being released in stages:

| Stage | Description | Status |
|-------|-------------|--------|
| 1 | Table replication — reproduce all paper tables from pre-computed results | **Available now** |
| 2 | Scenario extraction from scene graphs — re-run SceneFlowLang on the provided scene graphs | Coming soon |
| 3 | Full pipeline — scene graph generation from raw sensor data + scenario extraction | Coming soon |

---

### Stage 1 — Table replication (available now)

These scripts read the pre-computed extraction outputs and ground-truth annotation packages, then print the corresponding paper table to stdout. No re-running of the scene-graph pipeline or SceneFlowLang is needed.

Run all scripts from the **repo root**:

```bash
cd /path/to/scenario-extraction
```

#### Table 1 — Ablation study (many-to-one matching, 50% overlap)

Reproduces Table 1, which compares precision / recall / F1 across three scene-graph semantic levels (tracking-only, tracking+state, tracking+state+map) and a non-track-aware baseline, for the cut-in and longitudinal-following scenarios.

```bash
python replicate_data/table_1_ablation.py
```

**Dependencies:** standard library only (no extra packages required beyond what is already used by `scenarios/`).

**Expected output** (values should match the paper exactly):

```
Table 1: Ablation — many-to-one matching, 50% overlap, track-aware (rows 1-9) / non-track-aware (rows 10-12)
-----------------------------------------------------------------------------------------------------------
SG Level                                    Scenario                         N  Precision  Recall      F1
-----------------------------------------------------------------------------------------------------------
Level I: Tracking-only                      φ_long_following (30 m)       1368      0.165   0.424   0.238
                                            φ_long_following (60 m)       1944      0.193   0.393   0.259
                                            φ_cut_in                        64      0.391   0.281   0.327
-----------------------------------------------------------------------------------------------------------
Level II: Tracking & State                  φ_long_following (30 m)       1368      0.453   0.397   0.423
                                            φ_long_following (60 m)       1944      0.416   0.365   0.389
                                            φ_cut_in                        64      0.847   0.781   0.813
-----------------------------------------------------------------------------------------------------------
Level III: Tracking, State, & Map           φ_long_following (30 m)       1368      0.632   0.583   0.607
                                            φ_long_following (60 m)       1944      0.632   0.490   0.552
                                            φ_cut_in                        64      0.943   0.781   0.855
-----------------------------------------------------------------------------------------------------------
Tracking, State, & Map (non-track-aware)    φ_long_following (30 m)       1368      0.828   0.778   0.802
                                            φ_long_following (60 m)       1944      0.893   0.770   0.827
                                            φ_cut_in                        64      0.943   0.781   0.855
-----------------------------------------------------------------------------------------------------------
```

#### Table 2 — Ablation study (timestamp-based temporal localization)

Reproduces Table 2, which evaluates the same SG levels using duration-based (timestamp) matching rather than instance counting. TP/FP/FN are measured in seconds of overlap rather than interval counts.

```bash
python replicate_data/table_2_ablation_temporal.py
```

**Expected output** (values should match the paper exactly):

```
Table 2: Ablation — timestamp-based temporal localization, track-aware (rows 1-9) / non-track-aware (rows 10-12)
-------------------------------------------------------------------------------------------------
SG Level                                    Scenario                    Precision  Recall      F1
-------------------------------------------------------------------------------------------------
Level I: Tracking-only                      φ_long_following (30 m)         0.394   0.781   0.523
                                            φ_long_following (60 m)         0.497   0.750   0.598
                                            φ_cut_in                        0.459   0.315   0.373
-------------------------------------------------------------------------------------------------
Level II: Tracking & State                  φ_long_following (30 m)         0.747   0.714   0.730
                                            φ_long_following (60 m)         0.786   0.695   0.738
                                            φ_cut_in                        0.805   0.761   0.782
-------------------------------------------------------------------------------------------------
Level III: Tracking, State, & Map           φ_long_following (30 m)         0.788   0.789   0.788
                                            φ_long_following (60 m)         0.865   0.744   0.800
                                            φ_cut_in                        0.941   0.755   0.838
-------------------------------------------------------------------------------------------------
Tracking, State, & Map (non-track-aware)    φ_long_following (30 m)         0.841   0.819   0.830
                                            φ_long_following (60 m)         0.931   0.779   0.848
                                            φ_cut_in                        0.941   0.755   0.838
-------------------------------------------------------------------------------------------------
```

#### Table 3 — Log-level scenario extraction performance

Reproduces Table 3, which evaluates scenario detection at the log level: each of the 848 logs is classified as positive or negative, and log-level balanced accuracy, TP rate, and TN rate are reported across the three SG levels.

```bash
python replicate_data/table_3_ablation_log_metrics.py
```

**Expected output** (values match the paper; N=848 vs paper's 850 due to 2 logs absent from results). Argoverse 2 Training and Validation splits have a total of 848 driving logs, instead of the 850 as advertised, as two scenarios are duplicated:

```
Table 3: Log-level scenario extraction performance (non-track-aware)
-------------------------------------------------------------------------------------------------------------
SG Level                                    Scenario                        N  Log-bal. Acc  TP Rate  TN Rate
-------------------------------------------------------------------------------------------------------------
Level I: Tracking-only                      φ_long_following (30 m)       848         0.539    0.993    0.085
                                            φ_long_following (60 m)       848         0.514    1.000    0.029
                                            φ_cut_in                      848         0.639    0.311    0.966
-------------------------------------------------------------------------------------------------------------
Level II: Tracking & State                  φ_long_following (30 m)       848         0.845    0.915    0.776
                                            φ_long_following (60 m)       848         0.813    0.930    0.695
                                            φ_cut_in                      848         0.889    0.787    0.991
-------------------------------------------------------------------------------------------------------------
Level III: Tracking, State, & Map           φ_long_following (30 m)       848         0.822    0.913    0.731
                                            φ_long_following (60 m)       848         0.819    0.886    0.753
                                            φ_cut_in                      848         0.901    0.803    0.999
-------------------------------------------------------------------------------------------------------------
```

> **Note on Level II, long_following (30 m):** The paper lists TPR=0.706, TNR=0.910 for this row, but those values are internally inconsistent with the paper's own balanced accuracy of 0.845 — (0.706+0.910)/2 = 0.808 ≠ 0.845. The values produced here (TPR=0.915, TNR=0.776) are consistent: (0.915+0.776)/2 = 0.845. This was a typo, and will be fixed in the camera-ready version of the paper.

#### Table 4 — SG track fragmentation

Reproduces Table 4, which reports how SG (RoadScene2Vec) tracks fragment relative to the ground-truth AV2 tracks, restricted to tracks that participate in GT scenario APs.

```bash
python replicate_data/table_4_track_fragmentation.py
```

**Expected output** (values match the paper exactly):

```
Table 4: SG track fragmentation relative to AV2 tracks (GT scenario tracks)
----------------------------------------------------------------------------------------------
      Total # Tracks     Fragments / Track  Fragmented Tracks (%)  Dominant Track Coverage (%)
----------------------------------------------------------------------------------------------
                4494                 1.587                  34.1                  63.6
----------------------------------------------------------------------------------------------
```

#### Table 5 — Scenario coverage of the full Argoverse 2 dataset

Reproduces Table 5, which reports unique instance counts, median durations, unique scene counts, and dataset coverage percentage for each scenario variant, using the track+state+map scene graphs.

```bash
python replicate_data/table_5_compute_coverage.py
```

**Expected output** (values match the paper exactly):

```
Table 5: Scenario coverage of the Argoverse 2 dataset (track + HD map SG)
---------------------------------------------------------------------------------------------------------------------
Scenario                                    Unique # Instances  Median Duration (s)  Unique Scenes  % of Full Dataset
---------------------------------------------------------------------------------------------------------------------
φ_long_following,30m                                      1275                 1.90            585              68.99
φ_long_following,60m                                      1557                 2.00            640              75.47
φ_long_following,very_near,LV_decel                         30                 4.05             25               2.95
φ_cut_in_from_right                                         53                 8.30             50               5.90
φ_cut_in_from_right,snowy                                    0                 0.00              0               0.00
φ_cut_in_from_right,LV_decel                                 3                12.80              3               0.35
---------------------------------------------------------------------------------------------------------------------
```

---

### Stage 2 — Scenario extraction from scene graphs (coming soon)

Scripts to re-run SceneFlowLang scenario extraction directly from the provided scene graphs, reproducing the extraction results that the Stage 1 table scripts consume.

---

### Stage 3 — Full pipeline: scene graph generation + scenario extraction (coming soon)

Scripts covering the complete pipeline: running RoadScene2Vec on raw Argoverse 2 sensor data to regenerate scene graphs, followed by scenario extraction end-to-end.

---

## Authors and acknowledgment

Stefan Ramdhan

## License

See [LICENSE](LICENSE) if present.
