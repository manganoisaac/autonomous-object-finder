# group_project_controller

Controller for the 3006ICT group project: it finds the station that shows the target in
`config/assessment_mission.json` and stops at that station's observation position.
The design and the evaluation are described in the final report.

## Setup

Requirements: Webots R2025a and Python 3.13 (see `.python-version`). The controller needs
PyTorch, torchvision, timm, OpenCV and NumPy; `pyproject.toml` lists them and `uv.lock` pins
the versions we tested.

### 1. Create the Python environment

With [uv](https://docs.astral.sh/uv/) (recommended), from this folder:

```bash
uv sync
```

This creates `.venv/` here with the locked versions.

Without uv, use any Python 3.13:

```bash
python3.13 -m venv .venv
.venv/bin/pip install torch torchvision timm opencv-python numpy matplotlib   # Windows: .venv\Scripts\pip
```

A CPU-only PyTorch build is enough. Inference takes about 32 ms per frame on four CPU threads.

### 2. Point Webots at that Python

Webots runs Python controllers with the interpreter set in
**Tools → Preferences → General → Python command**. Set it to the environment's interpreter:

- Linux/macOS: `<this folder>/.venv/bin/python`
- Windows: `<this folder>\.venv\Scripts\python.exe`

Alternatively, start Webots from a shell where that environment is activated, so that `python`
on the `PATH` is the environment's interpreter.

### 3. Model weights

The fine-tuned detector weights are part of the repository: `detector/dinov3_s.pt` (43 MB, fp16).
Nothing is downloaded at run time. The model is built with `pretrained=False`, and the weights are
loaded from that file, so the controller also runs offline.

### 4. Run the student missions

1. Set the target in `config/assessment_mission.json`, e.g. `{"target": "soda_can"}`.
2. Open `worlds/training_start_A.wbt` (or `_B`, `_C`) in Webots and press Run.

The controller prints the mission progress to the Webots console (`predicted_target`,
`Found target`, `SAFETY EVENT` / `STUCK` if the safety layers act) and a final summary with the
mission time and the distance to the observation position. After the mission the motors are
stopped, and the robot holds its position until the simulation is stopped.

## Layout

| Path | Purpose |
| --- | --- |
| `group_project_controller.py` | mission loop: station order → plan → drive → scan and classify |
| `detector/` | DINOv3 ViT-S/16 classifier (`model.py`) and its weights (`dinov3_s.pt`) |
| `path/` | A* planner on the occupancy grid (Dijkstra kept for comparison) |
| `stations/` | greedy nearest-station visiting order |
| `robot/`, `movement/`, `gps/` | motion primitives and pose from GPS/IMU |
| `safety/` | directional proximity monitoring and stuck detection |
| `benchmark_paths.py`, `plot_route_planning.py` | offline planner benchmark and figure (not used at run time) |

## Acknowledgements

- **DINOv3** (Siméoni et al., 2025, Meta AI). The detector is the
  `vit_small_patch16_dinov3.lvd1689m` checkpoint, fine-tuned by us on the eight provided
  reference images. Use of the DINOv3 weights is subject to the DINOv3 License.
- **timm / PyTorch Image Models** (R. Wightman) for the model definition, and **PyTorch** and
  **torchvision** for inference.
- **OpenCV** and **NumPy** for image handling. **matplotlib** is used only by the offline plotting scripts.
- **Webots** (Cyberbotics) and the **e-puck** robot model. The worlds, maps, textures and reference
  images are the course materials provided for 3006ICT.
