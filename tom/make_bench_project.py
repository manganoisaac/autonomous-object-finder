"""Assemble a self-contained Webots project for headless benchmark/Monte Carlo runs.

Copies the course assets, points the e-puck at a chosen controller, adds the
referee supervisor, and rewrites remote EXTERNPROTO URLs to webots:// so the
world loads from the assets baked into the container. The official worlds in
worlds/ are never modified.
"""
import argparse
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
REMOTE = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/"
REFEREE = '\nRobot {\n  name "referee"\n  controller "referee"\n  supervisor TRUE\n}\n'


def lite(text):
    """Cheaper rendering: flat sky and floor. Camera images differ from the real world."""
    text = text.replace("TexturedBackground { }", "Background { skyColor [ 0.6 0.7 0.85 ] }")
    text = text.replace("TexturedBackgroundLight { }",
                        "DirectionalLight { direction 0.3 -0.4 -1 intensity 2.2 castShadows FALSE }\n"
                        "PointLight { location 0 0 2 intensity 6 }")
    return text.replace("RectangleArena {\n",
                        "RectangleArena {\n  floorAppearance PBRAppearance "
                        "{ baseColor 0.75 0.7 0.6 roughness 1 metalness 0 }\n", 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("course", type=Path, help="path to the provided 3006ICT Webots project")
    parser.add_argument("out", type=Path)
    parser.add_argument("--controller", default="bench_controller")
    args = parser.parse_args()

    REPO = args.course.resolve()
    out = args.out
    if out.exists():
        shutil.rmtree(out)
    for name in ("config", "maps", "protos", "targets", "textures"):
        shutil.copytree(REPO / name, out / name)
    shutil.copytree(REPO / "controllers", out / "controllers")
    for controller in (HERE / "controllers").iterdir():
        shutil.copytree(controller, out / "controllers" / controller.name, dirs_exist_ok=True)
    shutil.copy(REPO / "controllers" / "group_project_controller" / "project_utils.py",
                out / "controllers" / "bench_controller" / "project_utils.py")

    (out / "worlds").mkdir()
    for world in sorted((REPO / "worlds").glob("training_start_*.wbt")):
        text = world.read_text().replace(REMOTE, "webots://")
        text = text.replace('controller "group_project_controller"', f'controller "{args.controller}"')
        text += REFEREE
        start = world.stem.rsplit("_", 1)[-1]
        (out / "worlds" / f"bench_{start}.wbt").write_text(text)
        (out / "worlds" / f"bench_{start}_lite.wbt").write_text(lite(text))
    print("wrote", out, sorted(p.name for p in (out / "worlds").iterdir()))


if __name__ == "__main__":
    main()
