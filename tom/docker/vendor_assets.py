"""Mirror the remote Webots R2025a assets a world needs, so Webots runs offline.

R2025a worlds reference PROTOs by URL and the official image/deb ship none of
them, so an offline cluster node cannot load the world without this step.
"""
import os
import re
import sys
import time
from pathlib import Path
from urllib.request import Request, urlopen

BASE = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/"
URL_RE = re.compile(r'(?:https://raw\.githubusercontent\.com/cyberbotics/webots/R2025a/|webots://)([^"\s\]]+)')
# JavaScript template fragments such as `textures/cubic';` are not real paths.
UNSAFE = re.compile(r"['`;+{}]")
REL_RE = re.compile(r'"([^"\s]+\.(?:proto|jpg|jpeg|png|hdr|obj|mtl|dae|stl))"', re.I)

# Assets that PROTOs build from string templates, which the regexes cannot see.
TEMPLATED = [f"projects/default/worlds/textures/cubic/mountains_{side}.{ext}"
             for side in ("left", "right", "top", "bottom", "front", "back") for ext in ("jpg", "hdr")]
TEMPLATED += [f"projects/appearances/protos/textures/parquetry/chequered_parquetry_{m}.jpg"
              for m in ("base_color", "roughness", "normal", "occlusion")]
# Default sounds of built-in nodes (e.g. RotationalMotor). They live in the Webots
# binary, not in any PROTO; a missing one makes Webots hang then segfault offline.
TEMPLATED += [f"projects/default/worlds/sounds/{name}.wav"
              for name in ("bump", "linear_motor", "roll", "rotational_motor", "slide")]
SEEDS = [
    "projects/objects/backgrounds/protos/TexturedBackground.proto",
    "projects/objects/backgrounds/protos/TexturedBackgroundLight.proto",
    "projects/objects/floors/protos/RectangleArena.proto",
    "projects/robots/gctronic/e-puck/protos/E-puck.proto",
]


def fetch(out, rel, fails):
    dst = out / rel
    if dst.exists():
        return dst.read_bytes()
    dst.parent.mkdir(parents=True, exist_ok=True)
    for attempt in range(3):
        try:
            data = urlopen(Request(BASE + rel, headers={"User-Agent": "curl/8"}), timeout=60).read()
            dst.write_bytes(data)
            return data
        except Exception as exc:
            if attempt == 2:
                fails.append((rel, str(exc)))
                return None
            time.sleep(1.5 * (attempt + 1))


def main(out):
    seen, queue, fails = set(), SEEDS + TEMPLATED, []
    while queue:
        rel = os.path.normpath(queue.pop(0).lstrip("/"))
        if rel in seen:
            continue
        seen.add(rel)
        data = fetch(out, rel, fails)
        if data is None or not rel.endswith(".proto"):
            continue
        text = data.decode("utf-8", "ignore")
        queue += [m for m in URL_RE.findall(text) if m not in seen and not UNSAFE.search(m)]
        for m in REL_RE.findall(text):
            if not m.startswith(("http", "webots://")) and not UNSAFE.search(m):
                queue.append(os.path.join(os.path.dirname(rel), m))
    print(f"vendored {len(seen) - len(fails)} files into {out}")
    for rel, err in fails:
        print("FAIL", rel, err)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main(Path(sys.argv[1])))
