"""Target classifier: DINOv3 ViT-S/16 fine-tuned on the eight reference images.

Two details matter more than the architecture:

* The whole frame is fed to the model. A centre crop drops the poster as soon as the
  robot is turned away from the station, which costs ~20 points once the heading error
  passes 20 degrees.
* The model was trained on view-matched crops of the reference images (lower part only,
  darkened), because the camera sits low and half the stations are in shadow.
"""

from dataclasses import dataclass
from pathlib import Path

import timm
import torch
from torchvision import transforms as T

ARCH = "vit_small_patch16_dinov3.lvd1689m"
WEIGHTS = Path(__file__).with_name("dinov3_s.pt")

# The model is trained with label smoothing, so the probabilities are deliberately flat:
# a correct frame scores ~0.24, not ~0.9. On 632 captured frames the 5th percentile of
# target frames was 0.215 and the 95th percentile of distractor frames 0.148, so this
# threshold rejects every distractor while passing 95% of targets (AUROC 0.999).
CONFIDENCE_THRESHOLD = 0.20

# Fixed at training time. Do not reorder.
CLASSES = (
    "soda_can",
    "coffee_mug",
    "backpack",
    "fire_extinguisher",
    "camera",
    "running_shoe",
    "headphones",
    "wall_clock",
)


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float


class Detector:
    """Classifies one camera frame (H x W x 3, RGB uint8) as one of CLASSES."""

    # <<<<<<< HEAD
    def __init__(
        self, weights: Path = WEIGHTS, device: str | None = None, threads: int = 4
    ):
        torch.set_num_threads(threads)
        self.device = torch.device(
            device or ("cuda" if torch.cuda.is_available() else "cpu")
        )
        model = timm.create_model(
            ARCH, pretrained=False, num_classes=len(CLASSES), img_size=224
        )
        state = torch.load(
            weights, map_location="cpu"
        )  # stored as fp16 to halve the file
        model.load_state_dict({k: v.float() for k, v in state.items()})
        self.model = model.eval().to(self.device)
        self.transform = T.Compose(
            [
                T.ToPILImage(),
                T.Resize((224, 224), interpolation=T.InterpolationMode.BICUBIC),
                T.ToTensor(),
                T.Normalize(
                    timm.data.IMAGENET_DEFAULT_MEAN, timm.data.IMAGENET_DEFAULT_STD
                ),
            ]
        )

    @torch.no_grad()
    def scores(self, image) -> torch.Tensor:
        """Per-class probabilities, in CLASSES order."""
        x = self.transform(image).unsqueeze(0).to(self.device)
        return self.model(x).softmax(1)[0].cpu()

    def detect(self, image) -> Detection:
        probabilities = self.scores(image)
        index = int(probabilities.argmax())
        return Detection(CLASSES[index], float(probabilities[index]))

    def detect_best(self, images) -> Detection:
        """Most confident frame of several, e.g. a short left/right scan at a station.

        Scanning +/-20 degrees and keeping this frame is worth ~10 points whenever the
        robot arrives with a heading error; a single frame is fine when it arrives square.
        """
        return max((self.detect(image) for image in images), key=lambda d: d.confidence)
