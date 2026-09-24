from dataclasses import dataclass

import torchvision.models
from PIL import Image
from torch import torch
from torchvision import transforms


@dataclass()
class Detector:
    model: any
    transform: transforms.Compose
    classes: list[str]
    device: torch.device

    def detect(self, img) -> tuple[str, float]:
        img = self.transform(img)
        img = img.unsqueeze(0)
        img = img.to(self.device)
        out = self.model(img)[0]
        confidence, index = torch.max(out, 0)
        print(f"confidence: {confidence}")
        predicted_target = self.classes[index]
        return predicted_target, confidence.item()
