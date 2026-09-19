from dataclasses import dataclass

import torchvision.models
from PIL import Image
from torch import torch
from torchvision import transforms

# model = torch.load("detector_model.pth", weights_only=False)
# model.eval()
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# print(f"model: {model}")
# model = model.to(device)
#
# transform = transforms.Compose(
#     [
#         transforms.Resize(256),
#         transforms.CenterCrop(244),
#         transforms.ToTensor(),
#         transforms.Normalize([0.5, 0.5, 0.5], [0.2, 0.2, 0.2]),
#     ]
# )
#
# img = Image.open("images/fire-extinguisher/target_fire_extinguisher.png").convert("RGB")
# img = transform(img)
# print(f"img shape: {img.shape}")
# img = img.unsqueeze(0)
# print(f"squeezed img: {img.shape}")
# img = img.to(device)
# output = model(img)
# print(f"output: {output[0]}")
# confidence, index = torch.max(output, 1)
# print(f"confidence: {confidence.item()}")
# print(f"index: {index.item()}")
# CLASSES = [
#     "backpack",
#     "camera",
#     "clock",
#     "coffee-mug",
#     "fire-extinguisher",
#     "headphones",
#     "running-shoes",
#     "soda",
# ]
# print(f"prediction label: {CLASSES[index]}")


@dataclass()
class Detector:
    model: any
    transform: transforms.Compose
    classes: list[str]
    device: torch.device

    def detect(self, img) -> str:
        img = self.transform(img)
        img = img.unsqueeze(0)
        img = img.to(self.device)
        out = self.model(img)[0]
        confidence, index = torch.max(out, 0)
        predicted_target = self.classes[index]
        return predicted_target
