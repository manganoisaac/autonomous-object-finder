import os
import ssl
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import torch.nn as nn
import torchvision
from torch import optim, torch
from torchvision import datasets, models, transforms
from torchvision.io.image import decode_image
from torchvision.models import list_models
from torchvision.models.detection import (
    FasterRCNN_ResNet50_FPN_V2_Weights,
    fasterrcnn_resnet50_fpn_v2,
)
from torchvision.transforms.functional import to_pil_image
from torchvision.utils import draw_bounding_boxes

ssl._create_default_https_context = ssl._create_unverified_context

img = decode_image("images/backpack/target_backpack.png")

weights = FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
model = fasterrcnn_resnet50_fpn_v2(weights=weights, box_score_thresh=0.8)
model.eval()

preprocess = weights.transforms()


data_transforms = {
    "train": transforms.Compose(
        [
            transforms.RandAugment(),
            transforms.ColorJitter(),
            transforms.RandomPosterize(4),
            transforms.RandomSolarize(5),
            # transforms.RandomErasing(),
            transforms.RandomResizedCrop(224),
            transforms.RandomHorizontalFlip(),
            transforms.ToTensor(),
            transforms.Normalize([0.5, 0.5, 0.5], [0.2, 0.2, 0.2]),
        ]
    ),
    "val": transforms.Compose(
        [
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize([0.5, 0.5, 0.5], [0.2, 0.2, 0.2]),
        ]
    ),
}

data_dir = "./images"
image_dataset = datasets.ImageFolder(data_dir, transform=data_transforms["train"])
data_set_size = len(image_dataset)
class_names = image_dataset.classes
print(f"class names: {class_names}")
dataloader = torch.utils.data.DataLoader(
    image_dataset, batch_size=4, shuffle=True, num_workers=4
)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"using device: {device}")


def imshow(inp, title=None):
    inp: np.ndarray = inp.numpy().transpose((1, 2, 0))
    mean = [0.5, 0.5, 0.5]
    std = [0.2, 0.2, 0.2]
    inp = std * inp + mean
    inp = np.clip(inp, 0, 1)
    plt.imshow(inp)
    if title is not None:
        plt.title(title)
    plt.show()


inputs, classes = next(iter(dataloader))

out = torchvision.utils.make_grid(inputs)

imshow(out, [class_names[x] for x in classes])


def train_model(
    model, criterion, optimizer, dataloader, device, dataset_size, num_epochs=25
):
    model.train()
    for epoch in range(num_epochs):
        print(f"Epoch {epoch + 1} / {num_epochs - 1}")
        print("-" * 10)
        running_loss = 0.0
        running_corrects = 0

        for inputs, labels in dataloader:
            inputs = inputs.to(device)
            labels = labels.to(device)
            optimizer.zero_grad()
            outputs = model(inputs)
            _, preds = torch.max(outputs, 1)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item() * inputs.size(0)
            running_corrects += torch.sum(preds == labels.data)
        epoch_loss = running_loss / dataset_size
        epoch_acc = running_corrects.double() / dataset_size
        print(f"loss: {epoch_loss}")
        print(f"accuracty: {epoch_acc}")
    return model


model_ft = models.resnet18(weights="IMAGENET1K_V1")
num_ftrs = model_ft.fc.in_features

model_ft.fc = nn.Linear(num_ftrs, len(class_names))

model_ft = model_ft.to(device)

criterion = nn.CrossEntropyLoss()

optimixer_ft = optim.SGD(model_ft.parameters(), lr=0.001)

model_ft = train_model(
    model_ft, criterion, optimixer_ft, dataloader, device, data_set_size, 200
)

# save model
torch.save(model_ft, "detector_model.pth")
