from dataclasses import dataclass


@dataclass()
class IDetector:
    def detect(self, img): ...
