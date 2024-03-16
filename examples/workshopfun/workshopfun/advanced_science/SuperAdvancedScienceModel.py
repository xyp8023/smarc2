#!/usr/bin/python3

import random
try:
    from .IRPMModel import IRPMModel
except:
    from IRPMModel import IRPMModel

class SuperAdvancedScienceModel(IRPMModel):
    def __init__(self, range: int):
        self._range = range

    def set_range(self, range: int) -> None:
        self._range = range

    def computeRPM(self) -> int:
        return random.randint(-self._range, self._range)
    

if __name__ == "__main__":
    model = SuperAdvancedScienceModel(range=500)
    for i in range(10):
        print(model.computeRPM())

