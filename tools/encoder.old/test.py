# %% import
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

filename = 'data/KERISEv5/trap.csv'

raw = pd.read_csv(filename, delimiter='\t')
print(raw)

ch = 1
N = 2 ** 14
