import numpy as np

with open('JointPos.txt') as f:
    contents = f.read().splitlines()
    print(contents[0])
    arr = np.array(contents)
    print(arr[0][0])