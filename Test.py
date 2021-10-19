import numpy as np

n_r = np.zeros((4,2))
n_r[:, 0] = np.array([1, 0, 1, 0])
n_r = np.column_stack((n_r,[2,2,3,4]))
print(n_r)

a = np.array([1,2,3,4])
a = np.append(a,5)
print(a)