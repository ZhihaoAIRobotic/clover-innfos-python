import numpy as np

Su0 = np.vstack([np.zeros([6, 6*(50-1)]), np.tril(np.kron(np.ones([50-1, 50-1]), np.eye(6) * 0.2))])
print(Su0)
print(Su0.shape)