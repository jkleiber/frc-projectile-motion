
import numpy as np

def compute_error(actual, target):
    actual_vec = np.array(actual)
    target_vec = np.array(target)

    return target_vec - actual_vec

