
from timer import Timer
import numpy as np
from dvrk_planning.utilities import convert_mat_to_frame

mat = np.mat([[1, 0, 0, 0.123],
                [0, 1, 0, 0.3],
                [0, 0, 1, 12],
                [0, 0, 0, 1]], dtype=float)
print(mat)
t = Timer()

for i in range(10):
    t.start()
    frame = convert_mat_to_frame(mat)
    t.stop()
    print(t.str_elapsed_time())
