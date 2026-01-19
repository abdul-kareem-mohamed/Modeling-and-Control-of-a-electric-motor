import numpy as np

A = np.array([[1, 2, 3],
              [4, 5, 6],
              [7, 8, 9]])

scalar = 0.5
col_index = 2 # Target the third column

# Select the column and multiply
# Note: Using [col_index] instead of col_index keeps it as a 2D matrix (N x 1)
# If you use col_index scalar, you get a 1D array.
column_matrix = A[:, [col_index]] * scalar

print("Scaled Column Matrix:\n", column_matrix)
print("Shape:", column_matrix.shape)
