import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # For 3D plotting

def parse_tum_file(filename):
  timestamps = []
  positions = []
  orientations = [] # Quaternions (qx, qy, qz, qw)
  with open(filename, 'r') as f:
    for line in f:
      if line.strip(): # Skip empty lines
        data = line.strip().split()
        if len(data) >= 8: # Ensure line has enough data
          try:
            ts = float(data[0])
            x, y, z = float(data[1]), float(data[2]), float(data[3])
            qx, qy, qz, qw = float(data[4]), float(data[5]), float(data[6]), float(data[7])
            timestamps.append(ts)
            positions.append([x, y, z])
            orientations.append([qx, qy, qz, qw])
          except ValueError:
            print(f"Skipping line due to conversion error: {line}")
            continue
        else:
          print(f"Skipping line due to insufficient data: {line}")
          continue
  return np.array(timestamps), np.array(positions), np.array(orientations)

# --- Main Script ---
filename = 'poses_tum_coverted.txt' # Replace with your file path
timestamps, positions, orientations = parse_tum_file(filename)

if positions.size > 0:
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory path
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Trajectory Visualization')
    ax.legend()
    plt.show()

    # Optional: Plot 2D projections
    fig2, axs = plt.subplots(1, 3, figsize=(15, 5))
    axs[0].plot(positions[:, 0], positions[:, 1])
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    axs[0].set_title('Top View (X-Y)')
    axs[0].grid(True)
    axs[0].axis('equal') # Important for scale

    axs[1].plot(positions[:, 0], positions[:, 2])
    axs[1].set_xlabel('X')
    axs[1].set_ylabel('Z')
    axs[1].set_title('Side View (X-Z)')
    axs[1].grid(True)
    axs[1].axis('equal')

    axs[2].plot(positions[:, 1], positions[:, 2])
    axs[2].set_xlabel('Y')
    axs[2].set_ylabel('Z')
    axs[2].set_title('Front View (Y-Z)')
    axs[2].grid(True)
    axs[2].axis('equal')

    plt.tight_layout()
    plt.show()

else:
   print("No valid pose data found in the file.")
