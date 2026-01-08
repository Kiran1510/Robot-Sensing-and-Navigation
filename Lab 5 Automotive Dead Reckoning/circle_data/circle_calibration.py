import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg

# Reading CSV file
df = pd.read_csv('circle_data_0_imu.csv')

# Extracting magnetometer data (X and Y only)
mag_x = df['mag_x'].values
mag_y = df['mag_y'].values

# Put into matrix
mag_raw = np.column_stack([mag_x, mag_y])

print("raw magnetometer statistics (Tesla):")
print(f"X: min={mag_x.min():.6f}, max={mag_x.max():.6f}, range={mag_x.max()-mag_x.min():.6f}")
print(f"Y: min={mag_y.min():.6f}, max={mag_y.max():.6f}, range={mag_y.max()-mag_y.min():.6f}")

# Converting tesla to milliGauss
scale_factor = 1e7

mag_raw_mG = mag_raw * scale_factor

print(f"\nraw data in milliGauss:")
print(f"X: min={mag_raw_mG[:, 0].min():.2f}, max={mag_raw_mG[:, 0].max():.2f}, range={mag_raw_mG[:, 0].max()-mag_raw_mG[:, 0].min():.2f}")
print(f"Y: min={mag_raw_mG[:, 1].min():.2f}, max={mag_raw_mG[:, 1].max():.2f}, range={mag_raw_mG[:, 1].max()-mag_raw_mG[:, 1].min():.2f}")

# Hard iron correction
hard_iron_offset_x = (mag_raw_mG[:, 0].min() + mag_raw_mG[:, 0].max()) / 2.0
hard_iron_offset_y = (mag_raw_mG[:, 1].min() + mag_raw_mG[:, 1].max()) / 2.0
hard_iron_offset = np.array([hard_iron_offset_x, hard_iron_offset_y])

mag_hard_corrected = mag_raw_mG - hard_iron_offset

print(f"\nhard iron offset (mG): [{hard_iron_offset[0]:.2f}, {hard_iron_offset[1]:.2f}]")
print(f"post hard iron correction:")
print(f"X: min={mag_hard_corrected[:, 0].min():.2f}, max={mag_hard_corrected[:, 0].max():.2f}")
print(f"Y: min={mag_hard_corrected[:, 1].min():.2f}, max={mag_hard_corrected[:, 1].max():.2f}")

# Soft iron correction using ellipse fitting
def fit_ellipse(data):
    x = data[:, 0]
    y = data[:, 1]
    
    # Direct least squares ellipse fitting
    # ax^2 + bxy + cy^2 = 1 (normalized and centered at origin)
    D = np.column_stack([x**2, x*y, y**2])
    
    # Solving using least squares
    ones = np.ones(len(x))
    
    # Using single variable decomp for numerical stability
    U, S, Vt = np.linalg.svd(D, full_matrices=False)
    
    # Solution that minimizes ||D*v - 1||^2
    v = Vt.T @ np.linalg.inv(np.diag(S)) @ U.T @ ones
    
    # Constructing the matrix
    A = np.array([[v[0], v[1]/2],
                  [v[1]/2, v[2]]])
    
    # Eigenvalue decomposition to get principal axes
    eigenvalues, eigenvectors = np.linalg.eigh(A)
    
    # Semi axes lengths
    a = 1.0 / np.sqrt(eigenvalues[0])
    b = 1.0 / np.sqrt(eigenvalues[1])
    
    print(f"\nellipse semi-axes: a={a:.2f} mG, b={b:.2f} mG")
    print(f"ellipse eccentricity: {np.sqrt(1 - min(a,b)**2/max(a,b)**2):.4f}")
    
    # Transformation matrix to convert ellipse to circle
    avg_radius = (a + b) / 2.0
    scale_x = avg_radius / a
    scale_y = avg_radius / b
    
    # Transformation in principal axis
    scale_matrix = np.diag([scale_x, scale_y])
    
    # Full transform to rotate to principal axes, scale, and rotate back
    soft_iron_matrix = eigenvectors @ scale_matrix @ eigenvectors.T
    
    return soft_iron_matrix, avg_radius

# Fitting ellipse to hard iron corrected data
soft_iron_matrix, expected_radius = fit_ellipse(mag_hard_corrected)

print(f"\nsoft iron correction matrix:")
print(soft_iron_matrix)
print(f"expected field magnitude: {expected_radius:.2f} mG")

# Applying soft iron correction
mag_corrected_mG = (soft_iron_matrix @ mag_hard_corrected.T).T

print(f"\ncorrected magnetometer values (milliGauss):")
print(f"X: min={mag_corrected_mG[:, 0].min():.2f}, max={mag_corrected_mG[:, 0].max():.2f}")
print(f"Y: min={mag_corrected_mG[:, 1].min():.2f}, max={mag_corrected_mG[:, 1].max():.2f}")
print(f"mean: [{np.mean(mag_corrected_mG[:, 0]):.2f}, {np.mean(mag_corrected_mG[:, 1]):.2f}]")

# Calculating radius statistics for corrected data
radii_corrected = np.sqrt(mag_corrected_mG[:, 0]**2 + mag_corrected_mG[:, 1]**2)
print(f"corrected radius: mean={np.mean(radii_corrected):.2f}, std={np.std(radii_corrected):.2f} mG")

# Creating plot
plt.figure(figsize=(12, 12))

# Plotting raw data in red
plt.scatter(mag_raw_mG[:, 0], mag_raw_mG[:, 1], 
            c='red', alpha=0.5, s=10, label='raw data (including car interference)', 
            edgecolors='none')

# Plot corrected data in blue
plt.scatter(mag_corrected_mG[:, 0], mag_corrected_mG[:, 1], 
            c='blue', alpha=0.5, s=10, label='corrected data', 
            edgecolors='none')

plt.xlabel('magnetometer X (mG)', fontsize=14)
plt.ylabel('magnetometer Y (mG)', fontsize=14)
plt.title('magnetometer calibration: hard and soft iron correction)', 
          fontsize=16)
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.legend(fontsize=12, loc='upper right')

# Add crosshairs at origin
plt.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.4)
plt.axvline(x=0, color='k', linestyle='--', linewidth=0.8, alpha=0.4)

# Add circles to show the expected radius
circle = plt.Circle((0, 0), expected_radius, fill=False, 
                    color='green', linestyle=':', linewidth=2, 
                    label=f'target circle (r={expected_radius:.0f} mG)', alpha=0.6)
plt.gca().add_patch(circle)

plt.tight_layout()
plt.savefig('fig_0.png', dpi=300, bbox_inches='tight')

print("\n" + "="*70)
print("plot saved as 'fig_0.png'")
print("="*70)

print("\n" + "="*70)
print("calibration parameters:")
print("="*70)
print(f"hard iron offset (mG): [{hard_iron_offset[0]:.2f}, {hard_iron_offset[1]:.2f}]")
print(f"hard iron offset (Tesla): [{hard_iron_offset[0]/1e7:.9f}, {hard_iron_offset[1]/1e7:.9f}]")
print(f"\nsoft iron correction matrix:")
print(soft_iron_matrix)
