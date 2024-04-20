import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from tkinter import filedialog
import tkinter as tk

# Create a Tkinter window
root = tk.Tk()
root.withdraw()  # Hide the Tkinter window

# Prompt the user to select a file
file_path = filedialog.askopenfilename(title="Select data file", filetypes=[("Text files", "*.txt")])

# Load data from the selected file, skipping the header row
data = np.loadtxt(file_path, delimiter=",", skiprows=1)

# Extract latitude, longitude, and sos values
latitudes = data[:, 0]
longitudes = data[:, 1]
sos_values = data[:, -1]

# Define grid for interpolation
grid_x, grid_y = np.mgrid[min(latitudes):max(latitudes):100j, min(longitudes):max(longitudes):100j]

# Interpolate using scipy's griddata
grid_sos = griddata((latitudes, longitudes), sos_values, (grid_x, grid_y), method='linear')

# Plot the interpolated heatmap
plt.figure(figsize=(10, 8))
plt.imshow(grid_sos.T, extent=(min(longitudes), max(longitudes), min(latitudes), max(latitudes)), # Switched min/max
           origin='lower', cmap='coolwarm', alpha=0.9)
plt.colorbar(label='Sum of Squares (sos)')
plt.ylabel('Latitude')
plt.xlabel('Longitude')
plt.title('Interpolated Heatmap of Sum of Squares (sos)')
plt.show()
