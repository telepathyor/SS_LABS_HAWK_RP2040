import pandas as pd
import matplotlib.pyplot as plt
from tkinter import filedialog
import tkinter as tk
import numpy as np
from scipy.interpolate import griddata
from scipy.spatial import cKDTree

# Create custom interpolation functions
def custom_interpolation(x, y, z, xi, yi):
    points = np.column_stack((x, y))
    tree = cKDTree(points)
    zi = np.zeros(len(xi))

    for i in range(len(xi)):
        _, idx = tree.query([xi[i], yi[i]], k=3)  # Adjust k for desired neighbors
        zi[i] = np.mean(z[idx])

    return zi

def custom_interpolation_idf(x, y, z, xi, yi, power=3):
    zi = np.zeros(len(xi))
    for i in range(len(xi)):
        distances = np.sqrt((x - xi[i])**2 + (y - yi[i])**2)
        weights = 1 / (distances ** power)
        zi[i] = np.sum(z * weights) / np.sum(weights)
    return zi

# Create a Tkinter window (hidden)
root = tk.Tk()
root.withdraw()

# Open a file dialog to select the *.txt file
file_path = filedialog.askopenfilename(title="Select a TXT file", filetypes=[("Text files", "*.txt")])

if file_path:
    print("File selected:", file_path)  # Add this line to verify the file path

    # Read the CSV file into a DataFrame and specify column names
    df = pd.read_csv(file_path, names=['latitude', 'longitude', 'sos'], header=0)

    # Extract latitude, longitude, and signal value columns
    lat = df['latitude']
    long = df['longitude']
    sos = df['sos']

    # Define the grid for interpolation
    grid_x, grid_y = np.meshgrid(np.linspace(min(long), max(long), 100), np.linspace(min(lat), max(lat), 100))

    # Toggle between interpolation methods (custom_interpolation and custom_interpolation_idf)
    use_idf_interpolation = True  # Set this to True to use IDF, False for custom interpolation

    if use_idf_interpolation:
        grid_z = custom_interpolation_idf(long, lat, sos, grid_x.flatten(), grid_y.flatten(), power=2)
    else:
        grid_z = custom_interpolation(long, lat, sos, grid_x.flatten(), grid_y.flatten())

    grid_z = grid_z.reshape(grid_x.shape)

    # Clip values to ensure they are non-negative
    grid_z = np.clip(grid_z, 0, None)

    # Create the 2D heatmap or contour plot
    plt.figure(figsize=(10, 8))
    plt.contourf(grid_x, grid_y, grid_z, cmap='viridis', levels=60)
    plt.colorbar()
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('IDF Interpolation Plot' if use_idf_interpolation else 'Custom 2D Interpolation Plot')

    plt.show()
else:
    print("No file selected. Exiting.")
