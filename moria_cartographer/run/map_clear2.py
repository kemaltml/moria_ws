import numpy as np
import open3d as o3d
from PIL import Image
import yaml

# Load the 2D map
pgm_file = "my_map_2d.pgm"  # Replace with your actual .pgm file path
map_image = Image.open(pgm_file)
map_array = np.array(map_image)

# Load the 2D map metadata
yaml_file = "my_map_2d.yaml"  # Replace with your actual .yaml file path
with open(yaml_file, 'r') as yaml_file:
    map_metadata = yaml.safe_load(yaml_file)

resolution = map_metadata["resolution"]  # Resolution of the 2D map
origin = map_metadata["origin"]  # Origin of the map in [x, y, z]
occupied_thresh = map_metadata.get("occupied_thresh", 0.65)  # Default to 0.65 if not provided

# Load the 3D point cloud with RGB colors from .ply
ply_file = "3dmap.ply"  # Replace with your .ply file path
pcd = o3d.io.read_point_cloud(ply_file)
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)  # Preserve RGB colors

# Initialize lists to store the filtered 3D points and their colors
filtered_points = []
filtered_colors = []

# Debugging: Track points that are removed and retained
removed_points_count = 0
retained_points_count = 0

# Iterate over each 3D point in the point cloud
for i, point in enumerate(points):
    x, y, z = point[0], point[1], point[2]

    # Map the 3D [x, y] coordinate to the 2D map pixel
    map_x = int(round((x - origin[0]) / resolution))
    map_y = int(round((y - origin[1]) / resolution))

    # Check if the point lies within the bounds of the 2D map
    if 0 <= map_x < map_array.shape[1] and 0 <= map_y < map_array.shape[0]:
        # If the 2D map indicates free space (value < occupied_thresh), skip this point
        if map_array[map_y, map_x] < 255 * occupied_thresh:
            removed_points_count += 1
            continue

    # Keep the point and its color if it is not in free space
    filtered_points.append(point)
    filtered_colors.append(colors[i])
    retained_points_count += 1

# Debugging: Print summary of points filtered
print(f"Removed points: {removed_points_count}")
print(f"Retained points: {retained_points_count}")

# Create a new point cloud with the filtered points and colors
new_pcd = o3d.geometry.PointCloud()
new_pcd.points = o3d.utility.Vector3dVector(np.array(filtered_points))
new_pcd.colors = o3d.utility.Vector3dVector(np.array(filtered_colors))

# Apply additional filtering to clean up noise
new_pcd, ind = new_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# Save the cleaned point cloud back to .ply format
output_ply_file = "cleaned_3dmap.ply"
o3d.io.write_point_cloud(output_ply_file, new_pcd)

print(f"Filtered 3D map with preserved colors saved to '{output_ply_file}'")