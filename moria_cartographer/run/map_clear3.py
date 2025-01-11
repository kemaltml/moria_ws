import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from PIL import Image
import yaml

def graham_scan(points):
    """
    Implement Graham's scan algorithm to find the convex hull of a set of 2D points.
    """
    # Sort points by x-coordinate, then by y-coordinate
    points = sorted(points, key=lambda p: (p[0], p[1]))

    def cross(o, a, b):
        """2D cross product of OA and OB vectors. Positive if counter-clockwise, negative if clockwise."""
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Build the lower hull
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build the upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenate lower and upper hulls (excluding the last point of each because it's repeated)
    return lower[:-1] + upper[:-1]

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

# Load the initial 3D point cloud
ply_file = "3dmap.ply"  # Use the original map as input
pcd = o3d.io.read_point_cloud(ply_file)

# Step 1: Downsample the point cloud to reduce memory usage
voxel_size = 0.05  # Adjust based on required accuracy
pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)

# Step 2: Filter ground points
filtered_points = []
filtered_colors = []
for i, (x, y, z) in enumerate(points):
    if -0.1 <= z <= 0.1:  # Retain ground-level points
        filtered_points.append([x, y, z])
        filtered_colors.append(colors[i])

# Step 3: Cluster remaining points for wall filtering
filtered_points_np = np.array(filtered_points)
remaining_mask = ~np.isin(points.view([('', points.dtype)] * points.shape[1]),
                          filtered_points_np.view([('', filtered_points_np.dtype)] * filtered_points_np.shape[1])).all(axis=1)
remaining_points = points[remaining_mask]
remaining_colors = colors[remaining_mask]

if len(remaining_points) > 0:
    db = DBSCAN(eps=0.5, min_samples=10).fit(remaining_points[:, :2])
    core_samples_mask = db.labels_ != -1
    clustered_points = remaining_points[core_samples_mask]
    clustered_colors = remaining_colors[core_samples_mask]

    filtered_points.extend(clustered_points)
    filtered_colors.extend(clustered_colors)

# Step 4: Compute convex hull using Graham's scan on clustered points
xy_points = np.array(filtered_points)[:, :2]
convex_hull = graham_scan(xy_points.tolist())

# Step 5: Extend the hull vertically to form walls
boundary_points = []
boundary_colors = []
for x, y in convex_hull:
    for z in np.linspace(-0.1, 2.0, num=20):  # Adjust height range as needed
        boundary_points.append([x, y, z])
        boundary_colors.append([0.8, 0.5, 0.2])  # Distinct color for walls

# Combine ground points, clustered walls, and boundary walls
all_points = np.vstack([filtered_points, boundary_points])
all_colors = np.vstack([filtered_colors, boundary_colors])

refined_pcd = o3d.geometry.PointCloud()
refined_pcd.points = o3d.utility.Vector3dVector(all_points)
refined_pcd.colors = o3d.utility.Vector3dVector(all_colors)

# Step 6: Perform additional optimization using statistical outlier removal
refined_pcd, _ = refined_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)

# Save the refined 3D map
output_ply_file = "refined_3dmap_optimized.ply"
o3d.io.write_point_cloud(output_ply_file, refined_pcd)

print(f"Refined and optimized 3D map saved to '{output_ply_file}'")
