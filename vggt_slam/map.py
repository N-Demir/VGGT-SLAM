import os
import numpy as np
import torch
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from read_write_model import Camera, Image, Point3D, write_model, rotmat2qvec

class GraphMap:
    def __init__(self):
        self.submaps = dict()
    
    def get_num_submaps(self):
        return len(self.submaps)

    def add_submap(self, submap):
        submap_id = submap.get_id()
        self.submaps[submap_id] = submap
    
    def get_largest_key(self):
        if len(self.submaps) == 0:
            return -1
        return max(self.submaps.keys())
    
    def get_submap(self, id):
        return self.submaps[id]

    def get_latest_submap(self):
        return self.get_submap(self.get_largest_key())
    
    def retrieve_best_score_frame(self, query_vector, current_submap_id, ignore_last_submap=True):
        overall_best_score = 1000
        overall_best_submap_id = 0
        overall_best_frame_index = 0
        # search for best image to target image
        for submap_key in self.submaps.keys():
            if submap_key == current_submap_id:
                continue

            if ignore_last_submap and (submap_key == current_submap_id-1):
                continue

            else:
                submap = self.submaps[submap_key]
                submap_embeddings = submap.get_all_retrieval_vectors()
                scores = []
                for embedding in submap_embeddings:
                    score = torch.linalg.norm(embedding-query_vector)
                    scores.append(score.item())
                
                best_score_id = np.argmin(scores)
                best_score = scores[best_score_id]

                if best_score < overall_best_score:
                    overall_best_score = best_score
                    overall_best_submap_id = submap_key
                    overall_best_frame_index = best_score_id

        return overall_best_score, overall_best_submap_id, overall_best_frame_index

    def get_frames_from_loops(self, loops):
        frames = []
        for detected_loop in loops:
            frames.append(self.submaps[detected_loop.detected_submap_id].get_frame_at_index(detected_loop.detected_submap_frame))
        
        return frames
    
    def update_submap_homographies(self, graph):
        for submap_key in self.submaps.keys():
            submap = self.submaps[submap_key]
            submap.set_reference_homography(graph.get_homography(submap_key).matrix())
    
    def get_submaps(self):
        return self.submaps.values()

    def ordered_submaps_by_key(self):
        for k in sorted(self.submaps):
            yield self.submaps[k]

    def write_poses_to_file(self, file_name):
        with open(file_name, "w") as f:
            for submap in self.ordered_submaps_by_key():
                poses = submap.get_all_poses_world(ignore_loop_closure_frames=True)
                frame_ids = submap.get_frame_ids()
                assert len(poses) == len(frame_ids), "Number of provided poses and number of frame ids do not match"
                for frame_id, pose in zip(frame_ids, poses):
                    x, y, z = pose[0:3, 3]
                    rotation_matrix = pose[0:3, 0:3]
                    quaternion = R.from_matrix(rotation_matrix).as_quat() # x, y, z, w
                    output = np.array([float(frame_id), x, y, z, *quaternion])
                    f.write(" ".join(f"{v:.8f}" for v in output) + "\n")

    def save_framewise_pointclouds(self, file_name):
        os.makedirs(file_name, exist_ok=True)
        for submap in self.ordered_submaps_by_key():
            pointclouds, frame_ids, conf_masks = submap.get_points_list_in_world_frame(ignore_loop_closure_frames=True)
            for frame_id, pointcloud, conf_masks in zip(frame_ids, pointclouds, conf_masks):
                # save pcd as numpy array
                np.savez(f"{file_name}/{frame_id}.npz", pointcloud=pointcloud, mask=conf_masks)
                

    def write_points_to_file(self, file_name):
        pcd_all = []
        colors_all = []
        for submap in self.ordered_submaps_by_key():
            pcd = submap.get_points_in_world_frame()
            pcd = pcd.reshape(-1, 3)
            pcd_all.append(pcd)
            colors_all.append(submap.get_points_colors())
        pcd_all = np.concatenate(pcd_all, axis=0)
        colors_all = np.concatenate(colors_all, axis=0)
        if colors_all.max() > 1.0:
            colors_all = colors_all / 255.0
        pcd_all = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_all))
        pcd_all.colors = o3d.utility.Vector3dVector(colors_all)
        o3d.io.write_point_cloud(file_name, pcd_all)

    def write_colmap_format(self, output_dir, image_names=None):
        """
        Export reconstruction in COLMAP format using read_write_model.py functions.
        Uses the same strategy as save_framewise_pointclouds for point cloud processing.
        
        Args:
            output_dir: Directory to save COLMAP files
            image_names: List of image file names (optional, for better image names)
        """
        os.makedirs(output_dir, exist_ok=True)
        
        # Collect all data
        cameras = {}
        images = {}
        points3D = {}
        
        image_id = 1
        point_id = 1
        
        for submap in self.ordered_submaps_by_key():
            # Get camera poses and intrinsics
            poses = submap.get_all_poses_world(ignore_loop_closure_frames=True)
            frame_ids = submap.get_frame_ids()
            intrinsics = submap.vggt_intrinscs
            
            # Get image names
            if image_names is None:
                image_names = [f"frame_{int(fid)}.jpg" for fid in frame_ids]
            
            # Process each frame
            for i, (pose, frame_id, intrinsic) in enumerate(zip(poses, frame_ids, intrinsics)):
                # Camera parameters
                fx, fy = intrinsic[0, 0], intrinsic[1, 1]
                cx, cy = intrinsic[0, 2], intrinsic[1, 2]
                width, height = intrinsic[0, 2] * 2, intrinsic[1, 2] * 2  # Approximate
                
                # Convert pose to quaternion using read_write_model function
                rotation_matrix = pose[0:3, 0:3]
                qvec = rotmat2qvec(rotation_matrix)  # Returns (qw, qx, qy, qz)
                
                # Translation
                tvec = pose[0:3, 3]
                
                # Add camera if not already added
                camera_id = 1  # Single camera model
                if camera_id not in cameras:
                    cameras[camera_id] = Camera(
                        id=camera_id,
                        model="SIMPLE_PINHOLE",
                        width=int(width),
                        height=int(height),
                        params=np.array([fx, cx, cy])
                    )
                
                # Add image
                image_name = image_names[i] if i < len(image_names) else f"frame_{int(frame_id)}.jpg"
                images[image_id] = Image(
                    id=image_id,
                    qvec=qvec,
                    tvec=tvec,
                    camera_id=camera_id,
                    name=image_name,
                    xys=np.empty((0, 2)),  # No 2D points
                    point3D_ids=np.empty(0, dtype=int)  # No 3D point correspondences
                )
                
                image_id += 1
            
            # Get point clouds using the same strategy as save_framewise_pointclouds
            try:
                pointclouds, frame_ids_pc, conf_masks = submap.get_points_list_in_world_frame(ignore_loop_closure_frames=True)
                
                for frame_idx, (pointcloud, frame_id_pc, conf_mask) in enumerate(zip(pointclouds, frame_ids_pc, conf_masks)):
                    # Apply confidence mask to filter points
                    valid_mask = conf_mask >= submap.get_conf_threshold()
                    
                    if not np.any(valid_mask):
                        continue
                    
                    # Get valid points and colors
                    points = pointcloud[valid_mask].reshape(-1, 3)
                    
                    # Get colors for this frame (assuming colors are stored in submap)
                    colors = submap.colors[frame_idx][valid_mask].reshape(-1, 3)
                    
                    # Add all points
                    for point, color in zip(points, colors):
                        points3D[point_id] = Point3D(
                            id=point_id,
                            xyz=point,
                            rgb=np.array([int(color[0]), int(color[1]), int(color[2])]),
                            error=0.0,  # No error metric available
                            image_ids=np.empty(0, dtype=int),  # No track information
                            point2D_idxs=np.empty(0, dtype=int)  # No track information
                        )
                        point_id += 1
                    
            except Exception as e:
                print(f"Warning: Could not process point cloud for submap {submap.get_id()}: {e}")
        
        # Write COLMAP model using read_write_model functions (always text format)
        write_model(cameras, images, points3D, output_dir, ext=".txt")
        
        print(f"COLMAP format exported to {output_dir}")
        print(f"  - {len(cameras)} cameras")
        print(f"  - {len(images)} images")
        print(f"  - {len(points3D)} 3D points")