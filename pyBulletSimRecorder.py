"""
PyBullet Simulation Recorder (no external deps)

Records link and body poses from a running PyBullet simulation and saves them
to a .pkl file for later import into Blender or other visualisation tools.
Includes metadata such as FPS, frame count, and seed.

Author: Giorgio Clivio
License: GNU General Public License v3 (GPLv3)
Repository: https://github.com/GiorgioClivio/PyBullet-2-Blender-Simulation-Importer
"""

import pybullet as p
import pickle
import numpy as np
import xml.etree.ElementTree as ET
import os
import time
from os.path import abspath, dirname, basename, join
from transforms3d.quaternions import mat2quat
from transforms3d.affines import decompose


class PyBulletRecorder:
    """Dependency-free simulation recorder for PyBullet."""

    class LinkTracker:
        """Tracks a single link or base for recording."""
        def __init__(self, name, body_id, link_id, link_origin, mesh_path, mesh_scale):
            self.name = name
            self.body_id = body_id
            self.link_id = link_id
            self.mesh_path = mesh_path
            self.mesh_scale = mesh_scale

            # Decompose 4x4 transform into translation + quaternion
            decomposed_origin = decompose(link_origin)
            orn = mat2quat(decomposed_origin[1])
            orn = [orn[1], orn[2], orn[3], orn[0]]  # reorder w,x,y,z → x,y,z,w
            self.link_pose = [decomposed_origin[0], orn]

        def transform(self, position, orientation):
            return p.multiplyTransforms(position, orientation,
                                        self.link_pose[0], self.link_pose[1])

        def get_keyframe(self):
            """Return a position/orientation pair for this link."""
            if self.link_id == -1:
                position, orientation = p.getBasePositionAndOrientation(self.body_id)
                position, orientation = self.transform(position, orientation)
            else:
                link_state = p.getLinkState(self.body_id, self.link_id, computeForwardKinematics=True)
                position, orientation = self.transform(link_state[4], link_state[5])
            return {'position': list(position), 'orientation': list(orientation)}

    def __init__(self, fps=240, seed=None):
        self.links = []
        self.states = []
        self.fps = fps
        self.seed = seed

    # -------------------------------------------------------------
    # Registration
    # -------------------------------------------------------------
    def register_object(self, body_id, urdf_path, global_scaling=1.0):
        """
        Parses a URDF and registers all visual links for recording.
        """
        link_id_map = {-1: p.getBodyInfo(body_id)[0].decode("utf-8")}
        n_joints = p.getNumJoints(body_id)
        for i in range(n_joints):
            link_name = p.getJointInfo(body_id, i)[12].decode("utf-8")
            link_id_map[link_name] = i

        dir_path = dirname(abspath(urdf_path))
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        for link in root.findall('link'):
            name = link.get('name')
            visuals = link.findall('visual')
            for i, vis in enumerate(visuals):
                geom = vis.find('geometry')
                if geom is None:
                    continue
                mesh = geom.find('mesh')
                if mesh is None or 'filename' not in mesh.attrib:
                    continue

                mesh_path = join(dir_path, mesh.attrib['filename'])
                scale_str = mesh.attrib.get('scale', '1 1 1').split()
                scale = [float(x) * global_scaling for x in scale_str]

                self.links.append(PyBulletRecorder.LinkTracker(
                    name=f"{basename(urdf_path)}_{body_id}_{name}_{i}",
                    body_id=body_id,
                    link_id=link_id_map.get(name, -1),
                    link_origin=np.identity(4),
                    mesh_path=mesh_path,
                    mesh_scale=scale
                ))

    # -------------------------------------------------------------
    # Recording
    # -------------------------------------------------------------
    def add_keyframe(self):
        """Capture the transform of all registered links."""
        current_state = {}
        for link in self.links:
            current_state[link.name] = link.get_keyframe()
        self.states.append(current_state)
    
        # Print progress every 1000 frames
        total = len(self.states)
        if total % 1000 == 0:
            print(f"[Recorder] Captured frame {total:,}")

    def get_formatted_output(self):
        """Reformat raw states into a serializable dict."""
        out = {}
        for link in self.links:
            out[link.name] = {
                'type': 'mesh',
                'mesh_path': link.mesh_path,
                'mesh_scale': link.mesh_scale,
                'frames': [s[link.name] for s in self.states]
            }
        return out

    # -------------------------------------------------------------
    # Saving
    # -------------------------------------------------------------
    def save(self, path):
        """Save recording to .pkl file with metadata."""
        print(f"[Recorder] Saving to {path}")
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            data = self.get_formatted_output()

            total_frames = max(len(obj["frames"]) for obj in data.values())
            metadata = {
                "version": "1.1",
                "fps": self.fps,
                "num_frames": total_frames,
                "num_objects": len(data),
                "seed": self.seed,
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            }

            to_save = {"metadata": metadata, "data": data}
            with open(path, "wb") as f:
                pickle.dump(to_save, f)

            print(f"[Recorder] Saved successfully "
                  f"({metadata['num_frames']} frames at {metadata['fps']} Hz)")

        except Exception as e:
            print(f"[Recorder] Error while saving recording: {e}")
