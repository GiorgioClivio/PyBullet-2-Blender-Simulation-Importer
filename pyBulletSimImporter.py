"""
PyBullet Simulation Importer (Blender Add-on)

Imports recorded PyBullet simulations (.pkl files) into Blender as animated objects,
with support for range selection, frame decimation, playback speed remapping, and 
optional terrain mesh import (STL).

This add-on allows researchers and developers to visualise PyBullet simulation
results in Blender for high-quality rendering and analysis, enabling playback 
of robotic motion with adjustable temporal resolution and camera angles.

Author: Giorgio Clivio
Version: 0.3.0
Blender: 4.5+
License: GNU General Public License v3 (GPLv3)
Repository: https://github.com/GiorgioClivio/PyBullet-2-Blender-Simulation-Importer
"""


bl_info = {
    "name": "PyBullet Simulation Importer (v3)",
    "author": "Giorgio Clivio",
    "version": (0, 3, 0),
    "blender": (4, 5, 0),
    "location": "View3D > Sidebar > PyBullet",
    "description": "PyBullet .pkl importer with range selection, decimation, FPS remap, and terrain import.",
    "category": "Import-Export",
}

import bpy, pickle, os
from bpy.props import *
from bpy.types import Operator, Panel, PropertyGroup
from bpy_extras.io_utils import ImportHelper


# -------------------------------------------------------------------
# Utility Functions
# -------------------------------------------------------------------

def read_pkl_metadata(filepath):
    """Read simulation metadata (fps and frame count) from the .pkl file."""
    try:
        with open(filepath, "rb") as f:
            blob = pickle.load(f)

        sim_hz = 240
        num_frames = 0

        if isinstance(blob, dict):
            meta = blob.get("metadata", {})
            if isinstance(meta, dict):
                sim_hz = int(meta.get("fps", sim_hz))
                num_frames = int(meta.get("num_frames", num_frames))
            if num_frames == 0 and len(blob) > 0:
                first = next(iter(blob.values()))
                if isinstance(first, dict) and "frames" in first:
                    num_frames = len(first["frames"])

        return {"fps": sim_hz, "num_frames": num_frames}

    except Exception as e:
        print(f"[PyBulletImporter] Metadata read failed: {e}")
        return None


def import_mesh(filepath, name, scale=(1,1,1)):
    """Import STL/OBJ/DAE mesh and return Blender object (joined)."""
    if not os.path.exists(filepath):
        print(f"[PyBulletImporter] ❌ Mesh not found: {filepath}")
        return None

    ext = os.path.splitext(filepath)[1].lower()
    bpy.ops.object.select_all(action='DESELECT')

    try:
        if ext == ".stl":
            if hasattr(bpy.ops.wm, "stl_import"):
                bpy.ops.wm.stl_import(filepath=filepath)
            else:
                bpy.ops.import_mesh.stl(filepath=filepath)
        elif ext == ".obj":
            bpy.ops.import_scene.obj(filepath=filepath, axis_forward='Y', axis_up='Z')
        elif ext == ".dae":
            bpy.ops.wm.collada_import(filepath=filepath)
        else:
            print(f"[PyBulletImporter] ⚠️ Unsupported format: {ext}")
            return None
    except Exception as e:
        print(f"[PyBulletImporter] ❌ Mesh import failed for {name}: {e}")
        return None

    objs = [o for o in bpy.context.selected_objects if o.type == 'MESH']
    if not objs:
        print(f"[PyBulletImporter] ⚠️ No mesh imported for {name}")
        return None

    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()

    obj = bpy.context.view_layer.objects.active
    obj.name = name
    obj.scale = scale
    return obj


# -------------------------------------------------------------------
# Properties
# -------------------------------------------------------------------

class PyBulletImporterProperties(PropertyGroup):

    # --- SOURCE / METADATA ---

    pkl_path: StringProperty(
        name="Simulation File (.pkl)",
        subtype='FILE_PATH',
        description="Path to the recorded PyBullet simulation file",
        update=lambda self, ctx: PyBulletImporterProperties.on_pkl_path_changed(self, ctx)
    )

    sim_freq_hz: FloatProperty(
        name="Sim Frequency (Hz)",
        default=240.0,
        description="Tick rate / physics step rate of the simulation"
    )

    num_frames_detected: IntProperty(
        name="Detected Frames",
        default=0,
        description="Number of frames found in the .pkl file"
    )

    # --- RANGE SELECTION ---

    import_start_frame: IntProperty(
        name="Start Frame",
        default=0,
        min=0,
        description="First simulation frame index to import"
    )

    import_end_frame: IntProperty(
        name="End Frame",
        default=-1,
        min=-1,
        description="Last simulation frame index (-1 = import all)"
    )

    # --- PLAYBACK / REMAP ---

    remap_fps: EnumProperty(
        name="Target FPS",
        items=[
            ('24', "24 FPS", "Cinema-style timing"),
            ('30', "30 FPS", "Common real-time playback"),
            ('60', "60 FPS", "High framerate preview"),
        ],
        default='30',
        description="Blender scene FPS for playback"
    )

    preserve_realtime: BoolProperty(
        name="Preserve Real-Time Playback",
        default=True,
        description="Match Blender playback to physical sim time"
    )

    # --- DECIMATION ---

    frame_threshold: IntProperty(
        name="Too Many Frame Threshold",
        default=5000,
        min=100,
        description="If the .pkl exceeds this many frames, decimation is recommended"
    )

    decimate_keep_percent: FloatProperty(
        name="Decimation (%)",
        default=100.0,
        min=1.0,
        max=100.0,
        subtype='PERCENTAGE',
        description="Import fewer frames for faster conversion (100% = all)"
    )

    # --- TERRAIN ---

    terrain_path: StringProperty(
        name="Terrain STL",
        subtype='FILE_PATH',
        description="Path to the STL mesh of the terrain to import"
    )

    # --- callback ---
    @staticmethod
    def on_pkl_path_changed(self, context):
        path = self.pkl_path
        if not path or not os.path.exists(path):
            self.num_frames_detected = 0
            return
        meta = read_pkl_metadata(path)
        if meta:
            self.sim_freq_hz = float(meta.get("fps", 240))
            self.num_frames_detected = int(meta.get("num_frames", 0))
            print(f"[PyBulletImporter] Detected {self.num_frames_detected} frames @ {self.sim_freq_hz} Hz")
        else:
            print("[PyBulletImporter] No metadata found")


# -------------------------------------------------------------------
# Operators
# -------------------------------------------------------------------

class IMPORT_OT_PyBulletSimulation(Operator):
    bl_idname = "import_scene.pybullet_sim"
    bl_label = "Import Simulation"
    bl_description = "Import a PyBullet simulation from .pkl"

    def execute(self, context):
        props = context.scene.pbullet_props
        path = props.pkl_path
        if not path or not os.path.exists(path):
            self.report({'ERROR'}, "No valid .pkl selected")
            return {'CANCELLED'}

        with open(path, "rb") as f:
            blob = pickle.load(f)
        data = blob.get("data", blob) if isinstance(blob, dict) else {}
        if not isinstance(data, dict) or len(data) == 0:
            self.report({'ERROR'}, "No valid data in .pkl")
            return {'CANCELLED'}

        coll_name = f"PyBulletImport_{os.path.basename(path)}"
        new_coll = bpy.data.collections.new(coll_name)
        bpy.context.scene.collection.children.link(new_coll)
        scene = bpy.context.scene
        blend_fps = int(props.remap_fps)
        scene.render.fps = blend_fps
        sim_hz = props.sim_freq_hz if props.sim_freq_hz > 0 else 240.0

        if props.preserve_realtime:
            print(f"[PyBulletImporter] Real-time ON: mapping {sim_hz} Hz → {blend_fps} fps")
        else:
            print(f"[PyBulletImporter] Real-time OFF: raw frame mapping")

        dec_pct = props.decimate_keep_percent
        step = max(1, int(round(100.0 / dec_pct))) if dec_pct < 100.0 else 1
        print(f"[PyBulletImporter] Using step={step} (keep {dec_pct:.1f}% of frames)")

        keys = list(data.keys())
        total_objects = len(keys)
        last_used_frame_number = 1

        for obj_index, key in enumerate(keys):
            entry = data[key]
            if not isinstance(entry, dict) or "frames" not in entry:
                continue
            mesh_path = entry.get("mesh_path", "")
            frames_all = entry.get("frames", [])
            scale = entry.get("mesh_scale", [1,1,1])
            if not frames_all:
                continue

            start_idx = max(0, props.import_start_frame)
            end_idx = len(frames_all) if props.import_end_frame < 0 else min(props.import_end_frame, len(frames_all))
            if start_idx >= end_idx:
                self.report({'ERROR'}, f"Invalid import range: start={start_idx}, end={end_idx}")
                return {'CANCELLED'}

            frames = frames_all[start_idx:end_idx]
            print(f"[PyBulletImporter] Importing {key} ({obj_index+1}/{total_objects}) frames {start_idx}-{end_idx}")

            obj = import_mesh(mesh_path, key, scale)
            if obj is None:
                continue
            new_coll.objects.link(obj)

            decimated_indices = list(range(0, len(frames), step))
            for i, f_idx in enumerate(decimated_indices):
                frame_data = frames[f_idx]
                pos, orn = frame_data["position"], frame_data["orientation"]
                sim_time = (f_idx + start_idx) / sim_hz
                frame_number = int(sim_time * blend_fps) if props.preserve_realtime else i
                scene.frame_set(frame_number)
                obj.location = pos
                obj.rotation_mode = 'QUATERNION'
                obj.rotation_quaternion = (orn[3], orn[0], orn[1], orn[2])
                obj.keyframe_insert("location", frame=frame_number)
                obj.keyframe_insert("rotation_quaternion", frame=frame_number)
                if i % 250 == 0 or i == len(decimated_indices)-1:
                    pct = (i / max(1, len(decimated_indices)-1)) * 100
                    msg = f"{key} ({obj_index+1}/{total_objects}) – {pct:.1f}%"
                    print(f"[PyBulletImporter] {msg}")
                last_used_frame_number = max(last_used_frame_number, frame_number)

        scene.frame_end = max(1, last_used_frame_number)
        print(f"[PyBulletImporter] Timeline set to {scene.frame_end} frames")
        self.report({'INFO'}, f"✅ Import complete ({total_objects} objects)")
        print(f"[PyBulletImporter] ✅ Import complete ({total_objects} objects)")
        return {'FINISHED'}


class IMPORT_OT_PyBulletTerrain(Operator):
    bl_idname = "import_scene.pybullet_terrain"
    bl_label = "Import Terrain"
    bl_description = "Import an STL terrain mesh into the scene"

    def execute(self, context):
        props = context.scene.pbullet_props
        path = props.terrain_path
        if not path or not os.path.exists(path):
            self.report({'ERROR'}, "No valid STL selected")
            return {'CANCELLED'}
        obj = import_mesh(path, "Terrain")
        if obj:
            bpy.context.scene.collection.objects.link(obj)
            print(f"[PyBulletImporter] ✅ Terrain imported: {path}")
            self.report({'INFO'}, f"✅ Terrain imported: {os.path.basename(path)}")
        else:
            self.report({'ERROR'}, "Failed to import terrain")
        return {'FINISHED'}


# -------------------------------------------------------------------
# UI Panel
# -------------------------------------------------------------------

class VIEW3D_PT_PyBulletImporter(Panel):
    bl_label = "PyBullet Simulation Importer"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'PyBullet'

    def draw(self, context):
        layout = self.layout
        props = context.scene.pbullet_props

        # --- BOX: Simulation Source ---
        box_src = layout.box()
        row = box_src.row()
        row.label(text="Simulation Source", icon='FILE_FOLDER')
        box_src.prop(props, "pkl_path", text="Simulation File (.pkl)")
        row_meta = box_src.row(align=True)
        row_meta.label(text=f"Frames: {props.num_frames_detected}")
        row_meta.label(text=f"{props.sim_freq_hz:.0f} Hz")

        box_src.label(text="Import Range", icon='ARROW_LEFTRIGHT')
        col_rng = box_src.column(align=True)
        col_rng.prop(props, "import_start_frame", text="Start Frame")
        col_rng.prop(props, "import_end_frame", text="End Frame")

        box_src.label(text="Playback Mapping", icon='SEQUENCE')
        col_map = box_src.column(align=True)
        col_map.prop(props, "remap_fps", text="Target FPS")
        col_map.prop(props, "preserve_realtime", text="Preserve Real-Time")

        # --- BOX: Decimation ---
        if props.num_frames_detected > props.frame_threshold:
            box_dec = layout.box()
            row = box_dec.row()
            row.label(text="Decimation Options", icon='MOD_DECIM')
            box_dec.prop(props, "frame_threshold", text="Frame Threshold")
            box_dec.prop(props, "decimate_keep_percent", text="Decimation (%)", slider=True)

        # --- BOX: Import Controls ---
        box_imp = layout.box()
        row = box_imp.row()
        row.label(text="Import", icon='IMPORT')
        box_imp.operator(IMPORT_OT_PyBulletSimulation.bl_idname, icon='IMPORT', text="Import Simulation")

        # --- BOX: Terrain ---
        box_terr = layout.box()
        row = box_terr.row()
        row.label(text="Terrain", icon='MESH_GRID')
        box_terr.prop(props, "terrain_path", text="STL File")
        box_terr.operator("import_scene.pybullet_terrain", icon='IMPORT', text="Import Terrain")


# -------------------------------------------------------------------
# Registration
# -------------------------------------------------------------------

classes = (
    PyBulletImporterProperties,
    IMPORT_OT_PyBulletSimulation,
    IMPORT_OT_PyBulletTerrain,
    VIEW3D_PT_PyBulletImporter,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.pbullet_props = PointerProperty(type=PyBulletImporterProperties)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.pbullet_props

if __name__ == "__main__":
    register()
