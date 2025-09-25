import bpy
import os
import glob

# this script is tested with blender 4.3
# WARNING !! this script will erase your scene

# Parameters
TARGET_NUM_FACES = 24.0
FOLDER_PATH = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(FOLDER_PATH, "output")
EXTENSION = ".obj"


def safe_base_name(filename):
    # remove leading/trailing whitespace and split extension robustly
    base, _ = os.path.splitext(filename)
    return base.strip()

def decimate(obj):
    """Apply a decimation modifier to reduce face count."""
    nFaces = len(obj.data.polygons)
    if nFaces == 0:
        return
    ratio = TARGET_NUM_FACES / float(nFaces)
    mod = obj.modifiers.new(name="DecimateMod", type="DECIMATE")
    mod.ratio = ratio
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)


def load_obj(file):
    """Import an OBJ, clean, convex hull, decimate, then export."""
    filepath = os.path.join(FOLDER_PATH, file)

    # Import OBJ (Blender 4.x API)
    # ~ bpy.ops.wm.obj_import(filepath=filepath, forward_axis='X',up_axis='Z',)
    bpy.ops.wm.obj_import(filepath=filepath)

    # Get imported object (last selected)
    obj = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = obj

    # Clean geometry: delete faces and build convex hull
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.delete(type="EDGE_FACE")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.convex_hull()
    bpy.ops.object.mode_set(mode="OBJECT")

    # Decimate mesh
    decimate(obj)

    # Prepare output name and path
    name_base = safe_base_name(file)
    obj.name = f"{name_base}_reduced{EXTENSION}"
    # ~ name_base = 
    # ~ export_name = f"{name_base}_reduced{EXTENSION}"
    export_path = os.path.join(OUTPUT_PATH, f"{name_base}_reduced{EXTENSION}")

    # Export OBJ (Blender 4.x API)
    bpy.ops.wm.obj_export(
        filepath=export_path,
        export_selected_objects=True,
        export_uv=True,
        export_normals=True,
        export_materials=False,
        # ~ forward_axis='X',
        # ~ up_axis='Z',
    )

    # Clean up for next file
    bpy.ops.object.delete()


# --- Main script ---

# Clear the scene
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete()

# Ensure output dir exists
os.makedirs(OUTPUT_PATH, exist_ok=True)

# Change working dir to folder with OBJs
os.chdir(FOLDER_PATH)

# Process all OBJ files
for file in glob.glob(f"*{EXTENSION}"):
    load_obj(file)
