import airo_blender as ab
import bpy
import numpy as np
import objaverse

bpy.ops.object.delete()  # Delete default cube

# From:
# https://github.com/allenai/objaverse-rendering/blob/main/scripts/blender_script.py


# load the glb model
def load_object(object_path: str) -> None:
    """Loads a glb model into the scene."""
    if object_path.endswith(".glb"):
        bpy.ops.import_scene.gltf(filepath=object_path, merge_vertices=True, import_shading="FLAT")
    elif object_path.endswith(".fbx"):
        bpy.ops.import_scene.fbx(filepath=object_path)
    else:
        raise ValueError(f"Unsupported file type: {object_path}")
    return bpy.context.selected_objects[0]


lvis_annotations = objaverse.load_lvis_annotations()
table_uids = lvis_annotations["table"]


objects = objaverse.load_objects(
    uids=table_uids,
)

# lay all the model out in a grid:
for i, table_uid in enumerate(table_uids):
    table_object = load_object(objects[table_uid])

    # The logic below to improve the table scales does not work yet
    min_corner, max_corner = ab.axis_aligned_bounding_box(table_object)
    diagonal_length = np.linalg.norm(max_corner - min_corner)

    print(table_object.name, diagonal_length)

    if diagonal_length is np.nan:
        bpy.ops.object.delete()
        continue

    if diagonal_length > 2 and diagonal_length is not np.nan:
        table_object.scale = [1.0 / diagonal_length] * 3

    table_object.location = [i % 4, i // 4, 0]
