import airo_blender as ab
import bpy
import numpy as np

from syncloth.materials.dish_towel import create_gridded_dish_towel_material, create_striped_dish_towel_material


def add_random_plain_material(object: bpy.types.Object):
    color = np.random.uniform(size=3)
    material = ab.add_material(plane, color, roughness=1.0)
    material.node_tree.nodes["Principled BSDF"].inputs["Sheen"].default_value = 1
    object.data.materials.append(material)


def add_random_striped_material(object: bpy.types.Object):
    # half the towel have vertical stripes and half have horizontal stripes
    vertical = np.random.uniform() < 0.5

    # 80% of the stripes towel get a white background with a random srtipe color.
    if np.random.uniform() < 0.8:
        background_color = (1.0, 1.0, 1.0, 1.0)
        stripe_color = (*np.random.uniform(size=3), 1)
    else:
        # the remaining towel get a random background color.
        # of these, 50% have a white stripe color and 50% have a random stripe color.
        background_color = (*np.random.uniform(size=3), 1)
        if np.random.uniform() < 0.5:
            stripe_color = (1.0, 1.0, 1.0, 1.0)
        else:
            stripe_color = (*np.random.uniform(size=3), 1)

    amount_of_stripes = np.random.geometric(p=0.1)
    stripe_width = np.random.uniform()

    material = create_striped_dish_towel_material(
        amount_of_stripes, stripe_width, stripe_color, background_color, vertical
    )
    object.data.materials.append(material)


def add_random_gridded_material(object: bpy.types.Object):
    amount_of_vertical_stripes = np.random.geometric(p=0.1)
    # 50% chance that there are the same amount of horizontal stripes as vertical stripes
    if np.random.uniform() < 0.5:
        amount_of_horizontal_stripes = amount_of_vertical_stripes
    else:
        amount_of_horizontal_stripes = np.random.geometric(p=0.1)

    # same logic for the width of the stripes
    vertical_stripe_width = np.random.uniform()
    if np.random.uniform() < 0.5:
        horizontal_stripe_width = vertical_stripe_width
    else:
        horizontal_stripe_width = np.random.uniform()

    # again same for the color of the stripes
    vertical_stripe_color = (*np.random.uniform(size=3), 1)
    if np.random.uniform() < 0.5:
        horizontal_stripe_color = vertical_stripe_color
    else:
        horizontal_stripe_color = (*np.random.uniform(size=3), 1)

    # intersection color is always random
    intersection_color = (*np.random.uniform(size=3), 1)

    if np.random.uniform() < 0.8:
        background_color = (1.0, 1.0, 1.0, 1.0)
    else:
        background_color = (*np.random.uniform(size=3), 1)
        # If background color is not white, 50% chance to turn the stripes white
        if np.random.uniform() < 0.5:
            vertical_stripe_color = (1.0, 1.0, 1.0, 1.0)
            horizontal_stripe_color = (1.0, 1.0, 1.0, 1.0)
            intersection_color = (1.0, 1.0, 1.0, 1.0)

    material = create_gridded_dish_towel_material(
        amount_of_vertical_stripes,
        amount_of_horizontal_stripes,
        vertical_stripe_width,
        horizontal_stripe_width,
        vertical_stripe_color,
        horizontal_stripe_color,
        intersection_color,
        background_color,
    )
    object.data.materials.append(material)


bpy.ops.object.delete()
rows = cols = 20

import time

for row in range(rows):
    for col in range(cols):
        start = time.time()
        bpy.ops.mesh.primitive_plane_add()
        plane = bpy.context.object
        plane.scale = (0.45, 0.45, 0.45)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        plane.location = (col, row, 0)

        # 10% of the towels get a random plain color.
        if np.random.uniform() < 0.1:
            add_random_plain_material(plane)
            continue

        # Of the remaining towels, 50% are striped
        if np.random.uniform() < 0.5:
            add_random_striped_material(plane)
            continue

        # The remaining towels are gridded.
        add_random_gridded_material(plane)
        end = time.time()
        print(end - start)
