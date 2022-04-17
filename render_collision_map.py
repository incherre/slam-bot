'''A util to render saved collision maps.'''

import os
from math import inf
from PIL import Image, ImageDraw

from collision_map import CollisionMap

def image_from_collision_map(input_collision_map, image_scale = 50):
    max_x = -inf
    min_x = inf
    max_y = -inf
    min_y = inf
    max_stepped = 0
    max_missed = 0
    max_hit = 0

    for [x, y], location in input_collision_map.map.items():
        if x > max_x:
            max_x = x
        if x < min_x:
            min_x = x
        if y > max_y:
            max_y = y
        if y < min_y:
            min_y = y
        if location.stepped_count > max_stepped:
            max_stepped = location.stepped_count
        if location.missed_count > max_missed:
            max_missed = location.missed_count
        if location.hit_count > max_hit:
            max_hit = location.hit_count

    x_len = (max_x - min_x) // input_collision_map.scale
    y_len = (max_y - min_y) // input_collision_map.scale
    stepped_multiplier = 255 / max_stepped
    missed_multiplier = 255 / max_missed
    hit_multiplier = 255 / max_hit

    rendered_map = Image.new("RGB", ((x_len + 1) * image_scale, (y_len + 1) * image_scale))
    drawing_util = ImageDraw.Draw(rendered_map)
    for [x, y], location in input_collision_map.map.items():
        base_pos = ((x - min_x) // input_collision_map.scale,
                    y_len - ((y - min_y) // input_collision_map.scale))
        first_corner = (base_pos[0] * image_scale, base_pos[1] * image_scale)
        second_corner = (first_corner[0] + image_scale, first_corner[1] + image_scale)
        color = (int(location.hit_count * hit_multiplier),
                 int(location.stepped_count * stepped_multiplier),
                 int(location.missed_count * missed_multiplier))
        drawing_util.rectangle([first_corner, second_corner], fill = color)

    return rendered_map

if __name__ == "__main__":
    filepath = os.path.join(".", "results", "tk_sim_collision_map.txt")

    f = open(filepath, "r")
    map_from_file = CollisionMap.from_string(f.read())
    f.close()

    map_image = image_from_collision_map(map_from_file)
    outfile_path = os.path.splitext(filepath)[0] + ".png"
    map_image.save(outfile_path)
