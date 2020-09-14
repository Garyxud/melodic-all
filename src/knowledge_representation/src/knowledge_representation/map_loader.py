import os
from xml.etree import ElementTree as ElTree
import yaml
from PIL import Image
import re
from warnings import warn

text_el = "{http://www.w3.org/2000/svg}text"
circle_el = "{http://www.w3.org/2000/svg}circle"
line_el = "{http://www.w3.org/2000/svg}line"
poly_el = "{http://www.w3.org/2000/svg}polygon"
path_el = "{http://www.w3.org/2000/svg}path"
group_el = "{http://www.w3.org/2000/svg}g"
tspan_el = "{http://www.w3.org/2000/svg}tspan"

float_pattern = r"[-+]?\d*\.\d+|[-+]?\d+"
translation_pattern = r"^translate\(({})\,({})\)".format(float_pattern, float_pattern)


def populate_with_map_annotations(ltmc, map_name, points, poses, regions):
    """
    Inserts a map and supporting geometry into a knowledgebase. Any existing map by the name will be deleted

    Emits warnings for any annotation that can't be added, as when there are name collisions.
    :param ltmc: The knowledgebase to insert into
    :param map_name: Name of the map to create.
    :param points:
    :param poses:
    :param regions:
    :return: a tuple of counts of how many of each type of annotation were successfully inserted
    """
    # Wipe any existing map by this name
    map = ltmc.get_map(map_name)
    map.delete()
    map = ltmc.get_map(map_name)
    point_count = 0
    pose_count = 0
    region_count = 0

    for name, point in points:
        point = map.add_point(name, *point)
        if not point.is_valid():
            warn("Failed to add point '{}': {}".format(name, *point))
        else:
            point_count += 1

    for name, (p1_x, p1_y), (p2_x, p2_y) in poses:
        pose = map.add_pose(name, p1_x, p1_y, p2_x, p2_y)
        if not pose.is_valid():
            warn("Failed to add pose '{}': {}".format(name, *((p1_x, p1_y), (p2_x, p2_y))))
        else:
            pose_count += 1

    for name, points in regions:
        region = map.add_region(name, points)
        if not region.is_valid():
            warn("Failed to add region '{}': {}".format(name, *points))
        else:
            region_count += 1

    return point_count, pose_count, region_count


def load_map_from_yaml(path_to_yaml):
    """
    Attempt to load map annotations given a path to a YAML file. Emits warnings for issues with particular annotations.

    The PGM and the SVG will be loaded in the process. The SVG file to load is determined by the `annotations` key,
    or is an SVG with the same name as the YAML file.
    :param path_to_yaml:
    :return: a tuple of the name of the map and a nested tuple of points, poses and regions
    """
    parent_dir = os.path.dirname(path_to_yaml)
    yaml_name = os.path.basename(path_to_yaml).split(".")[0]
    with open(path_to_yaml) as map_yaml:
        map_metadata = yaml.load(map_yaml, Loader=yaml.FullLoader)

    image_path = os.path.join(parent_dir, map_metadata["image"])
    map_image = Image.open(image_path)
    map_metadata["width"] = map_image.size[0]
    map_metadata["height"] = map_image.size[1]

    if "annotations" not in map_metadata:
        # Fallback when no annotations key is given: look for an svg
        # file that has the same name as the yaml file
        annotation_path = os.path.join(parent_dir, yaml_name + ".svg")
    else:
        annotation_path = os.path.join(parent_dir, map_metadata["annotations"])

    if not os.path.isfile(annotation_path):
        # No annotations to load. Since you're trying to load annotations, this is probably an error of some sort
        warn("No annotation file found at {}".format(annotation_path))
        return yaml_name, None

    with open(annotation_path) as test_svg:
        svg_data = test_svg.readlines()
        svg_data = " ".join(svg_data)

    annotations = load_svg(svg_data)
    annotations = transform_to_map_coords(map_metadata, *annotations)
    return yaml_name, annotations


def load_svg(svg_data):
    tree = ElTree.fromstring(svg_data)
    parent_map = {c: p for p in tree.iter() for c in p}
    point_annotations = tree.findall(".//{}[@class='circle_annotation']".format(circle_el))
    point_names = tree.findall(".//{}[@class='circle_annotation']/../{}".format(circle_el, text_el))
    pose_annotations = tree.findall(".//{}[@class='pose_line_annotation']".format(line_el))
    pose_names = tree.findall(".//{}[@class='pose_line_annotation']/../{}".format(line_el, text_el))
    region_annotations = tree.findall(".//{}[@class='region_annotation']".format(poly_el))
    region_names = tree.findall(".//{}[@class='region_annotation']/../{}".format(poly_el, text_el))
    path_groups = tree.findall(".//{}[{}]".format(group_el, path_el))
    circle_groups = tree.findall(".//{}[{}]".format(group_el, circle_el))
    circle_groups = filter(lambda g: len(g.getchildren()) == 2, circle_groups)

    point_parents = map(parent_map.__getitem__, point_annotations)
    points = process_point_annotations(point_names, point_annotations, point_parents)
    pose_parents = map(parent_map.__getitem__, pose_annotations)
    poses = process_pose_annotations(pose_names, pose_annotations, pose_parents)
    region_parents = map(parent_map.__getitem__, region_annotations)
    regions = process_region_annotations(region_names, region_annotations, region_parents)

    # Messier extraction to get annotations stored as paths. These are from Inkscape or other regular editing tools.
    path_poses, path_regions = process_paths(path_groups)
    extra_points = []

    for group in circle_groups:
        circle, text = group.find(".//{}".format(circle_el)), group.find(".//{}".format(tspan_el))
        if "class" in circle.attrib:
            # This was probably created by the annotation tool. Already processed above
            continue
        name = text.text
        pixel_coord = float(circle.attrib["cx"]), float(circle.attrib["cy"])
        extra_points.append((name, pixel_coord))

    points += extra_points
    poses += path_poses
    regions += path_regions

    return points, poses, regions


def get_group_transform(group):
    if "transform" not in group.attrib:
        return 0, 0
    # We can only handle basic translate transforms right now
    translate_match = re.match(translation_pattern, group.attrib["transform"])
    if not translate_match:
        raise RuntimeError("Can't process because it has a complex transform: {}".format(group))
    else:
        return float(translate_match.group(1)), float(translate_match.group(2))


def get_text_from_group(group):
    # Inkscape tucks things in a tspan. Check that first
    text = group.find(".//{}".format(tspan_el))
    if text is None:
        text = group.find(".//{}".format(text_el))
    return text


def process_paths(path_groups):
    """
    Extracts pose and region annotations represented as paths

    :param path_groups: a list of groups each containing a text element and a path
    :return: a tuple of poses and regions
    """
    if len(path_groups) == 0:
        return [], []

    # SVG paths are specified in a rich language of segment commands:
    # https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/d
    # We'll use a new dependency to extract what we can
    from svgpathtools import parse_path, Line

    def is_line(path_part):
        return isinstance(path_part, Line)

    regions = []
    poses = []
    for group in path_groups:
        if len(group.getchildren()) != 2:
            # May want to print a warning here
            continue

        # We assume that the text was created in inkscape so the string will be in a tspan
        path, text = group.find(".//{}".format(path_el)), get_text_from_group(group)
        if text is None:
            warn("No text label found for path group: {}".format(group))
            continue
        name = text.text

        translate = 0, 0
        try:
            translate = get_group_transform(group)
        except RuntimeError:
            warn("Can't process path group '{}' because it has a complex transform: {}".format(name, group.attrib[
                "transform"]))
            continue
        path_geom = parse_path(path.attrib["d"])
        # Single line segment path => pose
        if len(path_geom) == 1 and is_line(path_geom[0]):
            line = path_geom[0]
            # We assume line starts at origin and points towards the second point
            start_coord = (line.start.real + translate[0], line.start.imag + translate[1])
            end_coord = (line.end.real + translate[0], line.end.imag + translate[1])
            poses.append((name, start_coord, end_coord))
        # If they're all lines, let's assume it's closed and use it as a region
        elif all(map(is_line, path_geom)):
            # Real part => x, imag part => y
            lines = map(lambda l: ((l.start.real, l.start.imag), (l.end.real, l.end.imag)), path_geom)
            # Each line segment starts where the previous ended, so we can drop the end points
            points = map(lambda l: l[0], lines)
            points = map(lambda p: (float(p[0]), float(p[1])), points)
            points = map(lambda p: (p[0] + translate[0], p[1] + translate[1]), points)
            regions.append((name, points))
        else:
            print("Encountered path that couldn't be parsed")
    return poses, regions


def process_point_annotations(point_names, point_annotations, point_groups):
    points = []
    for point, text, parent in zip(point_annotations, point_names, point_groups):
        name = text.text
        translate = 0, 0
        try:
            translate = get_group_transform(parent)
        except RuntimeError:
            warn("Can't process point '{}' because it has a complex transform: {}".format(name,
                                                                                          parent.attrib["transform"]))
            continue
        pixel_coord = float(point.attrib["cx"]) + translate[0], float(point.attrib["cy"]) + translate[1]
        points.append((name, pixel_coord))
    return points


def process_pose_annotations(pose_names, pose_annotations, pose_groups):
    poses = []
    for pose, text, parent in zip(pose_annotations, pose_names, pose_groups):
        name = text.text
        translate = 0, 0
        try:
            translate = get_group_transform(parent)
        except RuntimeError:
            warn("Can't process pose '{}' because it has a complex transform: {}".format(name,
                                                                                         parent.attrib["transform"]))
            continue
        start_cord = float(pose.attrib["x1"]) + translate[0], float(pose.attrib["y1"]) + translate[1]
        stop_cord = float(pose.attrib["x2"]) + translate[0], float(pose.attrib["y2"]) + translate[1]
        poses.append((name, start_cord, stop_cord))
    return poses


def process_region_annotations(region_names, region_annotations, region_groups):
    regions = []
    for region, text, parent in zip(region_annotations, region_names, region_groups):
        name = text.text
        translate = 0, 0
        try:
            translate = get_group_transform(parent)
        except RuntimeError:
            warn("Can't process region '{}' because it has a complex transform: {}".format(name,
                                                                                           parent.attrib["transform"]))
            continue
        points_strs = region.attrib["points"].split()
        poly_points = [(float(x_str), float(y_str)) for x_str, y_str in map(lambda x: x.split(","), points_strs)]
        # Apply any translation
        poly_points = map(lambda p: (p[0] + translate[0], p[1] + translate[1]), poly_points)
        regions.append((name, poly_points))
    return regions


def transform_to_map_coords(map_info, points, poses, regions):
    for i, point in enumerate(points):
        name, point = point
        point = point_to_map_coords(map_info, point)
        points[i] = (name, point)

    for i, pose in enumerate(poses):
        name, p1, p2 = pose
        p1 = point_to_map_coords(map_info, p1)
        p2 = point_to_map_coords(map_info, p2)
        poses[i] = (name, p1, p2)

    for i, region in enumerate(regions):
        name, poly_points = region
        poly_points = list(map(lambda p: point_to_map_coords(map_info, p), poly_points))
        regions[i] = (name, poly_points)

    return points, poses, regions


def point_to_map_coords(map_info, point):
    """
    Converts a pixel coordinate point into map coordinates
    :param map_info: A dictionary specifying the origin, resolution, width and height of the map
    :param point: A coordinate tuple in pixel coordinates
    :return: A coordinate tuple in map coordinates
    """
    map_origin, resolution, _, height = map_info["origin"][0:2], map_info["resolution"], map_info["width"], map_info[
        "height"]
    x, y = point
    # the map coordinate corresponding to the bottom left pixel
    origin_x, origin_y = map_origin
    vertically_flipped_point = x, height - y - 1
    map_x, map_y = vertically_flipped_point
    point = origin_x + map_x * resolution, origin_y + map_y * resolution
    return point
