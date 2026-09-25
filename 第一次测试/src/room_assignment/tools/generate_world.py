"""Generate the Gazebo Classic SDF room from measured box geometry (metres)."""

from pathlib import Path
from xml.etree import ElementTree as ET


# name, x, y, width, depth, height, material colour
OBJECTS = [
    ('north_wall', 0, 4, 12.2, .2, 2.2, '.72 .75 .79 1'),
    ('south_wall', 0, -4, 12.2, .2, 2.2, '.72 .75 .79 1'),
    ('west_wall', -6, 0, .2, 8.2, 2.2, '.72 .75 .79 1'),
    ('east_wall', 6, 0, .2, 8.2, 2.2, '.72 .75 .79 1'),
    ('left_partition_north', -2, 2.5, .16, 3, 2.2, '.84 .68 .51 1'),
    ('left_partition_south', -2, -2.5, .16, 3, 2.2, '.84 .68 .51 1'),
    ('right_partition_north', 2, 2.5, .16, 3, 2.2, '.84 .68 .51 1'),
    ('right_partition_south', 2, -2.5, .16, 3, 2.2, '.84 .68 .51 1'),
    ('bedroom_wall_west', -5.35, 2.2, 1.3, .16, 2.2, '.84 .68 .51 1'),
    ('bedroom_wall_east', -2.65, 2.2, 1.3, .16, 2.2, '.84 .68 .51 1'),
    ('storage_wall_west', 2.65, -2.2, 1.3, .16, 2.2, '.84 .68 .51 1'),
    ('storage_wall_east', 5.35, -2.2, 1.3, .16, 2.2, '.84 .68 .51 1'),
    ('bedroom_column', -5.2, 3, .35, .35, 2.2, '.42 .45 .48 1'),
    ('living_column', -3.2, -1.35, .35, .35, 2.2, '.42 .45 .48 1'),
    ('central_column_north', .75, 2.7, .35, .35, 2.2, '.42 .45 .48 1'),
    ('central_column_south', -.8, -2.7, .35, .35, 2.2, '.42 .45 .48 1'),
    ('study_column', 4.4, 2.2, .35, .35, 2.2, '.42 .45 .48 1'),
    ('living_box', -5.0, -2.6, .7, .6, .7, '.62 .37 .19 1'),
    ('hall_box', 0, -.3, .7, .7, .7, '.62 .37 .19 1'),
    ('study_box', 4.6, 1.2, .7, .6, .7, '.62 .37 .19 1'),
    ('storage_box', 5.1, -3, .8, .6, .7, '.62 .37 .19 1'),
]


def child(parent, tag, value=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if value is not None:
        element.text = str(value)
    return element


def add_box(world, name, x, y, width, depth, height, color):
    model = child(world, 'model', name=name)
    child(model, 'static', 'true')
    child(model, 'pose', f'{x} {y} {height / 2} 0 0 0')
    link = child(model, 'link', name='body')
    for tag in ('collision', 'visual'):
        part = child(link, tag, name=tag)
        child(child(child(part, 'geometry'), 'box'), 'size', f'{width} {depth} {height}')
        if tag == 'visual':
            material = child(part, 'material')
            child(material, 'ambient', color)
            child(material, 'diffuse', color)


def main():
    sdf = ET.Element('sdf', version='1.6')
    world = child(sdf, 'world', name='multiroom')
    child(child(world, 'include'), 'uri', 'model://sun')
    add_box(world, 'floor', 0, 0, 12, 8, .1, '.75 .75 .72 1')
    world.find("./model[@name='floor']/pose").text = '0 0 -0.05 0 0 0'
    for item in OBJECTS:
        add_box(world, *item)
    camera = child(child(world, 'gui', fullscreen='0'), 'camera', name='user_camera')
    child(camera, 'pose', '0 -14 13 0 0.7 1.57')
    child(camera, 'view_controller', 'orbit')
    ET.indent(sdf, space='  ')
    target = Path(__file__).resolve().parents[1] / 'worlds' / 'multiroom.world'
    target.parent.mkdir(exist_ok=True)
    ET.ElementTree(sdf).write(target, encoding='unicode', xml_declaration=True)


if __name__ == '__main__':
    main()
