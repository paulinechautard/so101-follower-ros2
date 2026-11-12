import math
import os
import tempfile
import xml.etree.ElementTree as ET

import yaml

CONTACT = {'friction': '2.0 0.02 0.001', 'condim': '6', 'solref': '0.004 1', 'solimp': '0.95 0.99 0.001'}
GREY = [0.6, 0.6, 0.6, 1.0]


def numbers(values):
    return ' '.join(f'{float(v):g}' for v in values)


def write_mjcf(world_file, mjcf_dir, wrist_camera):
    with open(world_file) as f:
        world = yaml.safe_load(f)
    root = ET.Element('mujoco', model=os.path.splitext(os.path.basename(world_file))[0])
    ET.SubElement(root, 'include', file='scene_camera.xml' if wrist_camera else 'scene.xml')
    worldbody = ET.SubElement(root, 'worldbody')
    for obj in world.get('objects') or []:
        if len(obj['size']) != 3 or len(obj['pose']) != 4:
            raise ValueError(f"{world_file}: {obj['id']}: size must be [x, y, z], pose [x, y, z, yaw]")
        if obj.get('mujoco', True) is False:
            continue
        x, y, z, yaw = (float(v) for v in obj['pose'])
        placement = {'pos': numbers((x, y, z)), 'quat': numbers((math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))}
        geom = {'name': obj['id'], 'type': 'box', 'size': numbers(v / 2 for v in obj['size']),
                'rgba': numbers(obj.get('rgba', GREY))}
        if 'mass' in obj:
            body = ET.SubElement(worldbody, 'body', name=obj['id'], **placement)
            ET.SubElement(body, 'freejoint')
            ET.SubElement(body, 'geom', mass=str(obj['mass']), **geom, **CONTACT)
        elif obj.get('visual'):
            ET.SubElement(worldbody, 'geom', **geom, **placement, contype='0', conaffinity='0')
        else:
            ET.SubElement(worldbody, 'geom', **geom, **placement, **CONTACT)
    ET.indent(root)

    out = tempfile.mkdtemp(prefix='so101_world_')
    os.symlink(os.path.realpath(os.path.join(mjcf_dir, '..', 'meshes')), os.path.join(out, 'meshes'))
    os.mkdir(os.path.join(out, 'mjcf'))
    for name in os.listdir(mjcf_dir):
        if name.endswith('.xml'):
            os.symlink(os.path.realpath(os.path.join(mjcf_dir, name)), os.path.join(out, 'mjcf', name))
    path = os.path.join(out, 'mjcf', 'world.xml')
    ET.ElementTree(root).write(path, encoding='unicode')
    return path
