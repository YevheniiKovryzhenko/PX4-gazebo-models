#!/usr/bin/env python3
"""Generate the experimental S2F-X200 Gazebo model from the committed data snapshot.

Run from any directory. The PX4 Gazebo submodule supplies the standard sensor
noise configuration; no MATLAB, SciPy or external meshes are required at runtime.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
DATA = json.loads((ROOT / 'matlab_data.json').read_text())


def node(parent, tag, text=None, **attrs):
    element = ET.SubElement(parent, tag, attrs)
    if text is not None:
        element.text = ' '.join(f'{v:.9g}' for v in text) if isinstance(text, (list, tuple)) else str(text)
    return element


def inertial(link, mass, diagonal, pose=None, products=(0, 0, 0)):
    inertia = node(link, 'inertial')
    if pose:
        node(inertia, 'pose', pose)
    node(inertia, 'mass', mass)
    tensor = node(inertia, 'inertia')
    for name, value in zip(['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'], [*diagonal, *products]):
        node(tensor, name, value)


def box(link, name, pose, size, color, collision=False):
    visual = node(link, 'visual', name=name)
    node(visual, 'pose', pose)
    node(node(node(visual, 'geometry'), 'box'), 'size', size)
    material = node(visual, 'material')
    node(material, 'ambient', color)
    node(material, 'diffuse', color)
    if collision:
        col = node(link, 'collision', name=name + '_collision')
        node(col, 'pose', pose)
        node(node(node(col, 'geometry'), 'box'), 'size', size)


def plugin(model, filename, name, values):
    result = node(model, 'plugin', filename='gz-sim-' + filename + '-system', name='gz::sim::systems::' + name)
    for key, value in values.items():
        node(result, key, value)
    return result


def generate():
    sdf = ET.Element('sdf', version='1.10')
    model = node(sdf, 'model', name='s2f_x200')
    node(model, 'pose', [0, 0, .3, 0, 0, 0])
    base = node(model, 'link', name='base_link')
    tensor = DATA['inertia_flu_kg_m2']
    # The small articulated-link masses are deducted from the base-link mass.
    # Their contribution to total inertia is small and is not subtracted here.
    inertial(base, DATA['mass_kg'] - .005 * 5 - .001 * 7 - .001,
             [tensor[i][i] for i in range(3)], DATA['cg_flu_m'] + [0, 0, 0],
             [tensor[0][1], tensor[0][2], tensor[1][2]])
    blue, gray, orange = '0.1 0.3 0.6 1', '0.2 0.2 0.2 1', '0.9 0.4 0.05 1'
    box(base, 'fuselage', [-.28, 0, 0, 0, 0, 0], [1.15, .15, .18], blue, True)
    box(base, 'wing', [-.05, 0, .017, 0, 0, 0], [.254, DATA['wing_span_m'], .025], blue, True)
    box(base, 'tail', [-.68, 0, .038, 0, 0, 0], [.1651, .55, .018], blue)
    box(base, 'fin', [-.65, 0, .14, 0, 0, 0], [.165, .015, .25], blue)
    for y in [-.35306, .35306]:
        box(base, 'boom_' + str(y), [-.1, y, .09, 0, 0, 0], [1.05, .025, .025], gray)
        box(base, 'skid_' + str(y), [0, y, -.19, 0, 0, 0], [.75, .025, .025], gray, True)
    standard = ET.parse(ROOT.parent / 'standard_vtol/model.sdf').find('model/link')
    for sensor in standard.findall('sensor'):
        sensor = deepcopy(sensor)
        # Older PX4 model baselines omit these labels. Keep generated X200 sensor
        # frames identical across supported versions without altering sensor physics.
        if sensor.find('gz_frame_id') is None:
            ET.SubElement(sensor, 'gz_frame_id').text = 'base_link'
        # Match the newer baseline's element order for reproducible output.
        frame = sensor.find('gz_frame_id')
        sensor.remove(frame)
        sensor.insert(0, frame)
        base.append(sensor)
    airspeed_link = node(model, 'link', name='airspeed_link')
    inertial(airspeed_link, .001, [1e-7, 1e-7, 1e-7])
    attachment = node(model, 'joint', name='airspeed_joint', type='fixed')
    node(attachment, 'parent', 'base_link')
    node(attachment, 'child', 'airspeed_link')
    speed = node(airspeed_link, 'sensor', name='air_speed', type='air_speed')
    node(speed, 'always_on', 'true')
    node(speed, 'update_rate', 50)
    for index, rotor in enumerate(DATA['rotors']):
        link = node(model, 'link', name=f'rotor_{index}')
        pose = rotor['position_flu_m'] + [math.radians(rotor['roll_deg']), math.radians(rotor['pitch_deg']), 0]
        node(link, 'pose', pose)
        inertial(link, .005, [1e-5, 1e-5, 2e-5])
        box(link, 'propeller', [0, 0, 0, 0, 0, 0], [2 * rotor['radius_m'], .025, .008], orange)
        joint = node(model, 'joint', name=f'rotor_{index}_joint', type='revolute')
        node(joint, 'parent', 'base_link')
        node(joint, 'child', f'rotor_{index}')
        axis = node(joint, 'axis')
        node(axis, 'xyz', '0 0 1')
        limit = node(axis, 'limit')
        node(limit, 'lower', -1e16)
        node(limit, 'upper', 1e16)
        plugin(model, 'multicopter-motor-model', 'MulticopterMotorModel', {
            'jointName': f'rotor_{index}_joint', 'linkName': f'rotor_{index}',
            # MATLAB spin sign is about the thrust axis in FRD; Gazebo local +Z
            # points along thrust, so its rotation sign is reversed.
            'turningDirection': 'ccw' if rotor['spin_frd'] < 0 else 'cw',
            'timeConstantUp': .025, 'timeConstantDown': .025,
            'maxRotVelocity': rotor['max_speed_rad_s'], 'motorConstant': rotor['thrust_coefficient'],
            'momentConstant': rotor['moment_ratio_m'], 'commandSubTopic': 'command/motor_speed',
            'actuator_number': index, 'rotorDragCoefficient': .0001,
            'rollingMomentCoefficient': 1e-6, 'rotorVelocitySlowdownSim': 10, 'motorType': 'velocity'})
    for index, surface in enumerate(DATA['surfaces']):
        link = node(model, 'link', name=surface['name'])
        cp = surface['cp_flu_m']
        node(link, 'pose', cp + [0, 0, 0])
        inertial(link, .001, [1e-6, 1e-6, 1e-6])
        box(link, 'surface', [0, 0, 0, 0, 0, 0], [.08, .2, .008] if index < 6 else [.08, .008, .17], orange)
        joint = node(model, 'joint', name=f'servo_{index}', type='revolute')
        node(joint, 'parent', 'base_link')
        node(joint, 'child', surface['name'])
        axis = node(joint, 'axis')
        # Positive commands follow the model's signed physical servo travel.
        node(axis, 'xyz', '0 1 0' if index < 6 else '0 0 1')
        limit = node(axis, 'limit')
        node(limit, 'lower', -math.pi / 6)
        node(limit, 'upper', math.pi / 6)
        node(limit, 'effort', 5)
        node(limit, 'velocity', 10)
        plugin(model, 'joint-position-controller', 'JointPositionController', {
            'joint_name': f'servo_{index}', 'sub_topic': f'servo_{index}',
            'p_gain': 1, 'i_gain': 0, 'd_gain': .02, 'cmd_max': 2, 'cmd_min': -2})
        aero = DATA[surface['profile']]
        plugin(model, 'lift-drag', 'LiftDrag', {
            'a0': aero['a0_rad'], 'cla': aero['cla'], 'cda': .65, 'cma': 0,
            'alpha_stall': aero['stall_rad'], 'cla_stall': -3.85, 'cda_stall': -.92, 'cma_stall': 0,
            'cp': cp, 'area': surface['area_m2'], 'air_density': 1.225,
            'forward': '1 0 0', 'upward': '0 0 1' if index < 6 else '0 -1 0',
            'link_name': 'base_link', 'control_joint_name': f'servo_{index}',
            'control_joint_rad_to_cl': -1.5})
    node(model, 'static', 'false')
    ET.indent(sdf, space='  ')
    (ROOT / 'model.sdf').write_text('<?xml version="1.0"?>\n<!-- Experimental integration model; regenerate with generate_model.py. -->\n' + ET.tostring(sdf, encoding='unicode') + '\n')


if __name__ == '__main__':
    generate()
