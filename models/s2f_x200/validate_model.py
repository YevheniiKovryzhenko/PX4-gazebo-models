#!/usr/bin/env python3
"""Check generated mass properties, motor wrenches and pre-stall aero samples.

Uses only Python's standard library. Runtime sensor checks require Gazebo/PX4.
"""
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
DATA = json.loads((ROOT / 'matlab_data.json').read_text())
MODEL = ET.parse(ROOT / 'model.sdf').find('model')


def rpy_matrix(angles):
    r, p, y = angles
    matrices = [
        [[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]],
        [[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]],
        [[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]],
    ]
    result = matrices[0]
    for left in matrices[1:]:
        result = [[sum(left[i][k] * result[k][j] for k in range(3))
                   for j in range(3)] for i in range(3)]
    return result


class X200Physics(unittest.TestCase):
    def test_complete_neutral_mass_properties(self):
        mass = 0
        first = [0, 0, 0]
        tensor_origin = [[0.0] * 3 for _ in range(3)]
        for link in MODEL.findall('link'):
            part_mass = float(link.findtext('inertial/mass'))
            pose = [float(x) for x in link.findtext('pose', '0 0 0 0 0 0').split()]
            ipose = [float(x) for x in link.findtext('inertial/pose', '0 0 0 0 0 0').split()]
            rotation = rpy_matrix(pose[3:])
            position = [pose[i] + sum(rotation[i][j] * ipose[j] for j in range(3))
                        for i in range(3)]
            self.assertEqual(ipose[3:], [0, 0, 0])
            values = link.find('inertial/inertia')
            inertia = [[float(values.findtext('i' + ''.join(sorted(a + b))))
                        for b in 'xyz'] for a in 'xyz']
            # Positive definite and principal moments satisfy the triangle rule.
            self.assertGreater(inertia[0][0], 0)
            self.assertGreater(inertia[0][0] * inertia[1][1] - inertia[0][1] ** 2, 0)
            determinant = sum(inertia[0][i] * (
                inertia[1][(i+1) % 3] * inertia[2][(i+2) % 3]
                - inertia[1][(i+2) % 3] * inertia[2][(i+1) % 3]) for i in range(3))
            self.assertGreater(determinant, 0)
            mass += part_mass
            for i in range(3):
                first[i] += part_mass * position[i]
                for j in range(3):
                    tensor_origin[i][j] += sum(rotation[i][k] * inertia[k][l] * rotation[j][l]
                                               for k in range(3) for l in range(3))
                    tensor_origin[i][j] += part_mass * (
                        (sum(x*x for x in position) if i == j else 0) - position[i] * position[j])
        self.assertAlmostEqual(mass, DATA['mass_kg'], places=8)
        cg = [x / mass for x in first]
        for i in range(3):
            self.assertAlmostEqual(cg[i], DATA['cg_flu_m'][i], places=8)
            for j in range(3):
                tensor = tensor_origin[i][j] - mass * (
                    (sum(x*x for x in cg) if i == j else 0) - cg[i]*cg[j])
                self.assertAlmostEqual(tensor, DATA['inertia_flu_kg_m2'][i][j], places=8)

    def test_motor_static_wrenches_against_matlab(self):
        plugins = MODEL.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
        for index, (rotor, plugin) in enumerate(zip(DATA['rotors'], plugins)):
            direction = 1 if plugin.findtext('turningDirection') == 'ccw' else -1
            kt = float(plugin.findtext('motorConstant'))
            kq = kt * float(plugin.findtext('momentConstant'))
            download = (1 - 2 * math.atan(.85 / 16) / (2 * math.pi)) ** (1/3)
            for row in DATA['rotor_reference_samples']['values']:
                rpm = row[0]
                if rpm < 3000:
                    continue
                omega2 = (rpm * math.pi / 30) ** 2
                thrust = -row[3] * download if index < 4 else row[7]
                # The exported lift sample is rotor 0. CALC_FM transforms the
                # FRD hub torque into the FLU thrust-axis direction.
                torque = row[6] * rotor['spin_frd'] if index < 4 else row[8]
                self.assertLess(abs(kt * omega2 / thrust - 1), .05)
                self.assertLess(abs(-direction * kq * omega2 / torque - 1), .1)
            polar = float(MODEL.findtext(f"link[@name='rotor_{index}']/inertial/inertia/izz"))
            slowdown = float(plugin.findtext('rotorVelocitySlowdownSim'))
            self.assertAlmostEqual(polar / slowdown, rotor['prop_inertia_kg_m2'])
            self.assertGreater(float(plugin.findtext('timeConstantUp')), .08)

    def test_near_level_aerodynamics_against_matlab(self):
        plugins = MODEL.findall("plugin[@name='gz::sim::systems::LiftDrag']")[:6]
        for row in DATA['aero_reference']['values']:
            if abs(row[6]) > 3:
                continue
            alpha = math.radians(row[6])
            lift = drag = pitch = 0
            for plugin in plugins:
                value = lambda name: float(plugin.findtext(name))
                effective = alpha + value('a0')
                panel_lift = 245 * value('area') * value('cla') * effective
                panel_drag = 245 * value('area') * value('cda') * abs(effective)
                fx = panel_lift * math.sin(alpha) - panel_drag * math.cos(alpha)
                fz = -panel_lift * math.cos(alpha) - panel_drag * math.sin(alpha)
                x, _, z_flu = map(float, plugin.findtext('cp').split())
                pitch += -z_flu * fx - x * fz + 245 * value('area') * value('cma') * effective
                lift += panel_lift
                drag += panel_drag
            ref_lift = row[7] * math.sin(alpha) - row[9] * math.cos(alpha)
            ref_drag = -row[7] * math.cos(alpha) - row[9] * math.sin(alpha)
            self.assertLess(abs(lift / ref_lift - 1), .06)
            self.assertLess(abs(drag / ref_drag - 1), .06)
            self.assertLess(abs(pitch - row[11]), .2)

    def test_control_force_and_pitch_signs_against_matlab(self):
        plugins = MODEL.findall("plugin[@name='gz::sim::systems::LiftDrag']")
        for i, (plugin, reference) in enumerate(zip(plugins, DATA['control_reference_per_deg']['values'])):
            area = float(plugin.findtext('area'))
            delta_lift = 245 * area * float(plugin.findtext('control_joint_rad_to_cl')) * math.pi / 180
            self.assertAlmostEqual(delta_lift, -reference[2 if i < 6 else 1], places=7)
            if i < 6:
                x = float(plugin.findtext('cp').split()[0])
                moment = x * delta_lift + 245 * area * float(plugin.findtext('cm_delta')) * math.pi / 180
                self.assertAlmostEqual(moment, reference[4], places=7)


if __name__ == '__main__':
    unittest.main()
