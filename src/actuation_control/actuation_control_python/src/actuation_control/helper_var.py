# Author: ML lab continuum-actuation team 
# University of California, San Diego

import numpy as np
import csv

# Tendon driven wrist design parameters
ins_init = -144.9  # mm
prox_tend_length_init = 121.4  # mm
distal_tend_length_init = 13.5  # mm
q_init = [ins_init, prox_tend_length_init, distal_tend_length_init]

max_tendon_command_prox = 6.0 *3#TODO - not actual cmd  bc frequency. # mm. Setting bound to help mitigate breaking
max_tendon_command_dist = 5.0 *3#TODO - not actual cmd  bc frequency.  # mm. Setting bound to help mitigate breaking

mm_to_blcd_cmd = 30/120 * (2*np.pi/10)  # 30/120 from counting rev on lead screw and measuring. 10 is from worm gear on CTR system

mm_to_endo_insertion_stepper_mm = 2 / 1 * 1/ 4  # 2 mm per 1 rev of stepper. 1 rev of endovascular lin actuator is 4 mm


def relative_joint_target_to_motor_targets(rel_joint_cmd):
    # Antagonistic control
    motor_targ = [0.0] * 5
    # motor_targ[0] = (abs_joint_cmd[0] - ins_init) * mm_to_blcd_cmd  # mm (joint) to radians. BLDC driving endovasc insertion
    motor_targ[0] = -(rel_joint_cmd[0]) * mm_to_endo_insertion_stepper_mm  # mm (joint) to radians. Stepper drived endovascular insertion

    # Tendon cmds (mm)
    motor_targ_from_home_prox = rel_joint_cmd[1]
    motor_targ_from_home_distal = rel_joint_cmd[2]

    bounded_motor_targ_prox = np.clip(rel_joint_cmd[1], -max_tendon_command_prox, max_tendon_command_prox)
    bounded_motor_targ_distal = np.clip(rel_joint_cmd[2], -max_tendon_command_dist, max_tendon_command_dist)

    if motor_targ_from_home_prox > bounded_motor_targ_prox or motor_targ_from_home_distal > bounded_motor_targ_distal:
        print("------------REACHED TENDON CMD BOUNDS------------")

    motor_targ[1] = -(bounded_motor_targ_prox)  # mm
    motor_targ[2] = -(bounded_motor_targ_distal)  # mm
    motor_targ[3] = (bounded_motor_targ_distal)  # mm
    motor_targ[4] = (bounded_motor_targ_prox)  # mm

    return motor_targ

def abs_joint_target_to_motor_targets(abs_joint_cmd):
    # Antagonistic control
    motor_targ = [0.0] * 5
    # motor_targ[0] = (abs_joint_cmd[0] - ins_init) * mm_to_blcd_cmd  # mm (joint) to radians. BLDC driving endovasc insertion
    motor_targ[0] = -(abs_joint_cmd[0] - ins_init) * mm_to_endo_insertion_stepper_mm  # mm (joint) to radians. Stepper drived endovascular insertion

    # Tendon cmds (mm)
    motor_targ_from_home_prox = abs_joint_cmd[1] - prox_tend_length_init
    motor_targ_from_home_distal = abs_joint_cmd[2] - distal_tend_length_init

    bounded_motor_targ_prox = np.clip(motor_targ_from_home_prox, -max_tendon_command_prox, max_tendon_command_prox)
    bounded_motor_targ_distal = np.clip(motor_targ_from_home_distal, -max_tendon_command_dist, max_tendon_command_dist)

    if motor_targ_from_home_prox > bounded_motor_targ_prox or motor_targ_from_home_distal > bounded_motor_targ_distal:
        print("------------REACHED TENDON CMD BOUNDS------------")

    motor_targ[1] = -(bounded_motor_targ_prox)  # mm
    motor_targ[2] = -(bounded_motor_targ_distal)  # mm
    motor_targ[3] = (bounded_motor_targ_distal)  # mm
    motor_targ[4] = (bounded_motor_targ_prox)  # mm

    return motor_targ

def bound_joint_target(joint_targ):
    # Antagonistic control
    bounded_joint_targ = joint_targ
    bounded_joint_targ[1] = np.clip(joint_targ[1], 
                                    prox_tend_length_init - max_tendon_command_prox, 
                                    prox_tend_length_init + max_tendon_command_prox)
    bounded_joint_targ[2] = np.clip(joint_targ[2], 
                                    distal_tend_length_init - max_tendon_command_dist, 
                                    distal_tend_length_init + max_tendon_command_dist)

    return bounded_joint_targ


def load_path_csv(file_path, tip_pos):
    path_points = []
    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            x = float(row['x']) + tip_pos[0]
            z = float(row['z']) + tip_pos[1]
            path_points.append([x, z])
    return np.array(path_points)

def load_traj_csv(file_path, tip_pos, tip_orien):
    traj_points = []
    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            x = float(row['x']) + tip_pos[0]
            z = float(row['z']) + tip_pos[1]
            # print("tip_orien used in load_traj_csv func:", tip_orien)
            orien = -float(row['T']) + tip_orien
            traj_points.append([x, z, orien])
    return np.array(traj_points)

def load_traj_and_q_csv(file_path, tip_pos, tip_orien):
    traj_and_q_points = []
    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            x = float(row['x']) + tip_pos[0]
            z = float(row['z']) + tip_pos[1]
            # print("tip_orien used in load_traj_and_q_csv func:", tip_orien)
            orien = float(row['T']) + tip_orien
            q0 = float(row['q0'])
            q1 = float(row['q1'])
            q2 = float(row['q2'])
            traj_and_q_points.append([x, z, orien, q0, q1, q2])
    return np.array(traj_and_q_points)
