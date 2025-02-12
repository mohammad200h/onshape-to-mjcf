

import mujoco as mj
import mujoco.viewer
import os
import argparse

try:
    from robotiq_2F85.utility import (
        get_joint_type,
        get_object_type,
        get_transmission_type,
        get_bias_type,
        get_equality_type
    )
except:
    from utility import (
        get_joint_type,
        get_object_type,
        get_transmission_type,
        get_bias_type,
        get_equality_type
    )

import json

code_dir = os.path.dirname(os.path.realpath(__file__))
meshes = []

def generate_tree(spec, body, data, joints_def):
    body = body.add_body(name = data['name'],
                         pos = data['pos'],
                         euler = data['euler'],
                         mass = data['inertia']['mass'],
                         ipos = data['inertia']['pos'],
                         fullinertia = data['inertia']['fullinertia'])
    g = data['geom']
    body.add_geom(pos = g['pos'], euler = g['euler'],
                  meshname = g['mesh'], rgba = g['rgba'])
    if g["mesh"] not in meshes:
        spec.add_mesh(name = g['mesh'],
                      file = f"{code_dir}/assets/{g['mesh']}.stl")
        meshes.append(g["mesh"])

    j = data['joint']
    if j:
        j = j.copy()
        j['type'] = get_joint_type(j['type'])
        if "default" in j.keys():
            j["default"] = joints_def[j["default"]]
        body.add_joint(**j)
    s = data['site']
    if s:
        body.add_site(name = s['name'], pos = s['pos'],euler = s['euler'])

    for child in data['children']:
        generate_tree(spec, body, child, joints_def)

def mjspec_model(remove_collision = True):
    spec = mj.MjSpec()
    spec.compiler.degree = False
    spec.compiler.eulerseq = ['X', 'Y', 'Z']

    robot_data = {}
    with open(code_dir + '/tree.json') as f:
        robot_data = json.load(f)

    # Defaults
    main = spec.default()
    main.geom.type = mj.mjtGeom.mjGEOM_MESH
    # Joint Defaults
    joints_def = {}
    for d in robot_data["joints_default"]:
        joints_def[d["name"]] = spec.add_default(d["name"],main)

        joints_def[d["name"]].joint.solimp_limit[:3] = d["solimp_limit"]
        joints_def[d["name"]].joint.solref_limit = d["solref_limit"]

        if "armature" in d.keys():
            joints_def[d["name"]].joint.armature = d["armature"]
        if "armature" in d.keys():
            joints_def[d["name"]].joint.damping = d["damping"]


    # Tree
    root = spec.worldbody
    generate_tree(spec, root, robot_data['tree'],joints_def)

    # Equality constraints if any
    equalities = robot_data['equality']
    for e in equalities:
        spec.add_equality(type = get_equality_type('connect'),
                          objtype = get_object_type('body'),
                          name1 = e['body1'], name2 = e['body2'],
                          data = e['anchor'] + [0] * 8,
                          solimp = e["solimp"] + [0] * 2 ,
                          solref = e["solref"]
                          )

    if remove_collision:
        model = spec.compile()
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        # Get Collisions
        contacts = data.contact
        contact_pairs = []
        for contact in contacts:
            g1 = contact.geom1
            g2 = contact.geom2
            b1_id = model.geom_bodyid[g1]
            b2_id = model.geom_bodyid[g2]
            b1 = mj.mj_id2name(model, get_object_type('body'), b1_id)
            b2 = mj.mj_id2name(model, get_object_type('body'), b2_id)
            contact_pairs.append((b1,b2))

        # Write contact pairs for reference
        robot_data['contacts'] = contact_pairs
        with open(code_dir + '/tree.json','w') as f:
            json.dump(robot_data,f, indent = 2)

    if "contacts" in robot_data.keys() and len("contacts") > 0:
        # Remove collisions
        contact_pairs = robot_data["contacts"]
        for b1,b2 in contact_pairs:
            spec.add_exclude(bodyname1 = b1, bodyname2 = b2)

    # Joint equality
    d = robot_data["joint_equality"]
    spec.add_equality(
        type = get_equality_type(d["type"]),
        objtype = get_object_type(d["objtype"]),
        name1 = d["joint1"],
        name2 = d["joint2"],
        solimp = d["solimp"] + [0] * 2,
        solref = d["solref"],
        data = d["polycoef"] + [0] * 6
    )

    # Tendon driven actuator
    # Wrapping tendon around joints
    d = robot_data["tendon"]
    tendon = spec.add_tendon(name = d["name"])
    for j in d["joints"]:
        tendon.wrap_joint(j["name"],j["coef"])
    # Actuating tendon
    # Actuator defaults
    d = robot_data["actuator_defaults"]
    main.actuator.trntype = get_transmission_type(d["trntype"])
    main.actuator.biastype = get_bias_type(d["biastype"])

    # Actuators
    d = robot_data["actuator"]
    gainprm = main.actuator.gainprm[:]
    biasprm = main.actuator.biasprm[:]

    gainprm[:3] = d["gainprm"]
    biasprm[:3] = d["biasprm"]

    spec.add_actuator(
        name = d["name"],
        target = d["tendon"],
        forcerange = d["forcerange"],
        ctrlrange = d["ctrlrange"],
        gainprm = gainprm,
        biasprm = biasprm
    )


    return root, spec

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--remove_collision',
        action = 'store_true',
        default = False,
        help = 'Remove all contacts'
    )

    args = parser.parse_args()


    root, spec = mjspec_model(args.remove_collision)
    model = spec.compile()
    data = mj.MjData(model)

    # Write  Model XML
    with open(code_dir + '/model.xml', 'w') as f:
        f.write(spec.to_xml())

    # Visualization
    with mj.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mj.mjv_defaultFreeCamera(model, viewer.cam)
        mj.mj_forward(model, data)
        while viewer.is_running():
            mj.mj_step(model, data)
            viewer.sync()

