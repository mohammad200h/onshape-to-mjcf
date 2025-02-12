

import mujoco as mj
import mujoco.viewer
import os
import argparse

try:
    from iiwa7.utility import (get_joint_type,
                         get_transmission_type,
                         get_gain_type,
                         get_bias_type)

except:
    from utility import (get_joint_type,
                         get_object_type,
                         get_transmission_type,
                         get_equality_type)

import json

code_dir = os.path.dirname(os.path.realpath(__file__))
meshes = []

def generate_tree(spec, body, data):
    body = body.add_body(name = data['name'],
                         pos = data['pos'],
                         euler = data['euler'],
                         mass = data['inertia']['mass'],
                         ipos = data['inertia']['pos'],
                         fullinertia = data['inertia']['fullinertia'],
                         explicitinertial = True)
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
        body.add_joint(**j)
    s = data['site']
    if s:
        body.add_site(name = s['name'], pos = s['pos'],euler = s['euler'])

    for child in data['children']:
        generate_tree(spec, body, child)

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

    # Tree
    root = spec.worldbody
    generate_tree(spec, root, robot_data['tree'])

    # Equality constraints if any
    equalities = robot_data['equality']
    for e in equalities:
        spec.add_equality(type = mj.mjtEq.mjEQ_CONNECT,
                          objtype = mj.mjtObj.mjOBJ_BODY,
                          name1 = e['body1'], name2 = e['body2'],
                          data = e['anchor'] + [0] * 8)

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
            b1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b1_id)
            b2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b2_id)
            contact_pairs.append((b1,b2))

        # Write contact pairs for reference
        robot_data['contacts'] = contact_pairs
        with open(code_dir + '/tree.json','w') as f:
            json.dump(robot_data,f, indent = 2)

        # Remove collisions
        for b1,b2 in contact_pairs:
            spec.add_exclude(bodyname1 = b1, bodyname2 = b2)

    # General actuator defaults
    d = robot_data["actuator_shared_defaults"]
    main.actuator.trntype = get_transmission_type(d["trntype"])
    main.actuator.gaintype = get_gain_type(d["gaintype"])
    main.actuator.biastype = get_bias_type(d["biastype"])

    main.actuator.gainprm[0] = d["gainprm"]
    main.actuator.biasprm[:3] = d["biasprm"]

    # Defaults for each type of actuator setting
    act_defs={}
    for d in robot_data["actuators_defaults"]:
        act_defs[d["name"]] = spec.add_default(d["name"],main)
        act_defs[d["name"]].actuator.ctrlrange = d["ctrlrange"]

    # Actuators
    for a in robot_data["actuators"]:
        a["default"] = act_defs[a["default"]]
        spec.add_actuator(**a)

    # Changing integrator to make sim stable
    spec.option.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST

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

