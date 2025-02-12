from ..onshape_api.config import config

utility_file = """
import mujoco as mj

def get_transmission_type(type_str):
    if type_str  == "joint":
        return mj.mjtTrn.mjTRN_JOINT
    elif type_str == "joint_in_parent":
        return mj.mjtTrn.mjTRN_JOINTINPARENT
    elif type_str == "slider_crank":
        return mj.mjtTrn.mjTRN_SLIDERCRANK
    elif type_str == "tendon":
        return mj.mjtTrn.mjTRN_TENDON
    elif type_str == "site":
        return mj.mjtTrn.mjTRN_SITE
    elif type_str == "body":
        return mj.mjtTrn.mjTRN_BODY
    elif type_str == "undefined":
        return mj.mjtTrn.mjTRN_UNDEFINED

    err = f\"\"\"
    transmission type {{type_str}} is not supported. choose from :
    [joint, joint_in_parent, slider_crank, tendon, site,
     body, undefined
    ]
    "\"\"
    raise ValueError(err)

def get_gain_type(type_str):
    if type_str == "fixed":
        return mj.mjtGain.mjGAIN_FIXED
    elif type_str =="affine":
        return mj.mjtGain.mjGAIN_AFFINE
    elif type_str == "muscle":
        return mj.mjtGain.mjGAIN_MUSCLE
    elif type_str == "user":
        return mj.mjtGain.mjGAIN_USER

    err = f\"\"\"
    gain type {{type_str}} is not supported. choose from :
    [fixed, affine, muscle, user]
    \"\"\"
    raise ValueError(err)

def get_bias_type(type_str):
    if type_str == "none":
        return mj.mjtBias.mjBIAS_NONE
    elif type_str =="affine":
        return mj.mjtBias.mjBIAS_AFFINE
    elif type_str == "muscle":
        return mj.mjtBias.mjBIAS_MUSCLE
    elif type_str == "user":
        return mj.mjtBias.mjBIAS_USER

    err = f\"\"\"
    bias type {type_str} is not supported. choose from :
    [none, affine, muscle, user]
    \"\"\"
    raise ValueError(err)

def get_joint_type(type_str):
    if type_str == 'hinge':
        return mj.mjtJoint.mjJNT_HINGE
    elif type_str == 'slide':
        return mj.mjtJoint.mjJNT_SLIDE
    elif type_str == 'ball':
        return mj.mjtJoint.mjJNT_BALL

    err = f\"\"\"
    joint type {type_str} is not supported. choose from :
    [hinge, slide, ball]
    \"\"\"
    raise ValueError(err)

def get_object_type(type_str):
    if type_str =="unknown":
        return mj.mjtObj.mjOBJ_UNKNOWN
    elif type_str =="body":
        return mj.mjtObj.mjOBJ_BODY
    elif type_str =="xbody":
        return mj.mjtObj.mjOBJ_XBODY
    elif type_str =="joint":
        return mj.mjtObj.mjOBJ_JOINT
    elif type_str =="dof":
        return mj.mjtObj.mjOBJ_DOF
    elif type_str =="geom":
        return mj.mjtObj.mjOBJ_GEOM
    elif type_str =="site":
        return mj.mjtObj.mjOBJ_SITE
    elif type_str =="camera":
        return mj.mjtObj.mjOBJ_CAMERA
    elif type_str =="light":
        return mj.mjtObj.mjOBJ_LIGHT
    elif type_str =="flex":
        return mj.mjtObj.mjOBJ_FLEX
    elif type_str =="mesh":
        return mj.mjtObj.mjOBJ_MESH
    elif type_str =="skin":
        return mj.mjtObj.mjOBJ_SKIN
    elif type_str =="hfield":
        return mj.mjtObj.mjOBJ_HFIELD
    elif type_str =="texture":
        return mj.mjtObj.mjOBJ_TEXTURE
    elif type_str =="material":
        return mj.mjtObj.mjOBJ_MATERIAL
    elif type_str =="obj_pair":
        return mj.mjtObj.mjOBJ_PAIR
    elif type_str =="obj_exclude":
        return mj.mjtObj.mjOBJ_EXCLUDE
    elif type_str =="equality":
        return mj.mjtObj.mjOBJ_EQUALITY
    elif type_str =="tendon":
        return mj.mjtObj.mjOBJ_TENDON
    elif type_str =="actuator":
        return mj.mjtObj.mjOBJ_ACTUATOR
    elif type_str =="sensor":
        return mj.mjtObj.mjOBJ_SENSOR
    elif type_str =="numeric":
        return mj.mjtObj.mjOBJ_NUMERIC
    elif type_str =="text":
        return mj.mjtObj.mjOBJ_TEXT
    elif type_str =="tuple":
        return mj.mjtObj.mjOBJ_TUPLE
    elif type_str =="key":
        return mj.mjtObj.mjOBJ_KEY
    elif type_str =="plugin":
        return mj.mjtObj.mjOBJ_PLUGIN
    elif type_str =="no_object":
        return mj.mjtObj.mjNOBJECT
    elif type_str =="obj_frame":
        return mj.mjtObj.mjOBJ_FRAME

    err = f\"\"\"
    object type {{type_str}} is not supported. choose from :
    [unknown, body, xbody, joint, dof,
     geom, site, camera, light, flex,
     mesh, skin, hfield, texture, material,
     obj_pair, obj_exclude, equality, tendon,
     actuator, sensor,numeric, text, tuple,
     key, plugin, no_object, obj_frame]
    \"\"\"
    raise ValueError(err)

def get_equality_type(type_str):
    if type_str == "connect":
        return mj.mjtEq.mjEQ_CONNECT
    elif type_str == "weld":
        return mj.mjtEq.mjEQ_WELD
    elif type_str == "joint":
        return mj.mjtEq.mjEQ_JOINT
    elif type_str == "tendon":
        return mj.mjtEq.mjEQ_TENDON
    elif type_str == "flex":
        return mj.mjtEq.mjEQ_FLEX
    elif type_str == "distance":
        return mj.mjtEq.mjEQ_DISTANCE

    err = f\"\"\"
    equality type {type_str} is not supported. choose from :
    [connect, weld, joint, tendon, flex, distance]
    \"\"\"
    raise ValueError(err)
"""

model_file = f"""

import mujoco as mj
import mujoco.viewer
import os
import argparse

try:
    from {config['packageName']}.utility import(
        get_joint_type,
        get_object_type,
        get_equality_type
    )
except:
    from utility import(
            get_joint_type,
            get_object_type,
            get_equality_type
    )

import json

code_dir = os.path.dirname(os.path.realpath(__file__))
meshes = []

def generate_tree(spec, body, data):
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
                      file = f"{{code_dir}}/assets/{{g['mesh']}}.stl")
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

    robot_data = {{}}
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
        spec.add_equality(type = get_equality_type('connect'),
                          objtype = get_object_type('body'),
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

"""



setup_file = f"""
from setuptools import find_packages, setup


setup(
    name="{config['packageName']}",
    packages = find_packages(),
    include_package_data = True,
    python_requires='>=3',
    author="Some Dude or Lady",
    license="MIT",
    install_requires=[
        "mujoco"
    ],
    package_data={{'': ['tree.json','assets/*']}},
    zip_safe=False
)
"""
