

import mujoco as mj
import mujoco.viewer
import os


import json

code_dir = os.path.dirname(os.path.realpath(__file__))
meshes = []

def get_joint_type(type_str):
    if type_str == 'hinge':
        return mj.mjtJoint.mjJNT_HINGE
    elif type_str == 'slide':
        return mj.mjtJoint.mjJNT_SLIDE
    elif type_str == 'ball':
        return mj.mjtJoint.mjJNT_BALL

def generate_tree(spec, body, data):
    body = body.add_body(name=data['name'],
                         pos=data['pos'],
                         euler=data['euler'],
                         mass=data['inertia']['mass'],
                         ipos=data['inertia']['pos'],
                         fullinertia=data['inertia']['fullinertia'])
    g = data['geom']
    body.add_geom(pos=g['pos'], euler=g['euler'],
                  meshname=g['mesh'], rgba=g['rgba'])
    if g["mesh"] not in meshes:
        spec.add_mesh(name=g['mesh'], file=f"{code_dir}/assets/{g['mesh']}.stl")
        meshes.append(g["mesh"])

    j = data['joint']
    if j:
        body.add_joint(name=j['name'], type=get_joint_type(j['j_type']),
                       range=j['j_range'])
    s = data['site']
    if s:
        body.add_site(name = s['name'], pos = s['pos'],euler = s['euler'])

    for child in data['children']:
        generate_tree(spec, body, child)

def mjspec_model():
    spec = mj.MjSpec()
    spec.compiler.degree = False
    spec.compiler.eulerseq = ['X', 'Y', 'Z']
    main = spec.default()
    main.geom.type = mj.mjtGeom.mjGEOM_MESH

    data = {}

    with open(code_dir+'/tree.json') as f:
        data = json.load(f)

    # tree
    root = spec.worldbody
    generate_tree(spec, root, data['tree'])

    # equality constraints if any
    equalities = data['equality']
    for e in equalities:
        spec.add_equality(type=mj.mjtEq.mjEQ_CONNECT,
                          objtype=mj.mjtObj.mjOBJ_BODY,
                          name1=e['body1'], name2=e['body2'], data=e['anchor'] + [0] * 8)

    return root, spec

if __name__ == "__main__":
    root, spec = mjspec_model()
    model = spec.compile()
    data = mj.MjData(model)

    with open("spec_model.xml", "w") as f:
        f.write(spec.to_xml())

    # visualization
    with mj.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mj.mjv_defaultFreeCamera(model, viewer.cam)
        mj.mj_forward(model, data)
        while viewer.is_running():
            mj.mj_step(model, data)
            viewer.sync()

