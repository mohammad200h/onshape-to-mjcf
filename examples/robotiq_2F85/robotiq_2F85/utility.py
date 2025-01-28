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

    err = f"""
    transmission type {{type_str}} is not supported. choose from :
    [joint, joint_in_parent, slider_crank, tendon, site,
     body, undefined
    ]
    """
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

    err = f"""
    gain type {{type_str}} is not supported. choose from :
    [fixed, affine, muscle, user]
    """
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

    err = f"""
    bias type {type_str} is not supported. choose from :
    [none, affine, muscle, user]
    """
    raise ValueError(err)

def get_joint_type(type_str):
    if type_str == 'hinge':
        return mj.mjtJoint.mjJNT_HINGE
    elif type_str == 'slide':
        return mj.mjtJoint.mjJNT_SLIDE
    elif type_str == 'ball':
        return mj.mjtJoint.mjJNT_BALL

    err = f"""
    joint type {type_str} is not supported. choose from :
    [hinge, slide, ball]
    """
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

    err = f"""
    object type {{type_str}} is not supported. choose from :
    [unknown, body, xbody, joint, dof,
     geom, site, camera, light, flex,
     mesh, skin, hfield, texture, material,
     obj_pair, obj_exclude, equality, tendon,
     actuator, sensor,numeric, text, tuple,
     key, plugin, no_object, obj_frame]
    """
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

    err = f"""
    equality type {type_str} is not supported. choose from :
    [connect, weld, joint, tendon, flex, distance]
    """
    raise ValueError(err)
