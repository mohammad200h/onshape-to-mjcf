
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
