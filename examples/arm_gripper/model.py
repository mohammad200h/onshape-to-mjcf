import mujoco as mj
import mujoco.viewer

from iiwa7.model import mjspec_model as iiwa7
from robotiq_2F85.model import mjspec_model as robotiq_2F85


def arm_gripper():
    arm, arm_spec = iiwa7()
    gripper, gripper_spec = robotiq_2F85()

    ee = arm_spec.sites[0]
    ee.attach(gripper,"_","")

    return arm, arm_spec


if __name__ == "__main__":
    root, spec = arm_gripper()

    model = spec.compile()
    data = mj.MjData(model)

    with open("model.xml", "w") as f:
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
