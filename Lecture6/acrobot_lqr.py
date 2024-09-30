import numpy as np
import time
from pydrake.all import (
    ControllabilityMatrix,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    WrapToSystem,
)
from pydrake.examples import AcrobotGeometry, AcrobotInput, AcrobotPlant, AcrobotState

from underactuated.meshcat_utils import MeshcatSliders

def UprightState():
    state = AcrobotState()
    state.set_theta1(np.pi)
    state.set_theta2(0.0)
    state.set_theta1dot(0.0)
    state.set_theta2dot(0.0)
    return state


def acrobot_demo():
    builder = DiagramBuilder()
    acrobot = builder.AddSystem(AcrobotPlant())

    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())
    AcrobotGeometry.AddToBuilder(builder, acrobot.get_output_port(0), scene_graph)
    meshcat.Delete()
    meshcat.Set2dRenderMode(xmin=-4, xmax=4, ymin=-4, ymax=4)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Setup slider input
    meshcat.AddSlider(
        "u",
        min=-5,
        max=5,
        step=0.1,
        value=0.0,
        decrement_keycode="ArrowLeft",
        increment_keycode="ArrowRight",
    )
    torque_system = builder.AddSystem(MeshcatSliders(meshcat, ["u"]))
    builder.Connect(torque_system.get_output_port(), acrobot.get_input_port())

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Set the initial conditions (theta1, theta2, theta1dot, theta2dot)
    context.SetContinuousState([1, 0, 0, 0])

    simulator.set_target_realtime_rate(1.0)

    print("Use the slider in the MeshCat controls to apply elbow torque.")
    print("Press 'Stop Simulation' in MeshCat to continue.")
    meshcat.AddButton("Stop Simulation")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)

    meshcat.DeleteAddedControls()

def acrobot_balancing_example():
    def BalancingLQR():
        # Design an LQR controller for stabilizing the Acrobot around the upright.
        # Returns a (static) AffineSystem that implements the controller (in
        # the original AcrobotState coordinates).

        acrobot = AcrobotPlant()
        context = acrobot.CreateDefaultContext()

        input = AcrobotInput()
        input.set_tau(0.0)
        acrobot.get_input_port(0).FixValue(context, input)

        context.get_mutable_continuous_state_vector().SetFromVector(
            UprightState().CopyToVector()
        )

        Q = np.diag((10.0, 10.0, 1.0, 1.0))
        R = [1]

        return LinearQuadraticRegulator(acrobot, context, Q, R)

    builder = DiagramBuilder()
    acrobot = builder.AddSystem(AcrobotPlant())

    saturation = builder.AddSystem(Saturation(min_value=[-10], max_value=[10]))
    builder.Connect(saturation.get_output_port(0), acrobot.get_input_port(0))
    wrapangles = WrapToSystem(4)
    wrapangles.set_interval(0, 0, 2.0 * np.pi)
    wrapangles.set_interval(1, -np.pi, np.pi)
    wrapto = builder.AddSystem(wrapangles)
    builder.Connect(acrobot.get_output_port(0), wrapto.get_input_port(0))
    controller = builder.AddSystem(BalancingLQR())
    builder.Connect(wrapto.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))

    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())
    AcrobotGeometry.AddToBuilder(builder, acrobot.get_output_port(0), scene_graph)
    meshcat.Delete()
    meshcat.Set2dRenderMode(xmin=-4, xmax=4, ymin=-4, ymax=4)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Simulate
    simulator.set_target_realtime_rate(1.0)
    duration = 4.0
    for i in range(10):
        context.SetTime(0.0)
        context.SetContinuousState(
            UprightState().CopyToVector()+ 0.05 * np.random.randn(4,)
        )
        simulator.Initialize()
        simulator.AdvanceTo(duration)
        time.sleep(1)


if __name__=="__main__":
    meshcat = StartMeshcat()
    # acrobot_demo()
    acrobot_balancing_example()
