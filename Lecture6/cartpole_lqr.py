import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ControllabilityMatrix,
    DiagramBuilder,
    LeafSystem,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Parser,
    Simulator,
    StartMeshcat,
)
from underactuated.meshcat_utils import MeshcatSliders
from underactuated import ConfigureParser



def cartpole_balancing_example():
    def UprightState():
        state = (0, np.pi, 0, 0)
        return state

    def Controllability(plant):
        context = plant.CreateDefaultContext()
        plant.get_actuation_input_port().FixValue(context, [0])
        plant.SetPositionsAndVelocities(context, UprightState())

        linearized_plant = Linearize(
            plant,
            context,
            input_port_index=plant.get_actuation_input_port().get_index(),
            output_port_index=plant.get_state_output_port().get_index(),
        )
        print(linearized_plant.A())
        print(linearized_plant.B())
        print(
            f"The singular values of the controllability matrix are: {np.linalg.svd(ControllabilityMatrix(linearized_plant), compute_uv=False)}"
        )

    def BalancingLQR(plant):
        # Design an LQR controller for stabilizing the CartPole around the upright.
        # Returns a (static) AffineSystem that implements the controller (in
        # the original CartPole coordinates).

        context = plant.CreateDefaultContext()
        plant.get_actuation_input_port().FixValue(context, [0])

        plant.SetPositionsAndVelocities(context, UprightState())

        Q = np.diag((10.0, 10.0, 1.0, 1.0))
        R = np.array([1])

        # MultibodyPlant has many (optional) input ports, so we must pass the
        # input_port_index to LQR.
        import pdb; pdb.set_trace()
        return LinearQuadraticRegulator(
            plant,
            context,
            Q,
            R,
            input_port_index=plant.get_actuation_input_port().get_index())

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.1)
    parser = Parser(plant)
    ConfigureParser(parser)
    parser.AddModelsFromUrl("package://underactuated/models/cartpole.urdf")
    plant.Finalize()
    controller = builder.AddSystem(BalancingLQR(plant))
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())

    # Setup visualization
    meshcat.Delete()
    meshcat.Set2dRenderMode(xmin=-2.5, xmax=2.5, ymin=-1.0, ymax=2.5)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyMutableContextFromRoot(context)

    # Simulate
    simulator.set_target_realtime_rate(1.0 )
    duration = 5.0
    for i in range(10):
        context.SetTime(0.0)
        plant.SetPositionsAndVelocities(
            plant_context,
            UprightState() + 0.1 * np.random.randn(4,),
        )
        simulator.Initialize()
        simulator.AdvanceTo(duration)


np.set_printoptions(formatter={"float": lambda x: "{0:0.4f}".format(x)})


if __name__=="__main__":
    meshcat = StartMeshcat()
    cartpole_balancing_example()
