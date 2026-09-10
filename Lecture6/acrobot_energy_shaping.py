import numpy as np
import time
from pydrake.all import (
    ControllabilityMatrix,
    DiagramBuilder,
    LeafSystem,
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

class SpongSwingupController(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self._x_port = self.DeclareVectorInputPort(name="x", size=4)
        self.DeclareVectorOutputPort(name="u", size=1, calc=self.CalcControl)

        # Create an LQR controller for the linearized system about the upright position.
        self.acrobot = AcrobotPlant()
        self.acrobot_context = self.acrobot.CreateDefaultContext()

        input = AcrobotInput()
        input.set_tau(0.0)
        self.acrobot.get_input_port(0).FixValue(self.acrobot_context, input)

        self.acrobot_context.get_mutable_continuous_state_vector().SetFromVector(
            UprightState().CopyToVector()
        )

        self.E_desired = self.acrobot.EvalPotentialEnergy(self.acrobot_context)

        self.k_e = 5
        self.k_p = 50
        self.k_d = 5

        Q = np.diag((10.0, 10.0, 1.0, 1.0))
        R = [1]
        linear_acrobot = Linearize(self.acrobot, self.acrobot_context)
        self.K, self.S = LinearQuadraticRegulator(linear_acrobot.A(), linear_acrobot.B(), Q, R)

    def CalcControl(self, context, output):
        x = self._x_port.Eval(context)
        theta2 = x[1]
        theta2dot = x[3]
        self.acrobot_context.get_mutable_continuous_state_vector().SetFromVector(x)

        x_e = (x - UprightState().CopyToVector())
        lqr_cost_to_go = x_e.T @ self.S @ x_e
        print(lqr_cost_to_go)
        
        if lqr_cost_to_go < 100:
            u = -self.K @ x_e
            output[0] = u[0]
        else:
            # Energy shaping controller
            PE = self.acrobot.EvalPotentialEnergy(self.acrobot_context)
            KE = self.acrobot.EvalKineticEnergy(self.acrobot_context)
            E = PE + KE
            E_tilde =  E - self.E_desired

            u_e = -self.k_e * E_tilde * theta2dot

            # print(E_tilde)

            y = -self.k_p * theta2 - self.k_d * theta2dot;

            # Partial feedback linearization
            M = self.acrobot.MassMatrix(self.acrobot_context)
            bias = self.acrobot.DynamicsBiasTerm(self.acrobot_context)
            M_inverse = np.linalg.inv(M)
            a3 = M_inverse[1, 1]
            a2 = M_inverse[0, 1]
            u_p = (a2 * bias[0] + y) / a3 + bias[1]

            u = u_e + u_p

            output[0] = u

def acrobot_swingup_example():

    builder = DiagramBuilder()
    acrobot = builder.AddSystem(AcrobotPlant())

    saturation = builder.AddSystem(Saturation(min_value=[-10], max_value=[10]))
    builder.Connect(saturation.get_output_port(0), acrobot.get_input_port(0))
    wrapangles = WrapToSystem(4)
    wrapangles.set_interval(0, 0, 2.0 * np.pi)
    wrapangles.set_interval(1, -np.pi, np.pi)
    wrapto = builder.AddSystem(wrapangles)
    builder.Connect(acrobot.get_output_port(0), wrapto.get_input_port(0))
    controller = builder.AddSystem(SpongSwingupController())
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
    duration = 30.0
    context.SetTime(0.0)
    # context.SetContinuousState(
    #     UprightState().CopyToVector()+ 0.05 * np.random.randn(4,)
    # )
    context.SetContinuousState(0.05 * np.random.randn(4,))
    simulator.Initialize()
    simulator.AdvanceTo(duration)


if __name__=="__main__":
    meshcat = StartMeshcat()
    # acrobot_demo()
    acrobot_swingup_example()
