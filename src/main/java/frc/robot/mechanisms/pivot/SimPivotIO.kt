package frc.robot.mechanisms.pivot

import frc.robot.utils.RobotParameters
import org.wpilib.math.controller.PIDController
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.DCMotorSim
import java.lang.Math.clamp
import kotlin.math.abs

class SimPivotIO : PivotIO {
    private val pivotMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)

    private val pivotSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                pivotMotorModel,
                0.25,
                RobotParameters.PivotParameters.PIVOT_GEAR_RATIO,
            ),
            pivotMotorModel,
        )

    private var pivotClosedLoop = false
    private val pivotController = PIDController(0.0, 0.0, 0.0)
    private var pivotFFVolts = 0.0
    private var pivotAppliedVolts = 0.0

    override fun updateInputs(inputs: PivotIO.PivotIOInputs) {
        if (pivotClosedLoop) {
            pivotAppliedVolts =
                pivotFFVolts + pivotController.calculate(pivotSim.angularVelocity)
        } else {
            pivotController.reset()
        }

        // Update simulation state
        pivotSim.setInputVoltage(clamp(pivotAppliedVolts, -12.0, 12.0))

        inputs.data.pivotConnected = true
        inputs.data.pivotPositionRad = pivotSim.angularPosition
        inputs.data.pivotAppliedVolts = pivotAppliedVolts
        inputs.data.pivotSupplyCurrentAmps = abs(pivotSim.currentDraw)
        inputs.data.pivotTorqueCurrentAmps = 0.0
        inputs.data.pivotVelocityRadPerSec = pivotSim.angularVelocity

        pivotSim.update(0.02)
    }

    /**
     * Sets the angle of the simPivot motor based on state. Input is * by 360 degrees since the real motor takes in rotations.
     *
     * @param state The desired state of the pivot.
     */
    override fun setPivotState(state: PivotIO.PivotPosition) {
        pivotSim.setAngle(state.position * 360.0)
    }
}
