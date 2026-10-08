package frc.robot.mechanisms.transport

import com.ctre.phoenix6.BaseStatusSignal
import frc.robot.mechanisms.pivot.PivotIO
import frc.robot.utils.RobotParameters
import org.wpilib.math.controller.PIDController
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.math.util.Units
import org.wpilib.simulation.DCMotorSim
import java.lang.Math.clamp
import kotlin.math.abs

class SimsTransportIO : TransportIO {
    private val indexerMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)
    private val hopperMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)

    private val indexerSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                indexerMotorModel,
                0.25,
                RobotParameters.TransportParameters.INDEXER_GEAR_RATIO,
            ),
            indexerMotorModel,
        )

    private var indexerClosedLoop = false
    private val indexerController = PIDController(0.0, 0.0, 0.0)
    private var indexerFFVolts = 0.0
    private var indexerAppliedVolts = 0.0

    private val hopperSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                hopperMotorModel,
                0.25,
                RobotParameters.TransportParameters.HOPPER_GEAR_RATIO,
            ),
            hopperMotorModel,
        )

    private var hopperClosedLoop = false
    private val hopperController = PIDController(0.0, 0.0, 0.0)
    private var hopperFFVolts = 0.0
    private var hopperAppliedVolts = 0.0

    override fun updateInputs(inputs: TransportIO.TransportIOInputs) {
        if (!indexerClosedLoop) {
            indexerController.reset()
        }
        if (!hopperClosedLoop) {
            hopperController.reset()
        }

        indexerAppliedVolts = indexerController.calculate(indexerSim.angularVelocity)
        hopperAppliedVolts = hopperController.calculate(hopperSim.angularVelocity)

        // Update simulation state
        indexerSim.setInputVoltage(clamp(indexerAppliedVolts, -12.0, 12.0))
        hopperSim.setInputVoltage(clamp(hopperAppliedVolts, -12.0, 12.0))

        inputs.data.indexerConnected = true
        inputs.data.hopperConnected = true

        inputs.data.indexerPositionRad = indexerSim.angularPosition
        inputs.data.indexerAppliedVolts = indexerAppliedVolts
        inputs.data.indexerSupplyCurrentAmps = abs(indexerSim.currentDraw)
        inputs.data.indexerTorqueCurrentAmps = 0.0
        inputs.data.indexerVelocityRadPerSec = indexerSim.angularVelocity

        inputs.data.hopperPositionRad = hopperSim.angularPosition
        inputs.data.hopperAppliedVolts = hopperAppliedVolts
        inputs.data.hopperSupplyCurrentAmps = abs(hopperSim.currentDraw)
        inputs.data.hopperTorqueCurrentAmps = 0.0
        inputs.data.hopperVelocityRadPerSec = hopperSim.angularVelocity

        indexerSim.update(0.02)
        hopperSim.update(0.02)
    }

    /**
     * Sets the angle of the simPivot motor based on state. Input is * by 360 degrees since the real motor takes in rotations.
     *
     * @param state The desired state of the pivot.
     */
    override fun setTransportState(state: TransportIO.TransportState) {
        indexerClosedLoop = true
        indexerController.setSetpoint(state.velocity)
        hopperClosedLoop = true
        hopperController.setSetpoint(state.velocity)
    }
}