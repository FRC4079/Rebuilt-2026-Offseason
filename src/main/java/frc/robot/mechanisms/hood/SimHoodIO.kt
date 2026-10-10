package frc.robot.mechanisms.hood

import frc.robot.utils.RobotParameters
import org.wpilib.math.controller.PIDController
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.DCMotorSim
import java.lang.Math.clamp
import kotlin.math.abs

class SimHoodIO : HoodIO {
    private val hoodMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)

    private val hoodSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                hoodMotorModel,
                0.1,
                RobotParameters.HoodParameters.HOOD_GEAR_RATIO,
            ),
            hoodMotorModel,
        )

    private var closedLoop = false
    private val hoodController = PIDController(0.0, 0.0, 0.0)
    private var targetPositionRad = 0.0
    private var hoodAppliedVolts = 0.0

    override fun updateInputs(inputs: HoodIO.HoodIOInputs) {
        if (closedLoop) {
            hoodAppliedVolts = hoodController.calculate(hoodSim.angularPosition, targetPositionRad)
            hoodSim.setInputVoltage(clamp(hoodAppliedVolts, -12.0, 12.0))
        } else {
            hoodController.reset()
        }

        hoodSim.update(0.02)

        inputs.data.hoodConnected = true
        inputs.data.hoodPositionRad = hoodSim.angularPosition
        inputs.data.hoodVelocityRadPerSec = hoodSim.angularVelocity
        inputs.data.hoodAppliedVolts = hoodAppliedVolts
        inputs.data.hoodSupplyCurrentAmps = abs(hoodSim.currentDraw)
        inputs.data.hoodTorqueCurrentAmps = 0.0
    }

    override fun setPower(power: Double) {
        closedLoop = false
        hoodAppliedVolts = clamp(power * 12.0, -12.0, 12.0)
        hoodSim.setInputVoltage(hoodAppliedVolts)
    }

    override fun setHoodState(state: HoodIO.HoodPositionState) {
        closedLoop = true
        targetPositionRad = state.positionRad
    }

    override fun setPosition(positionRad: Double) {
        closedLoop = true
        targetPositionRad = positionRad
    }

    override fun disablePower() {
        closedLoop = false
        hoodSim.setInputVoltage(0.0)
    }

    override fun setCurrentLimit(currentLimit: Int) {
    }
}