package frc.robot.subsystems.intake

import frc.robot.utils.RobotParameters.IntakeParameters
import org.wpilib.math.controller.PIDController
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.DCMotorSim
import java.lang.Math.clamp
import kotlin.math.abs

class SimIntakeIO : IntakeIO {
    companion object {
        private val intakeMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)
    }

    private val intakeMotorSim = DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            intakeMotorModel,
            0.000,
            IntakeParameters.PhysicalParameters.INTAKE_GEAR_RATIO,
        ),
        intakeMotorModel,
    )

    private var intakeClosedLoop = false
    private var intakeVelocityControl = false
    private var intakeController = PIDController(0.0, 0.0, 0.0)
    private var intakeAppliedVolts = 0.0

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        if (!intakeClosedLoop) {
            intakeController.reset()
        }

        if (intakeVelocityControl) {
            intakeAppliedVolts = intakeController.calculate(intakeMotorSim.angularVelocity)
        } else {
            intakeAppliedVolts = intakeController.calculate(intakeMotorSim.angularPosition)
        }

        // TODO: set real min voltage, max voltage, and dt
        intakeMotorSim.setInputVoltage(clamp(intakeAppliedVolts, 0.00, 0.00))
        intakeMotorSim.update(0.00)

        inputs.data.intakeConnected = true
        inputs.data.intakePositionRad = intakeMotorSim.angularPosition
        inputs.data.intakeVelocityRadPerSec = intakeMotorSim.angularVelocity
        inputs.data.intakeAppliedVolts = intakeAppliedVolts
        inputs.data.intakeSupplyCurrentAmps = abs(intakeMotorSim.currentDraw)
        inputs.data.intakeTorqueCurrentAmps = 0.0
    }

    override fun setVoltage(volts: Double) {
        intakeClosedLoop = false
        intakeAppliedVolts = volts
    }

    override fun setPosition(positionRad: Double) {
        intakeClosedLoop = true
        intakeVelocityControl = false
        intakeController.setSetpoint(positionRad)
    }

    override fun setVelocity(velocityRadPerSec: Double) {
        intakeClosedLoop = true
        intakeVelocityControl = true
        intakeController.setSetpoint(velocityRadPerSec)
    }

    override fun setIntakePID(kP: Double, kI: Double, kD: Double) {
        intakeController.setPID(kP, kI, kD)
    }

    override fun stop() {
        setVoltage(0.0)
    }
}