package frc.robot.subsystems.intake

import org.littletonrobotics.junction.AutoLog
import frc.robot.utils.logging.ReflectiveLoggableInputs

interface IntakeIO {
    @AutoLog
    open class IntakeIOInputs {
        @JvmField
        var data: IntakeIOData = IntakeIOData(
            intakeConnected = false,
            intakePositionRad = 0.0,
            intakeVelocityRadPerSec = 0.0,
            intakeAppliedVolts = 0.0,
            intakeSupplyCurrentAmps = 0.0,
            intakeTorqueCurrentAmps = 0.0,
        )
    }

    data class IntakeIOData(
        var intakeConnected: Boolean,
        var intakePositionRad: Double,
        var intakeVelocityRadPerSec: Double,
        var intakeAppliedVolts: Double,
        var intakeSupplyCurrentAmps: Double,
        var intakeTorqueCurrentAmps: Double,
    ) : ReflectiveLoggableInputs()

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun setVoltage(volts: Double) {}

    fun setPosition(positionRad: Double) {}

    fun setVelocity(velocityRadPerSec: Double) {}

    fun setIntakePID(kP: Double, kI: Double, kD: Double) {}

    fun stop() {}
}