package frc.robot.subsystems.pivot

import frc.robot.utils.logging.ReflectiveLoggableInputs
import org.littletonrobotics.junction.AutoLog
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.units.Units.*

interface PivotIO {
    @AutoLog
    open class PivotIOInputs {
        @JvmField
        var data: PivotIOData = PivotIOData(
            pivotConnected = false,
            pivotPositionRad = 0.0,
            pivotVelocityRadPerSec = 0.0,
            pivotAppliedVolts = 0.0,
            pivotSupplyCurrentAmps = 0.0,
            pivotTorqueCurrentAmps = 0.0,
            pivotPositionState = PivotPosition.STOW,
        )
    }

    data class PivotIOData(
        var pivotConnected: Boolean,
        var pivotPositionRad: Double,
        var pivotVelocityRadPerSec: Double,
        var pivotAppliedVolts: Double,
        var pivotSupplyCurrentAmps: Double,
        var pivotTorqueCurrentAmps: Double,
        var pivotPositionState: PivotPosition,
    ) : ReflectiveLoggableInputs()

    enum class PivotPosition(val position : Double) {
        STOW(0.0),
        DEPLOY(10.0),
    }

    /**
     * Updates the inputs for the pivot subsystem. Controls all variable data for the IO.
     *
     * @param inputs Input data to be passed into pivot logic
     */
    fun updateInputs(inputs: PivotIOInputs) {}

    /**
     * sets the power that will be applied to the motor
     *
     * @param power the power applied to the motor between -1 and 1.
     */
    fun setPower(power: Double) {}

    /**
     * sets the pivot state, which sets position in periodic
     *
     * @param state the state to set the pivot to
     */
    fun setPivotState(state: PivotPosition) {}

    /** Disables the motor  */
    fun disablePower() {}

    /**
     * sets the current limit for the pivot
     *
     * @param currentLimit the maximum current limit
     */
    fun setCurrentLimit(currentLimit: Int) {}
}
