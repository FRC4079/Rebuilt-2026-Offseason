package frc.robot.mechanisms.hood

import frc.robot.utils.logging.ReflectiveLoggableInputs
import org.littletonrobotics.junction.AutoLog
import org.wpilib.units.Units.*

interface HoodIO {
    @AutoLog
    open class HoodIOInputs {
        @JvmField
        var data: HoodIOData = HoodIOData(
            hoodConnected = false,
            hoodPositionRad = 0.0,
            hoodVelocityRadPerSec = 0.0,
            hoodAppliedVolts = 0.0,
            hoodSupplyCurrentAmps = 0.0,
            hoodTorqueCurrentAmps = 0.0,
            hoodPositionState = HoodPositionState.HOME,
        )
    }

    data class HoodIOData(
        var hoodConnected: Boolean,
        var hoodPositionRad: Double,
        var hoodVelocityRadPerSec: Double,
        var hoodAppliedVolts: Double,
        var hoodSupplyCurrentAmps: Double,
        var hoodTorqueCurrentAmps: Double,
        var hoodPositionState: HoodPositionState,
    ) : ReflectiveLoggableInputs()

    enum class HoodPositionState(val positionRad: Double) {
        HOME(0.0),
        STOW(0.0),
        AIM(0.0),
    }

    /**
     * Updates the inputs for the hood subsystem. Controls all variable data for the IO.
     *
     * @param inputs Input data to be passed into hood logic
     */
    fun updateInputs(inputs: HoodIOInputs) {}

    /**
     * sets the power that will be applied to the motor
     *
     * @param power the power applied to the motor between -1 and 1.
     */
    fun setPower(power: Double) {}

    /**
     * sets the hood state, which sets position in periodic
     *
     * @param state the state to set the hood to
     */
    fun setHoodState(state: HoodPositionState) {}

    /** Disables the motor  */
    fun disablePower() {}

    /**
     * sets the current limit for the hood
     *
     * @param currentLimit the maximum current limit
     */
    fun setCurrentLimit(currentLimit: Int) {}
}