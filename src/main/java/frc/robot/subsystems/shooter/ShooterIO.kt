package frc.robot.subsystems.shooter

import frc.robot.utils.logging.ReflectiveLoggableInputs
import org.littletonrobotics.junction.AutoLog
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.units.Units.*

interface ShooterIO {
    @AutoLog
    open class ShooterIOInputs {
        @JvmField
        var data: ShooterIOData = ShooterIOData(
            topMotorConnected = false,
            bottomMotorConnected = false,
            topPositionRad = 0.0,
            bottomPositionRad = 0.0,
            topVelocityRadPerSec = 0.0,
            bottomVelocityRadPerSec = 0.0,
            topAppliedVolts = 0.0,
            bottomAppliedVolts = 0.0,
            topSupplyCurrentAmps = 0.0,
            bottomSupplyCurrentAmps = 0.0,
            topTorqueCurrentAmps = 0.0,
            bottomTorqueCurrentAmps = 0.0,
            shooterState = ShooterState.IDLE,
        )
    }

    data class ShooterIOData(
        var topMotorConnected: Boolean,
        var bottomMotorConnected: Boolean,
        var topPositionRad: Double,
        var bottomPositionRad: Double,
        var topVelocityRadPerSec: Double,
        var bottomVelocityRadPerSec: Double,
        var topAppliedVolts: Double,
        var bottomAppliedVolts: Double,
        var topSupplyCurrentAmps: Double,
        var bottomSupplyCurrentAmps: Double,
        var topTorqueCurrentAmps: Double,
        var bottomTorqueCurrentAmps: Double,
        var shooterState: ShooterState,
    ) : ReflectiveLoggableInputs()

    enum class ShooterState(val velocityRadPerSec: Double) {
        IDLE(0.0),
        SPINUP(300.0),
        SHOOT(400.0),
    }

    /**
     * Updates the inputs for the shooter subsystem. Controls all variable data for the IO.
     *
     * @param inputs Input data to be passed into shooter logic
     */
    fun updateInputs(inputs: ShooterIOInputs) {}

    /**
     * sets the power that will be applied to the motors
     *
     * @param topPower the power applied to the top motor between -1 and 1.
     * @param bottomPower the power applied to the bottom motor between -1 and 1.
     */
    fun setPower(topPower: Double, bottomPower: Double) {}

    /**
     * sets the shooter state, which sets velocity in periodic
     *
     * @param state the state to set the shooter to
     */
    fun setShooterState(state: ShooterState) {}

    /** Disables the motors  */
    fun disablePower() {}

    /**
     * sets the current limit for the shooter
     *
     * @param currentLimit the maximum current limit
     */
    fun setCurrentLimit(currentLimit: Int) {}
}