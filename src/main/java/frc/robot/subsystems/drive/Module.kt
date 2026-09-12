package frc.robot.subsystems.drive

import frc.robot.utils.RobotParameters.SwerveParameters
import org.littletonrobotics.junction.Logger
import org.wpilib.driverstation.RobotState
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.util.Units
import org.wpilib.util.Alert
import org.wpilib.util.Alert.Level

class Module(
    private val io: ModuleIO,
    private val index: Int,
) {
    private val inputs = ModuleIOInputsAutoLogged()

    private val ffModel = SwerveParameters.PIDParameters.DRIVE_FF

    private val driveDisconnectedAlert: Alert =
        Alert(
            "Module Alerts",
            "Disconnected drive motor on module $index.",
            Level.HIGH,
        )
    private val turnDisconnectedAlert: Alert =
        Alert(
            "Module Alerts",
            "Disconnected turn motor on module $index.",
            Level.HIGH,
        )

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drive/Module$index", inputs)

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected)
        turnDisconnectedAlert.set(!inputs.turnConnected)

        // Coast when disabled
        if (RobotState.isDisabled()) {
            io.coast()
        }
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it.  */
    fun runSetpoint(state: SwerveModuleState) {
        // Optimize velocity setpoint
        state.optimize(this.angle)
        state.cosineScale(inputs.turnPosition)

        // Apply setpoints
        val speedRadPerSec: Double = state.speed / DriveConstants.wheelRadius
        io.runDriveVelocity(speedRadPerSec, ffModel.calculate(speedRadPerSec))
        io.runTurnPosition(state.angle)
    }

    /** Runs the module with the specified output while controlling to zero degrees.  */
    fun runCharacterization(output: Double) {
        io.runDriveOpenLoop(output)
        io.runTurnPosition(Rotation2d())
    }

    /** Disables all outputs to motors.  */
    fun stop() {
        io.runDriveOpenLoop(0.0)
        io.runTurnOpenLoop(0.0)
    }

    val angle: Rotation2d
        /** Returns the current turn angle of the module.  */
        get() = inputs.turnPosition

    val positionMeters: Double
        /** Returns the current drive position of the module in meters.  */
        get() = inputs.drivePositionRad * SwerveParameters.PhysicalParameters.WHEEL_DIAMETER / 2.0

    val velocityMetersPerSec: Double
        /** Returns the current drive velocity of the module in meters per second.  */
        get() = inputs.driveVelocityRadPerSec * SwerveParameters.PhysicalParameters.WHEEL_DIAMETER / 2.0

    val position: SwerveModulePosition
        /** Returns the module position (turn angle and drive position).  */
        get() = SwerveModulePosition(this.positionMeters, this.angle)

    val state: SwerveModuleState?
        /** Returns the module state (turn angle and drive velocity).  */
        get() = SwerveModuleState(this.velocityMetersPerSec, this.angle)

    val wheelRadiusCharacterizationPosition: Double
        /** Returns the module position in radians.  */
        get() = inputs.drivePositionRad

    val fFCharacterizationVelocity: Double
        /** Returns the module velocity in rotations/sec (Phoenix native units).  */
        get() = Units.radiansToRotations(inputs.driveVelocityRadPerSec)
}
