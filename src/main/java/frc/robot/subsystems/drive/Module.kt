package frc.robot.subsystems.drive

import frc.robot.utils.RobotParameters.SwerveParameters
import org.littletonrobotics.junction.Logger
import org.wpilib.driverstation.RobotState
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.kinematics.SwerveModuleVelocity
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

    /** Runs the module with the specified setpoint state.
     * 1. Optimizes the setpoint state to minimize rotation. Flips spin direction if the inverse direction is closer
     * 2. Scales the speed by cosine of error to minimize inaccuracy before the module reaches the desired state
     *
     * @param state The setpoint state to run the module with.
     */
    fun runSetpoint(state: SwerveModuleVelocity) {
        // Optimize state
        val optimized = state.optimize(this.angle).cosineScale(inputs.turnPosition)

        // Apply setpoints
        val speedRadPerSec: Double = optimized.velocity / SwerveParameters.PhysicalParameters.WHEEL_DIAMETER / 2
        io.runDriveVelocity(speedRadPerSec, ffModel.calculate(speedRadPerSec))
        io.runTurnPosition(optimized.angle)
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

    /** Returns the current turn angle of the module.  */
    val angle: Rotation2d
        get() = inputs.turnPosition

    /** Returns the current drive position of the module in meters.  */
    val positionMeters: Double
        get() = inputs.drivePositionRad * SwerveParameters.PhysicalParameters.WHEEL_DIAMETER / 2.0

    /** Returns the current drive velocity of the module in meters per second.  */
    val velocityMetersPerSec: Double
        get() = inputs.driveVelocityRadPerSec * SwerveParameters.PhysicalParameters.WHEEL_DIAMETER / 2.0

    /** Returns the module position (turn angle and drive position).  */
    val position: SwerveModulePosition
        get() = SwerveModulePosition(this.positionMeters, this.angle)

    /** Returns the module state (turn angle and drive velocity).  */
    val state: SwerveModuleVelocity
        get() = SwerveModuleVelocity(this.velocityMetersPerSec, this.angle)

    /** Returns the module position in radians.  */
    val wheelRadiusCharacterizationPosition: Double
        get() = inputs.drivePositionRad

    /** Returns the module velocity in rotations/sec (Phoenix native units).  */
    val fFCharacterizationVelocity: Double
        get() = Units.radiansToRotations(inputs.driveVelocityRadPerSec)
}
