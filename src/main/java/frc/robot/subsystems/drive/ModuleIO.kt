package frc.robot.subsystems.drive
import frc.robot.utils.logging.ReflectiveLoggableInputs
import org.littletonrobotics.junction.AutoLog
import org.wpilib.math.geometry.Rotation2d

interface ModuleIO {
    @AutoLog
    open class ModuleIOInputs {
        @JvmField
        var data: ModuleIOData =
            ModuleIOData(
                driveConnected = false,
                drivePositionRad = 0.0,
                driveVelocityRadPerSec = 0.0,
                driveAppliedVolts = 0.0,
                driveSupplyCurrentAmps = 0.0,
                driveTorqueCurrentAmps = 0.0,
                turnConnected = false,
                turnEncoderConnected = false,
                turnAbsolutePosition = Rotation2d.ZERO,
                turnPosition = Rotation2d.ZERO,
                turnVelocityRadPerSec = 0.0,
                turnAppliedVolts = 0.0,
                turnSupplyCurrentAmps = 0.0,
                turnTorqueCurrentAmps = 0.0,
            )

        @JvmField
        var odometryDrivePositionsRad: DoubleArray = doubleArrayOf()

        @JvmField
        var odometryTurnPositions: Array<Rotation2d?> = arrayOf<Rotation2d?>()
    }

    data class ModuleIOData(
        var driveConnected: Boolean,
        var drivePositionRad: Double,
        var driveVelocityRadPerSec: Double,
        var driveAppliedVolts: Double,
        var driveSupplyCurrentAmps: Double,
        var driveTorqueCurrentAmps: Double,
        var turnConnected: Boolean,
        var turnEncoderConnected: Boolean,
        var turnAbsolutePosition: Rotation2d?,
        var turnPosition: Rotation2d?,
        var turnVelocityRadPerSec: Double,
        var turnAppliedVolts: Double,
        var turnSupplyCurrentAmps: Double,
        var turnTorqueCurrentAmps: Double,
    ) : ReflectiveLoggableInputs()

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs) {}

    /** Run the drive motor at the specified open loop value.  */
    fun runDriveOpenLoop(output: Double) {}

    /** Run the turn motor at the specified open loop value.  */
    fun runTurnOpenLoop(output: Double) {}

    /** Run the drive motor at the specified velocity.  */
    fun runDriveVelocity(
        velocityRadPerSec: Double,
        feedforward: Double,
    ) {}

    /** Run the turn motor to the specified rotation.  */
    fun runTurnPosition(rotation: Rotation2d) {}

    /** Set P, I, and D gains for closed loop control on drive motor.  */
    fun setDrivePID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {}

    /** Set P, I, and D gains for closed loop control on turn motor.  */
    fun setTurnPID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {}

    /** Set brake mode on drive motor  */
    fun setBrakeMode(enabled: Boolean) {}
}
