package frc.robot.subsystems.drive

import org.littletonrobotics.junction.AutoLog
import org.wpilib.math.geometry.Rotation2d

interface ModuleIO {
    @AutoLog
    data class ModuleIOInputs(
        var driveConnected: Boolean = false,
        var drivePositionRad: Double = 0.0,
        var driveVelocityRadPerSec: Double = 0.0,
        var driveAppliedVolts: Double = 0.0,
        var driveSupplyCurrentAmps: Double = 0.0,
        var driveTorqueCurrentAmps: Double = 0.0,
        var turnConnected: Boolean = false,
        var turnAbsolutePosition: Rotation2d = Rotation2d(),
        var turnPosition: Rotation2d = Rotation2d(),
        var turnVelocityRadPerSec: Double = 0.0,
        var turnAppliedVolts: Double = 0.0,
        var turnSupplyCurrentAmps: Double = 0.0,
        var turnTorqueCurrentAmps: Double = 0.0,
    )

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs?) {}

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
    fun runTurnPosition(rotation: Rotation2d?) {}

    /** Run in coast mode.  */
    fun coast() {}
}
