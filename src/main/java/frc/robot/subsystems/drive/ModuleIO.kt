package frc.robot.subsystems.drive
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.wpilib.math.geometry.Rotation2d

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
) : LoggableInputs {
    override fun toLog(table: LogTable) {
        table.put("DriveConnected", driveConnected)
        table.put("DrivePositionRad", drivePositionRad)
        table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec)
        table.put("DriveAppliedVolts", driveAppliedVolts)
        table.put("DriveSupplyCurrentAmps", driveSupplyCurrentAmps)
        table.put("DriveTorqueCurrentAmps", driveTorqueCurrentAmps)
        table.put("TurnConnected", turnConnected)
        table.put("TurnEncoderConnected", turnEncoderConnected)
        table.put("TurnAbsolutePosition", Rotation2d.struct, turnAbsolutePosition)
        table.put("TurnPosition", Rotation2d.struct, turnPosition)
        table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec)
        table.put("TurnAppliedVolts", turnAppliedVolts)
        table.put("TurnSupplyCurrentAmps", turnSupplyCurrentAmps)
        table.put("TurnTorqueCurrentAmps", turnTorqueCurrentAmps)
    }

    override fun fromLog(table: LogTable) {
        driveConnected = table.get("DriveConnected", driveConnected)
        drivePositionRad = table.get("DrivePositionRad", drivePositionRad)
        driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec)
        driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts)
        driveSupplyCurrentAmps = table.get("DriveSupplyCurrentAmps", driveSupplyCurrentAmps)
        driveTorqueCurrentAmps = table.get("DriveTorqueCurrentAmps", driveTorqueCurrentAmps)
        turnConnected = table.get("TurnConnected", turnConnected)
        turnEncoderConnected = table.get("TurnEncoderConnected", turnEncoderConnected)
        turnAbsolutePosition = table.get("TurnAbsolutePosition", Rotation2d.struct, turnAbsolutePosition)
        turnPosition = table.get("TurnPosition", Rotation2d.struct, turnPosition)
        turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", turnVelocityRadPerSec)
        turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts)
        turnSupplyCurrentAmps = table.get("TurnSupplyCurrentAmps", turnSupplyCurrentAmps)
        turnTorqueCurrentAmps = table.get("TurnTorqueCurrentAmps", turnTorqueCurrentAmps)
    }
}

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
