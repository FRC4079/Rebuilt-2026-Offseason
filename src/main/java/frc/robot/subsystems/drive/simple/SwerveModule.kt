package frc.robot.subsystems.drive.simple

import frc.robot.Robot
import frc.robot.subsystems.drive.ModuleIO
import frc.robot.subsystems.drive.ModuleIOInputsAutoLogged
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.RobotParameters.SwerveParameters.PIDParameters
import frc.robot.utils.logging.LoggedTracer
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.wpilib.math.filter.Debouncer
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.math.util.Units
import org.wpilib.util.Alert
import kotlin.math.abs

/** Represents a swerve module used in a swerve drive system. */
class SwerveModule(
    private val io: ModuleIO,
    private val index: Int,
) {
    private val inputs = ModuleIOInputsAutoLogged()

    private val driveP = LoggedNetworkNumber("/Tuning/Swerve/$index/Drive P", PIDParameters.DRIVE_PID_AUTO.p)
    private val driveI = LoggedNetworkNumber("/Tuning/Swerve/$index/Drive I", PIDParameters.DRIVE_PID_AUTO.i)
    private val driveD = LoggedNetworkNumber("/Tuning/Swerve/$index/Drive D", PIDParameters.DRIVE_PID_AUTO.d)

    private val steerP = LoggedNetworkNumber("/Tuning/Swerve/$index/Steer P", PIDParameters.STEER_PID_AUTO.p)
    private val steerI = LoggedNetworkNumber("/Tuning/Swerve/$index/Steer I", PIDParameters.STEER_PID_AUTO.i)
    private val steerD = LoggedNetworkNumber("/Tuning/Swerve/$index/Steer D", PIDParameters.STEER_PID_AUTO.d)

    private val driveMotorConnectedDebouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)
    private val turnMotorConnectedDebouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)
    private val turnEncoderConnectedDebouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)

    private val driveDisconnectedAlert =
        Alert(
            "Swerve Module Alerts",
            "Disconnected drive motor on module $index.",
            Alert.Level.HIGH,
        )

    private val turnDisconnectedAlert =
        Alert(
            "Swerve Module Alerts",
            "Disconnected turn motor on module $index.",
            Alert.Level.HIGH,
        )

    private val turnEncoderDisconnectedAlert =
        Alert(
            "Swerve Module Alerts",
            "Disconnected turn encoder on module $index.",
            Alert.Level.HIGH,
        )

    private var odometryPositions: Array<SwerveModulePosition?> = arrayOf()

    fun updateInputs() {
        io.updateInputs(inputs)
        Logger.processInputs("Swerve/Module$index", inputs)
    }

    fun periodic() {
        val sampleCount = inputs.odometryDrivePositionsRad.size
        odometryPositions = arrayOfNulls(sampleCount)

        for (i in 0 until sampleCount) {
            odometryPositions[i] =
                SwerveModulePosition(
                    inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS_METERS,
                    inputs.odometryTurnPositions[i] ?: Rotation2d.ZERO,
                )
        }

        driveDisconnectedAlert.set(
            !driveMotorConnectedDebouncer.calculate(inputs.data.driveConnected) && !Robot.isJITing(),
        )
        turnDisconnectedAlert.set(
            !turnMotorConnectedDebouncer.calculate(inputs.data.turnConnected) && !Robot.isJITing(),
        )
        turnEncoderDisconnectedAlert.set(
            !turnEncoderConnectedDebouncer.calculate(inputs.data.turnEncoderConnected) && !Robot.isJITing(),
        )

        LoggedTracer.record("Swerve/Module$index")
    }

    fun runSetpoint(state: SwerveModuleVelocity) {
        val currentAngle = angle ?: Rotation2d.ZERO
        val optimized = state.optimize(currentAngle).cosineScale(currentAngle)

        if (abs(optimized.velocity) < SwerveParameters.Thresholds.STATE_SPEED_THRESHOLD) {
            io.runDriveOpenLoop(0.0)
        } else {
            io.runDriveVelocity(optimized.velocity / WHEEL_RADIUS_METERS, 0.0)
        }

        if (abs(optimized.angle.minus(currentAngle).degrees) < SwerveParameters.Thresholds.TURN_DEADBAND_DEGREES) {
            io.runTurnOpenLoop(0.0)
        } else {
            io.runTurnPosition(optimized.angle)
        }
    }

    var state: SwerveModuleVelocity
        get() = SwerveModuleVelocity(velocityMetersPerSec, angle ?: Rotation2d.ZERO)
        set(value) {
            runSetpoint(value)
        }

    val position: SwerveModulePosition
        get() = SwerveModulePosition(positionMeters, angle ?: Rotation2d.ZERO)

    val angle: Rotation2d?
        get() = inputs.data.turnPosition ?: inputs.data.turnAbsolutePosition

    val positionMeters: Double
        get() = inputs.data.drivePositionRad * WHEEL_RADIUS_METERS

    val velocityMetersPerSec: Double
        get() = inputs.data.driveVelocityRadPerSec * WHEEL_RADIUS_METERS

    fun getOdometryPositions(): Array<SwerveModulePosition?> = odometryPositions

    fun stop() {
        io.runDriveOpenLoop(0.0)
        io.runTurnOpenLoop(0.0)
    }

    fun setDrivePID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        io.setDrivePID(kP, kI, kD)
    }

    fun setSteerPID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        io.setTurnPID(kP, kI, kD)
    }

    fun setTelePID() {
        setDrivePID(
            PIDParameters.DRIVE_PID_TELE.p,
            PIDParameters.DRIVE_PID_TELE.i,
            PIDParameters.DRIVE_PID_TELE.d,
        )
        setSteerPID(
            PIDParameters.STEER_PID_TELE.p,
            PIDParameters.STEER_PID_TELE.i,
            PIDParameters.STEER_PID_TELE.d,
        )
    }

    fun setAutoPID() {
        setDrivePID(
            PIDParameters.DRIVE_PID_AUTO.p,
            PIDParameters.DRIVE_PID_AUTO.i,
            PIDParameters.DRIVE_PID_AUTO.d,
        )
        setSteerPID(
            PIDParameters.STEER_PID_AUTO.p,
            PIDParameters.STEER_PID_AUTO.i,
            PIDParameters.STEER_PID_AUTO.d,
        )
    }

    fun applyTelePIDValues() {
        io.setDrivePID(driveP.get(), driveI.get(), driveD.get())
        io.setTurnPID(steerP.get(), steerI.get(), steerD.get())
    }

    fun updateTelePID() {
        PIDParameters.DRIVE_PID_TELE.p = driveP.get()
        PIDParameters.DRIVE_PID_TELE.i = driveI.get()
        PIDParameters.DRIVE_PID_TELE.d = driveD.get()

        PIDParameters.STEER_PID_TELE.p = steerP.get()
        PIDParameters.STEER_PID_TELE.i = steerI.get()
        PIDParameters.STEER_PID_TELE.d = steerD.get()

        applyTelePIDValues()
    }

    fun resetDrivePosition() {
        // RealModuleIO currently zeros the drive position during hardware construction.
        // Add resetDrivePosition() to ModuleIO if runtime zeroing is needed.
    }

    fun setBrakeMode(enabled: Boolean) {
        io.setBrakeMode(enabled)
    }

    val wheelRadiusCharacterizationPosition: Double
        get() = inputs.data.drivePositionRad

    val fFCharacterizationVelocity: Double
        get() = Units.radiansToRotations(inputs.data.driveVelocityRadPerSec)

    companion object {
        private const val WHEEL_RADIUS_METERS =
            SwerveParameters.PhysicalParameters.WHEEL_DIAMETER / 2.0
    }
}
