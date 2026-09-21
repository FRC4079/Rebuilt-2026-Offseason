package frc.robot.subsystems.drive

import frc.robot.utils.LoggedTunableNumber
import frc.robot.utils.RobotParameters
import org.littletonrobotics.junction.Logger
import org.wpilib.math.controller.SimpleMotorFeedforward
import org.wpilib.math.filter.Debouncer
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.system.DCMotor
import org.wpilib.util.Alert

class Module(
    private val io: ModuleIO,
    private val index: Int,
) {
    companion object {
        private val drivekS: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekS")
        private val drivekV: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekV")
        private val drivekT: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekT")
        private val drivekP: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekP")
        private val drivekD: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekD")
        private val turnkP: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/TurnkP")
        private val turnkD: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/TurnkD")
    }

    init {
        drivekS.initDefault(5.0)
        drivekV.initDefault(0.0)
        // Multiplied by desired wheelTorqueNm
        drivekT.initDefault(ModuleIOComp.driveReduction / DCMotor.getKrakenX60Foc(1).Kt)
        drivekP.initDefault(35.0)
        drivekD.initDefault(0.0)
        turnkP.initDefault(4000.0)
        turnkD.initDefault(50.0)
    }

    private val inputs = ModuleIOInputsAutoLogged()

    private var ffModel: SimpleMotorFeedforward

    // Connected debouncers
    private val driveMotorConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)
    private val turnMotorConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)
    private val turnEncoderConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)

    // Connection alerts
    private val alertType: String = "Module Alerts"
    private val driveDisconnectedAlert: Alert
    private val turnDisconnectedAlert: Alert
    private val turnEncoderDisconnectedAlert: Alert
    private var odometryPositions: Array<SwerveModulePosition?> = arrayOf<SwerveModulePosition?>()

    init {
        ffModel = SimpleMotorFeedforward(drivekS.get(), drivekV.get())

        driveDisconnectedAlert =
            Alert(alertType, "Disconnected drive motor on module $index.", Alert.Level.HIGH)
        turnDisconnectedAlert =
            Alert(alertType, "Disconnected turn motor on module $index.", Alert.Level.HIGH)
        turnEncoderDisconnectedAlert =
            Alert(alertType, "Disconnected turn encoder on module $index.", Alert.Level.HIGH)
    }

    fun updateInputs() {
        io.updateInputs(inputs)
        Logger.processInputs("Drive/Module$index", inputs)
    }

    fun periodic() {
        // Update tunable numbers
        if (drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
            ffModel = SimpleMotorFeedforward(drivekS.get(), drivekV.get())
        }
        if (drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
            io.setDrivePID(drivekP.get(), 0.0, drivekD.get())
        }
        if (turnkP.hasChanged(hashCode()) || turnkD.hasChanged(hashCode())) {
            io.setTurnPID(turnkP.get(), 0.0, turnkD.get())
        }

        // Calculate positions for odometry
        val sampleCount = inputs.odometryDrivePositionsRad.size // All signals are sampled together
        odometryPositions = kotlin.arrayOfNulls<SwerveModulePosition>(sampleCount)
        for (i in 0..<sampleCount) {
            val positionMeters: Double =
                inputs.odometryDrivePositionsRad[i] * RobotParameters.SwerveParameters.PhysicalParameters.WHEEL_DIAMETER / 2
            val angle: Rotation2d? = inputs.odometryTurnPositions[i]
            odometryPositions[i] = SwerveModulePosition(positionMeters, angle)
        }

        // Update alerts
        driveDisconnectedAlert.set(
            !driveMotorConnectedDebouncer.calculate(inputs.driveConnected) && !Robot.isJITing(),
        )
        turnDisconnectedAlert.set(
            !turnMotorConnectedDebouncer.calculate(inputs.turnConnected) && !Robot.isJITing(),
        )
        turnEncoderDisconnectedAlert.set(
            !turnEncoderConnectedDebouncer.calculate(inputs.turnEncoderConnected) &&
                !Robot.isJITing(),
        )

        // Record cycle time
        LoggedTracer.record("Drive/Module$index")
    }

    /** Runs the module with the specified setpoint state.  */
    fun runSetpoint(state: SwerveModuleState) {
        // Apply setpoints
        val speedRadPerSec: Double = state.speedMetersPerSecond / DriveConstants.wheelRadius
        io.runDriveVelocity(speedRadPerSec, ffModel.calculate(speedRadPerSec))
        if (Math.abs(state.angle.minus(this.angle).getDegrees()) < DriveConstants.turnDeadbandDegrees) {
            io.runTurnOpenLoop(0.0)
        } else {
            io.runTurnPosition(state.angle)
        }
    }

    /**
     * Runs the module with the specified setpoint state and a setpoint wheel force used for
     * torque-based feedforward.
     */
    fun runSetpoint(
        state: SwerveModuleState,
        wheelTorqueNm: Double,
    ) {
        // Apply setpoints
        val speedRadPerSec: Double = state.speedMetersPerSecond / DriveConstants.wheelRadius
        io.runDriveVelocity(
            speedRadPerSec,
            ffModel.calculate(speedRadPerSec) + wheelTorqueNm * drivekT.get(),
        )
        if (Math.abs(state.angle.minus(this.angle).getDegrees()) < DriveConstants.turnDeadbandDegrees) {
            io.runTurnOpenLoop(0.0)
        } else {
            io.runTurnPosition(state.angle)
        }
    }

    /** Runs the module with the specified output while controlling to zero degrees.  */
    fun runCharacterization(output: Double) {
        io.runDriveOpenLoop(output)
        io.runTurnPosition(Rotation2d.kZero)
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
        get() = inputs.drivePositionRad * DriveConstants.wheelRadius

    val velocityMetersPerSec: Double
        /** Returns the current drive velocity of the module in meters per second.  */
        get() = inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius

    val position: SwerveModulePosition?
        /** Returns the module position (turn angle and drive position).  */
        get() = SwerveModulePosition(this.positionMeters, this.angle)

    val state: SwerveModuleState?
        /** Returns the module state (turn angle and drive velocity).  */
        get() = SwerveModuleState(this.velocityMetersPerSec, this.angle)

    /** Returns the module positions received this cycle.  */
    fun getOdometryPositions(): Array<SwerveModulePosition?> = odometryPositions

    val wheelRadiusCharacterizationPosition: Double
        /** Returns the module position in radians.  */
        get() = inputs.drivePositionRad

    val fFCharacterizationVelocity: Double
        /** Returns the module velocity in rotations/sec (Phoenix native units).  */
        get() = Units.radiansToRotations(inputs.driveVelocityRadPerSec)

    // Sets brake mode to {@code enabled}
    fun setBrakeMode(enabled: Boolean) {
        io.setBrakeMode(enabled)
    }
}
