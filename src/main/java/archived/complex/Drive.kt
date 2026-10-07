package archived.complex

import frc.robot.Robot
import frc.robot.mechanisms.drive.GyroIOInputsAutoLogged
import frc.robot.mechanisms.drive.gyro.GyroIO
import frc.robot.mechanisms.drive.module.ModuleIO
import frc.robot.utils.RobotParameters
import frc.robot.utils.logging.LoggedTracer
import frc.robot.utils.logging.LoggedTunableNumber
import frc.robot.utils.phoenix.PhoenixOdometryThread
import frc.robot.utils.swerve.ModuleLimits
import frc.robot.utils.swerve.SwerveSetpoint
import frc.robot.utils.swerve.SwerveSetpointGenerator
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.driverstation.RobotState
import org.wpilib.math.filter.Debouncer
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveDriveKinematics
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.math.linalg.Vector
import org.wpilib.math.numbers.N2
import org.wpilib.system.Timer
import org.wpilib.util.Alert
import java.util.Optional
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.abs

class Drive(
    private val gyroIO: GyroIO,
    flModuleIO: ModuleIO,
    frModuleIO: ModuleIO,
    blModuleIO: ModuleIO,
    brModuleIO: ModuleIO,
) : Mechanism {
    private val gyroInputs = GyroIOInputsAutoLogged()
    private val modules =
        arrayOf(
            Module(flModuleIO, 0),
            Module(frModuleIO, 1),
            Module(blModuleIO, 2),
            Module(brModuleIO, 3),
        )

    private val gyroConnectedDebouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)
    private val gyroDisconnectedAlert =
        Alert(
            "Drive Alert",
            "Disconnected gyro, using kinematics as fallback.",
            Alert.Level.HIGH,
        )
    private val lastMovementTimer = Timer()

    private val kinematics: SwerveDriveKinematics = RobotParameters.SwerveParameters.PhysicalParameters.kinematics
    private val moduleLocations = RobotParameters.SwerveParameters.PhysicalParameters.MODULE_LOCATIONS

    @AutoLogOutput
    private var velocityMode = false

    @AutoLogOutput
    private var brakeModeEnabled = true

    @AutoLogOutput
    private var coastRequest = CoastRequest.ALWAYS_BRAKE

    private var currentSetpoint =
        SwerveSetpoint(
            ChassisVelocities(0.0, 0.0, 0.0),
            Array(modules.size) { SwerveModuleVelocity(0.0, Rotation2d.ZERO) },
        )

    private val swerveSetpointGenerator = SwerveSetpointGenerator(kinematics, moduleLocations)

    enum class CoastRequest {
        AUTOMATIC,
        ALWAYS_BRAKE,
        ALWAYS_COAST,
    }

    init {
        lastMovementTimer.start()
        setBrakeMode(true)
        PhoenixOdometryThread.getInstance().start()
    }

    fun periodic() {
        odometryLock.lock()
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)
        for (module in modules) {
            module.updateInputs()
        }
        odometryLock.unlock()
        LoggedTracer.record("Drive/Inputs")

        for (module in modules) {
            module.periodic()
        }

        if (RobotState.isDisabled()) {
            for (module in modules) {
                module.stop()
            }
            Logger.recordOutput("Drive/SwerveStates/Setpoints", SwerveModuleVelocity.struct)
            Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", SwerveModuleVelocity.struct)
        }

        val sampleTimestamps =
            if (RobotParameters.SwerveParameters.getMode() == RobotParameters.Mode.SIM) {
                doubleArrayOf(Timer.getTimestamp())
            } else {
                gyroInputs.odometryYawTimestamps
            }

        for (i in sampleTimestamps.indices) {
            val wheelPositions = Array(modules.size) { SwerveModulePosition() }
            for (j in modules.indices) {
                wheelPositions[j] = modules[j].getOdometryPositions()[i] ?: SwerveModulePosition()
            }

            val yaw =
                if (gyroInputs.data.connected) {
                    Optional.of(gyroInputs.odometryYawPositions[i])
                } else {
                    Optional.empty()
                }

            frc.robot.RobotState.getInstance().addOdometryObservation(
                frc.robot.RobotState.OdometryObservation(
                    timestamp = sampleTimestamps[i],
                    wheelPositions = wheelPositions,
                    yaw = yaw,
                ),
            )
        }

        frc.robot.RobotState
            .getInstance()
            .addDriveSpeeds(chassisSpeeds)
        frc.robot.RobotState
            .getInstance()
            .setPitch(gyroInputs.data.pitchPosition)
        frc.robot.RobotState
            .getInstance()
            .setRoll(gyroInputs.data.rollPosition)

        Logger.recordOutput(
            "RobotState/EstimatedPose3d",
            Pose3d(
                frc.robot.RobotState
                    .getInstance()
                    .estimatedPose,
            ),
        )

        if (modules.any { abs(it.velocityMetersPerSec) > coastMetersPerSecondThreshold.get() }) {
            lastMovementTimer.reset()
        }

        if (RobotState.isEnabled()) {
            coastRequest = CoastRequest.ALWAYS_BRAKE
        }

        when (coastRequest) {
            CoastRequest.AUTOMATIC -> {
                if (RobotState.isEnabled()) {
                    setBrakeMode(true)
                } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
                    setBrakeMode(false)
                }
            }

            CoastRequest.ALWAYS_BRAKE -> {
                setBrakeMode(true)
            }

            CoastRequest.ALWAYS_COAST -> {
                setBrakeMode(false)
            }
        }

        if (!velocityMode) {
            currentSetpoint = SwerveSetpoint(chassisSpeeds, moduleStates)
        }

        gyroDisconnectedAlert.set(
            !gyroConnectedDebouncer.calculate(gyroInputs.data.connected) &&
                RobotParameters.SwerveParameters.getMode() != RobotParameters.Mode.SIM &&
                !Robot.isJITing(),
        )

        LoggedTracer.record("Drive/Periodic")
    }

    private fun setBrakeMode(enabled: Boolean) {
        if (brakeModeEnabled != enabled) {
            modules.forEach { it.setBrakeMode(enabled) }
        }
        brakeModeEnabled = enabled
    }

    fun runVelocity(speeds: ChassisVelocities) {
        velocityMode = true
        val discreteSpeeds = speeds.discretize(LOOP_PERIOD_SECS)
        val setpointStatesUnoptimized = kinematics.toSwerveModuleVelocities(discreteSpeeds)
        currentSetpoint =
            swerveSetpointGenerator.generateSetpoint(
                moduleLimits,
                currentSetpoint,
                discreteSpeeds,
                LOOP_PERIOD_SECS,
            ) ?: currentSetpoint
        val setpointStates = currentSetpoint.moduleStates

        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", SwerveModuleVelocity.struct, *setpointStatesUnoptimized)
        Logger.recordOutput("Drive/SwerveStates/Setpoints", SwerveModuleVelocity.struct, *setpointStates)
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisVelocities)

        for (i in modules.indices) {
            modules[i].runSetpoint(setpointStates[i])
        }
    }

    fun runVelocity(
        speeds: ChassisVelocities,
        moduleForces: List<Vector<N2>>,
    ) {
        velocityMode = true
        val discreteSpeeds = speeds.discretize(LOOP_PERIOD_SECS)
        val setpointStatesUnoptimized = kinematics.toSwerveModuleVelocities(discreteSpeeds)
        currentSetpoint =
            swerveSetpointGenerator.generateSetpoint(
                moduleLimits,
                currentSetpoint,
                discreteSpeeds,
                LOOP_PERIOD_SECS,
            ) ?: currentSetpoint
        val setpointStates = currentSetpoint.moduleStates
        val wheelForces = Array(modules.size) { SwerveModuleVelocity() }

        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", SwerveModuleVelocity.struct, *setpointStatesUnoptimized)
        Logger.recordOutput("Drive/SwerveStates/Setpoints", SwerveModuleVelocity.struct, *setpointStates)
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisVelocities)

        for (i in modules.indices) {
            val wheelAngle = modules[i].state?.angle ?: Rotation2d.ZERO
            val optimized = setpointStates[i].optimize(wheelAngle).cosineScale(wheelAngle)
            setpointStates[i] = optimized

            val wheelDirection = VecBuilder.fill(wheelAngle.cos, wheelAngle.sin)
            val wheelTorqueNm = moduleForces[i].dot(wheelDirection) * wheelRadiusMeters
            modules[i].runSetpoint(optimized, wheelTorqueNm)

            wheelForces[i] = SwerveModuleVelocity(wheelTorqueNm, optimized.angle)
        }
        Logger.recordOutput("Drive/SwerveStates/ModuleForces", SwerveModuleVelocity.struct, *wheelForces)
    }

    fun runCharacterization(output: Double) {
        velocityMode = false
        for (module in modules) {
            module.runCharacterization(output)
        }
    }

    fun stop() {
        runVelocity(ChassisVelocities(0.0, 0.0, 0.0))
    }

    fun stopWithX() {
        val headings = Array(moduleLocations.size) { i -> moduleLocations[i].angle.orElse(Rotation2d.ZERO) }
        kinematics.resetHeadings(*headings)

        val states = kinematics.toSwerveModuleVelocities(ChassisVelocities(0.0, 0.0, 0.0))
        for (i in modules.indices) {
            val optimized = states[i].optimize(modules[i].angle ?: Rotation2d.ZERO)
            modules[i].runSetpoint(optimized)
        }
    }

    @get:AutoLogOutput(key = "Drive/SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleVelocity>
        get() = Array(modules.size) { i -> modules[i].state ?: SwerveModuleVelocity() }

    @get:AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
    private val chassisSpeeds: ChassisVelocities
        get() = kinematics.toChassisVelocities(*moduleStates)

    val wheelRadiusCharacterizationPositions: DoubleArray
        get() = DoubleArray(modules.size) { i -> modules[i].wheelRadiusCharacterizationPosition }

    val fFCharacterizationVelocity: Double
        get() = modules.sumOf { it.fFCharacterizationVelocity } / modules.size.toDouble()

    val gyroRotation: Rotation2d
        get() = gyroInputs.data.yawPosition

    companion object {
        private const val LOOP_PERIOD_SECS = 0.02

        val odometryLock: Lock = ReentrantLock()
        private val coastWaitTime = LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5)
        private val coastMetersPerSecondThreshold =
            LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", 0.05)

        private val moduleLimits =
            ModuleLimits(
                RobotParameters.SwerveParameters.PhysicalParameters.MAX_SPEED,
                RobotParameters.SwerveParameters.PhysicalParameters.MAX_SPEED * 2.0,
                RobotParameters.SwerveParameters.PhysicalParameters.MAX_ANGULAR_SPEED,
            )

        private val wheelRadiusMeters = RobotParameters.SwerveParameters.PhysicalParameters.WHEEL_DIAMETER / 2.0
    }
}
