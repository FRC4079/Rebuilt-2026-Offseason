package frc.robot.subsystems.drive

import frc.robot.utils.RobotParameters
import frc.robot.utils.phoenix.PhoenixOdometryThread
import frc.robot.utils.swerve.SwerveSetpoint
import lombok.Setter
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.math.filter.Debouncer
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveDriveKinematics
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.system.Timer
import org.wpilib.util.Alert
import java.lang.Module
import java.util.Arrays
import java.util.Optional
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.collections.get
import kotlin.text.get

class Drive(
    private val gyroIO: GyroIO,
    flModuleIO: ModuleIO?,
    frModuleIO: ModuleIO?,
    blModuleIO: ModuleIO?,
    brModuleIO: ModuleIO?,
) : Mechanism() {
    private val gyroInputs = GyroIOInputsAutoLogged()
    private val modules: Array<Module> = arrayOfNulls<Module>(4) as Array<Module> // FL, FR, BL, BR
    private val gyroConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)
    private val gyroDisconnectedAlert: Alert =
        Alert(
            "Drive Alert",
            "Disconnected gyro, using kinematics as fallback.",
            Alert.Level.HIGH,
        )

    private val lastMovementTimer: Timer = Timer()

    private val kinematics: SwerveDriveKinematics = RobotParameters.SwerveParameters.PhysicalParameters.kinematics

    @AutoLogOutput
    private var velocityMode = false

    @AutoLogOutput
    private var brakeModeEnabled = true

    private var currentSetpoint: SwerveSetpoint =
        SwerveSetpoint(
            ChassisVelocities(0.0, 0.0, 0.0),
            arrayOf(
                SwerveModuleVelocity(0.0, Rotation2d.ZERO),
                SwerveModuleVelocity(0.0, Rotation2d.ZERO),
                SwerveModuleVelocity(0.0, Rotation2d.ZERO),
                SwerveModuleVelocity(0.0, Rotation2d.ZERO),
            ),
        )
    private val swerveSetpointGenerator: SwerveSetpointGenerator

    enum class CoastRequest {
        AUTOMATIC,
        ALWAYS_BRAKE,
        ALWAYS_COAST,
    }

    @Setter
    @AutoLogOutput
    private var coastRequest = CoastRequest.ALWAYS_BRAKE

    init {
        modules[0] = Module(flModuleIO, 0)
        modules[1] = Module(frModuleIO, 1)
        modules[2] = Module(blModuleIO, 2)
        modules[3] = Module(brModuleIO, 3)
        lastMovementTimer.start()
        setBrakeMode(true)

        swerveSetpointGenerator =
            SwerveSetpointGenerator(kinematics, DriveConstants.moduleTranslations)

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start()
    }

    public override fun periodic() {
        odometryLock.lock() // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)
        for (module in modules) {
            module.updateInputs()
        }
        odometryLock.unlock()
        LoggedTracer.record("Drive/Inputs")

        // Call periodic on modules
        for (module in modules) {
            module.periodic()
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (module in modules) {
                module.stop()
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Drive/SwerveStates/Setpoints", arrayOf<SwerveModuleState?>())
            Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", arrayOf<SwerveModuleState?>())
        }

        // Send odometry updates to robot state
        val sampleTimestamps =
            if (Constants.getMode() === Mode.SIM) {
                doubleArrayOf(Timer.getTimestamp())
            } else {
                gyroInputs.odometryYawTimestamps // All signals are sampled together
            }
        val sampleCount = sampleTimestamps.size
        for (i in 0..<sampleCount) {
            val wheelPositions: Array<SwerveModulePosition?> = kotlin.arrayOfNulls<SwerveModulePosition>(4)
            for (j in 0..3) {
                wheelPositions[j] = modules[j].getOdometryPositions()[i]
            }
            RobotState
                .getInstance()
                .addOdometryObservation(
                    OdometryObservation(
                        wheelPositions,
                        Optional.ofNullable<T?>(
                            if (gyroInputs.data.connected()) gyroInputs.odometryYawPositions[i] else null,
                        ),
                        sampleTimestamps[i],
                    ),
                )

            // Log 3D robot pose
            Logger.recordOutput(
                "RobotState/EstimatedPose3d",
                Pose3d(RobotState.getInstance().getEstimatedPose())
                    .exp(
                        Twist3d(
                            0.0,
                            0.0,
                            Math.abs(gyroInputs.data.pitchPosition().getRadians()) *
                                DriveConstants.trackWidthX /
                                2.0,
                            0.0,
                            gyroInputs.data.pitchPosition().getRadians(),
                            0.0,
                        ),
                    ).exp(
                        Twist3d(
                            0.0,
                            0.0,
                            Math.abs(gyroInputs.data.rollPosition().getRadians()) *
                                DriveConstants.trackWidthY /
                                2.0,
                            gyroInputs.data.rollPosition().getRadians(),
                            0.0,
                            0.0,
                        ),
                    ),
            )
        }

        RobotState.getInstance().addDriveSpeeds(this.chassisSpeeds)
        RobotState.getInstance().setPitch(gyroInputs.data.pitchPosition())
        RobotState.getInstance().setRoll(gyroInputs.data.rollPosition())

        // Update brake mode
        // Reset movement timer if velocity above threshold
        if (Arrays
                .stream<Module?>(modules)
                .anyMatch { module: Module? -> Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecondThreshold.get() }
        ) {
            lastMovementTimer.reset()
        }

        if (DriverStation.isEnabled()) {
            coastRequest = CoastRequest.ALWAYS_BRAKE
        }

        when (coastRequest) {
            CoastRequest.AUTOMATIC -> {
                if (DriverStation.isEnabled()) {
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

        // Update current setpoint if not in velocity mode
        if (!velocityMode) {
            currentSetpoint = SwerveSetpoint(this.chassisSpeeds, this.moduleStates)
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(
            !gyroConnectedDebouncer.calculate(gyroInputs.data.connected()) && Constants.getMode() !== Mode.SIM && !Robot.isJITing(),
        )

        // Record cycle time
        LoggedTracer.record("Drive/Periodic")
    }

    /** Set brake mode to `enabled` doesn't change brake mode if already set.  */
    private fun setBrakeMode(enabled: Boolean) {
        if (brakeModeEnabled != enabled) {
            Arrays.stream<Module?>(modules).forEach { module: Module? -> module.setBrakeMode(enabled) }
        }
        brakeModeEnabled = enabled
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    fun runVelocity(speeds: ChassisSpeeds?) {
        velocityMode = true
        // Calculate module setpoints
        val discreteSpeeds: ChassisSpeeds? = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs)
        val setpointStatesUnoptimized: Array<SwerveModuleState?>? = kinematics.toSwerveModuleStates(discreteSpeeds)
        currentSetpoint =
            swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                Constants.loopPeriodSecs,
            )
        val setpointStates: Array<SwerveModuleState?> = currentSetpoint.moduleStates()

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized)
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates)
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds())

        // Send setpoints to modules
        for (i in 0..3) {
            modules[i].runSetpoint(setpointStates[i])
        }
    }

    /**
     * Runs the drive at the desired velocity with setpoint module forces.
     *
     * @param speeds Speeds in meters/sec
     * @param moduleForces The forces applied to each module
     */
    fun runVelocity(
        speeds: ChassisSpeeds?,
        moduleForces: MutableList<Vector<N2?>>,
    ) {
        velocityMode = true
        // Calculate module setpoints
        val discreteSpeeds: ChassisSpeeds? = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs)
        val setpointStatesUnoptimized: Array<SwerveModuleState?>? = kinematics.toSwerveModuleStates(discreteSpeeds)
        currentSetpoint =
            swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                Constants.loopPeriodSecs,
            )
        val setpointStates: Array<SwerveModuleState?> = currentSetpoint.moduleStates()

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized)
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates)
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds())

        // Save module forces to swerve states for logging
        val wheelForces: Array<SwerveModuleState?> = kotlin.arrayOfNulls<SwerveModuleState>(4)
        // Send setpoints to modules
        val moduleStates: Array<SwerveModuleState?> = this.moduleStates
        for (i in 0..3) {
            // Optimize state
            val wheelAngle: Rotation2d = moduleStates[i].angle
            setpointStates[i].optimize(wheelAngle)
            setpointStates[i].cosineScale(wheelAngle)

            // Calculate wheel torque in direction
            val wheelForce: Vector<N2?> = moduleForces.get(i)
            val wheelDirection: Vector<N2?>? = VecBuilder.fill(wheelAngle.getCos(), wheelAngle.getSin())
            val wheelTorqueNm: Double = wheelForce.dot(wheelDirection) * DriveConstants.wheelRadius
            modules[i].runSetpoint(setpointStates[i], wheelTorqueNm)

            // Save to array for logging
            wheelForces[i] = SwerveModuleState(wheelTorqueNm, setpointStates[i].angle)
        }
        Logger.recordOutput("Drive/SwerveStates/ModuleForces", wheelForces)
    }

    /** Runs the drive in a straight line with the specified drive output.  */
    fun runCharacterization(output: Double) {
        velocityMode = false
        for (i in 0..3) {
            modules[i].runCharacterization(output)
        }
    }

    /** Stops the drive.  */
    fun stop() {
        runVelocity(ChassisSpeeds())
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    fun stopWithX() {
        val headings: Array<Rotation2d?> = kotlin.arrayOfNulls<Rotation2d>(4)
        for (i in 0..3) {
            headings[i] = DriveConstants.moduleTranslations[i].getAngle()
        }
        kinematics.resetHeadings(headings)

        // Bypass swerve setpoint generator
        val states: Array<SwerveModuleState?> = kinematics.toSwerveModuleStates(ChassisSpeeds())
        for (i in 0..3) {
            states[i].optimize(modules[i].getAngle())
            modules[i].runSetpoint(states[i])
        }
    }

    @get:AutoLogOutput(key = "Drive/SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState>
        /** Returns the module states (turn angles and drive velocities) for all the modules.  */
        get() {
            val states: Array<SwerveModuleState?> = kotlin.arrayOfNulls<SwerveModuleState>(4)
            for (i in 0..3) {
                states[i] = modules[i].getState()
            }
            return states
        }

    @get:AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
    private val chassisSpeeds: ChassisSpeeds
        /** Returns the measured chassis speeds of the robot.  */
        get() = kinematics.toChassisSpeeds(this.moduleStates)

    val wheelRadiusCharacterizationPositions: DoubleArray
        /** Returns the position of each module in radians.  */
        get() {
            val values = DoubleArray(4)
            for (i in 0..3) {
                values[i] = modules[i].getWheelRadiusCharacterizationPosition()
            }
            return values
        }

    val fFCharacterizationVelocity: Double
        /** Returns the average velocity of the modules in rotations/sec (Phoenix native units).  */
        get() {
            var output = 0.0
            for (i in 0..3) {
                output += modules[i].getFFCharacterizationVelocity() / 4.0
            }
            return output
        }

    val gyroRotation: Rotation2d
        /** Returns the raw gyro rotation read by the IMU  */
        get() = gyroInputs.data.yawPosition()

    companion object {
        val odometryLock: Lock = ReentrantLock()
        private val coastWaitTime: LoggedTunableNumber = LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5)
        private val coastMetersPerSecondThreshold: LoggedTunableNumber =
            LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", .05)
    }
}
