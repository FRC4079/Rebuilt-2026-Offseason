package frc.robot.subsystems.drive

import frc.robot.utils.RobotParameters.SwerveParameters
import org.littletonrobotics.junction.AutoLogOutput
import org.wpilib.command3.Mechanism
import org.wpilib.driverstation.DriverStation
import org.wpilib.math.estimator.SwerveDrivePoseEstimator
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.SwerveDriveKinematics
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.numbers.N1
import org.wpilib.math.numbers.N3
import org.wpilib.util.Alert
import java.lang.Module
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.collections.get

class Drive(
    private val gyroIO: GyroIO,
    flModuleIO: ModuleIO?,
    frModuleIO: ModuleIO?,
    blModuleIO: ModuleIO?,
    brModuleIO: ModuleIO?,
) : Mechanism {
    private val gyroInputs: GyroIOInputsAutoLogged = GyroIOInputsAutoLogged()
    private val modules: Array<Module> = arrayOfNulls<Module>(4) as Array<Module> // FL, FR, BL, BR
    private val gyroDisconnectedAlert: Alert =
        Alert(
            "Swerve Alerts",
            "Disconnected gyro on drive, switching to kinematics.",
            Alert.Level.MEDIUM,
        )

    private val kinematics = SwerveParameters.PhysicalParameters.kinematics
    private var rawGyroRotation = Rotation2d()
    private val lastModulePositions: Array<SwerveModulePosition?> = // For delta tracking
        arrayOf<SwerveModulePosition>(
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
        )
    private val poseEstimator = SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d())

    init {
        modules[0] = Module(flModuleIO, 0)
        modules[1] = Module(frModuleIO, 1)
        modules[2] = Module(blModuleIO, 2)
        modules[3] = Module(brModuleIO, 3)
    }

    public override fun periodic() {
        odometryLock.lock() // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)
        for (module in modules) {
            module.periodic()
        }
        odometryLock.unlock()

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", arrayOf<SwerveModuleState?>())
            Logger.recordOutput("SwerveStates/SetpointsOptimized", arrayOf<SwerveModuleState?>())
        }

        // Calculate odometry
        // Read wheel positions and deltas from each module
        val modulePositions = arrayOfNulls<SwerveModulePosition>(4)
        val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)
        for (moduleIndex in 0..3) {
            modulePositions[moduleIndex] = modules[moduleIndex].getPosition()
            moduleDeltas[moduleIndex] =
                SwerveModulePosition(
                    modulePositions[moduleIndex]!!.distance - lastModulePositions[moduleIndex]!!.distance,
                    modulePositions[moduleIndex]!!.angle,
                )
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
        }
        if (gyroInputs.connected) {
            // Use the real gyro angle
            rawGyroRotation = gyroInputs.yawPosition
        } else {
            // Use the angle delta from the kinematics and module deltas
            val twist = kinematics.toTwist2d(*moduleDeltas)
            rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
        }
        poseEstimator.updateWithTime(Timer.getTimestamp(), rawGyroRotation, modulePositions)

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() !== Mode.SIM)
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    fun runVelocity(speeds: ChassisSpeeds) {
        // Calculate module setpoints
        val discreteSpeeds: ChassisSpeeds? = speeds.discretize(Constants.loopPeriodSecs)
        val setpointStates: Array<SwerveModuleState?> = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.maxLinearSpeed)

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates)
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds)

        // Send setpoints to modules
        for (i in 0..3) {
            modules[i].runSetpoint(setpointStates[i])
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates)
    }

    /** Runs the drive in a straight line with the specified drive output.  */
    fun runCharacterization(output: Double) {
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
        val headings = arrayOfNulls<Rotation2d>(4)
        for (i in 0..3) {
            headings[i] = DriveConstants.moduleTranslations[i].getAngle()
        }
        kinematics.resetHeadings(*headings)
        stop()
    }

    @get:AutoLogOutput(key = "SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState>
        /** Returns the module states (turn angles and drive velocities) for all of the modules.  */
        get() {
            val states: Array<SwerveModuleState?> = kotlin.arrayOfNulls<SwerveModuleState>(4)
            for (i in 0..3) {
                states[i] = modules[i].getState()
            }
            return states
        }

    private val modulePositions: Array<SwerveModulePosition?>
        /** Returns the module positions (turn angles and drive positions) for all of the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModulePosition>(4)
            for (i in 0..3) {
                states[i] = modules[i].getPosition()
            }
            return states
        }

    @get:AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
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

    @get:AutoLogOutput(key = "Odometry/Robot")
    var pose: Pose2d?
        /** Returns the current odometry pose.  */
        get() = poseEstimator.getEstimatedPosition()
        /** Resets the current odometry pose.  */
        set(pose) {
            poseEstimator.resetPosition(rawGyroRotation, this.modulePositions, pose)
        }

    val rotation: Rotation2d?
        /** Returns the current odometry rotation.  */
        get() = this.pose!!.getRotation()

    /** Adds a new timestamped vision measurement.  */
    fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d?,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3?, N1?>?,
    ) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters,
            timestampSeconds,
            visionMeasurementStdDevs,
        )
    }

    val maxLinearSpeedMetersPerSec: Double
        /** Returns the maximum linear speed in meters per sec.  */
        get() = DriveConstants.maxLinearSpeed

    val maxAngularSpeedRadPerSec: Double
        /** Returns the maximum angular speed in radians per sec.  */
        get() = this.maxLinearSpeedMetersPerSec / DriveConstants.driveBaseRadius

    companion object {
        val odometryLock: Lock = ReentrantLock()
    }
}
