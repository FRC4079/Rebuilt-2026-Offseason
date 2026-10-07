package frc.robot.mechanisms.drive

import frc.robot.RobotState
import frc.robot.mechanisms.drive.gyro.GyroIO
import frc.robot.mechanisms.drive.gyro.GyroIOInputsAutoLogged
import frc.robot.mechanisms.drive.module.ModuleIO
import frc.robot.mechanisms.drive.module.SwerveModule
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.phoenix.PhoenixOdometryThread
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.math.estimator.SwerveDrivePoseEstimator
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.system.Timer
import java.util.Optional

class Swerve(
    private val moduleIOs: Array<ModuleIO>,
    private val gyroIO: GyroIO,
) : Mechanism {
    private val gyroInputs = GyroIOInputsAutoLogged()
    private val modules: Array<SwerveModule> = initializeModules()

    private val poseEstimator =
        SwerveDrivePoseEstimator(
            SwerveParameters.PhysicalParameters.kinematics,
            Rotation2d.ZERO,
            modulePositions,
            Pose2d(),
        )

    private var previousFieldRelVelocities = ChassisVelocities()
    private var setStates = Array(modules.size) { SwerveModuleVelocity() }

    init {
        gyroIO.reset()
        PhoenixOdometryThread.getInstance().start()
    }

    private fun initializeModules(): Array<SwerveModule> =
        Array(SwerveParameters.MODULE_CONFIGS.size) { index ->
            SwerveModule(
                moduleIOs[index],
                index,
            )
        }

    fun periodic() {
        gyroIO.updateInputs(gyroInputs)
        for (module in modules) {
            module.updateInputs()
        }

        for (module in modules) {
            module.periodic()
        }

        poseEstimator.update(pidgeyRotation, modulePositions)

        RobotState.getInstance().addDriveSpeeds(autoSpeeds)
        Logger.recordOutput("Swerve/Pose", poseEstimator.estimatedPosition)
        Logger.recordOutput("Swerve/States/Measured", SwerveModuleVelocity.struct, *moduleStates)
        Logger.recordOutput("Swerve/States/Setpoints", SwerveModuleVelocity.struct, *setStates)

        previousFieldRelVelocities = fieldRelativeVelocity
    }

    fun setDriveSpeeds(
        forwardSpeed: Double,
        leftSpeed: Double,
        turnSpeed: Double,
        isFieldOriented: Boolean = SwerveParameters.Thresholds.IS_FIELD_ORIENTED,
    ) {
        val speeds =
            if (isFieldOriented) {
                ChassisVelocities(forwardSpeed, leftSpeed, turnSpeed).toRobotRelative(pidgeyRotation)
            } else {
                ChassisVelocities(forwardSpeed, leftSpeed, turnSpeed)
            }.discretize(LOOP_PERIOD_SECS)

        runVelocity(speeds)
    }

    fun runVelocity(speeds: ChassisVelocities) {
        val states = SwerveParameters.PhysicalParameters.kinematics.toSwerveModuleVelocities(speeds)
        setStates = states

        for (i in modules.indices) {
            modules[i].state = states[i]
        }
    }

    val fieldRelativeVelocity: ChassisVelocities
        get() =
            autoSpeeds.toFieldRelative(pidgeyRotation)
    val pidgeyRotation: Rotation2d
        get() = gyroInputs.data.yawPosition

    val pose: Pose2d
        get() = poseEstimator.estimatedPosition

    fun zeroPose() {
        poseEstimator.resetPosition(
            pidgeyRotation,
            modulePositions,
            Pose2d(),
        )
    }

    fun newPose(pose: Pose2d?) {
        if (pose == null) return

        poseEstimator.resetPosition(
            pidgeyRotation,
            modulePositions,
            pose,
        )
    }

    val autoSpeeds: ChassisVelocities
        get() = SwerveParameters.PhysicalParameters.kinematics.toChassisVelocities(*moduleStates)

    fun chassisSpeedsDrive(chassisSpeeds: ChassisVelocities?) {
        if (chassisSpeeds == null) return
        runVelocity(chassisSpeeds)
    }

    var moduleStates: Array<SwerveModuleVelocity>
        get() = Array(modules.size) { index -> modules[index].state }
        set(states) {
            setStates = states
            for (i in states.indices) {
                modules[i].state = states[i]
            }
        }

    val modulePositions: Array<SwerveModulePosition>
        get() = Array(modules.size) { index -> modules[index].position }

    fun addOdometryObservation() {
        RobotState.getInstance().addOdometryObservation(
            RobotState.OdometryObservation(
                timestamp = Timer.getTimestamp(),
                wheelPositions = modulePositions,
                yaw = Optional.of(pidgeyRotation),
            ),
        )
    }

    fun stop() {
        for (module in modules) {
            module.stop()
        }
    }

    fun setAutoPID() {
        for (module in modules) {
            module.setAutoPID()
        }
    }

    fun setTelePID() {
        for (module in modules) {
            module.setTelePID()
            module.applyTelePIDValues()
        }
    }

    fun resetDrive() {
        for (module in modules) {
            module.resetDrivePosition()
        }
    }

    fun updateModuleTelePIDValues() {
        for (module in modules) {
            module.updateTelePID()
        }
    }

    fun setBrakeMode(enabled: Boolean) {
        for (module in modules) {
            module.setBrakeMode(enabled)
        }
    }

    fun pathFindToGoal(): Command? = null

    fun pathFindTest(): Command? = null

    private val LOOP_PERIOD_SECS = 0.02
}
