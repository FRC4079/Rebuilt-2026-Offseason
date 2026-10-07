package frc.robot.mechanisms.drive

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.Pigeon2
import frc.robot.RobotState
import frc.robot.mechanisms.drive.module.RealModuleIO
import frc.robot.mechanisms.drive.module.SwerveModule
import frc.robot.utils.RobotParameters.CANBusParameters
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
import org.wpilib.util.Alert
import java.util.Optional

object Swerve : Mechanism {
    private val pidgey = Pigeon2(CANBusParameters.PIDGEY_ID, CANBus(CANBusParameters.SWERVE_CANBUS_ID))
    private val modules: Array<SwerveModule> = initializeModules()

    private val poseEstimator =
        SwerveDrivePoseEstimator(
            SwerveParameters.PhysicalParameters.kinematics,
            pidgeyRotation,
            modulePositions,
            Pose2d(),
        )

    private var previousFieldRelVelocities = ChassisVelocities()
    private var setStates = Array(modules.size) { SwerveModuleVelocity() }

    init {
        pidgey.reset()
        PhoenixOdometryThread.getInstance().start()
    }

    private fun initializeModules(): Array<SwerveModule> =
        Array(SwerveParameters.MODULE_CONFIGS.size) { index ->
            SwerveModule(
                RealModuleIO(SwerveParameters.MODULE_CONFIGS[index]),
                index,
            )
        }

    fun periodic() {
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
        get() = pidgey.rotation2d

    val heading: Double
        get() = -pidgey.yaw.valueAsDouble

    val pidgeyYaw: Double
        get() = pidgey.yaw.valueAsDouble

    fun resetPidgey() {
        pidgey.reset()
    }

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

    val rotationPidgey: Rotation2d
        get() = Rotation2d.fromDegrees(-pidgey.rotation2d.degrees)

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

    private const val LOOP_PERIOD_SECS = 0.02
}
