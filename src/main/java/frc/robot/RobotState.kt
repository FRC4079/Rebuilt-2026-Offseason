package frc.robot

import frc.robot.utils.RobotParameters
import frc.robot.utils.math.GeomUtil
import org.littletonrobotics.junction.AutoLogOutput
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform2d
import org.wpilib.math.geometry.Twist2d
import org.wpilib.math.interpolation.TimeInterpolatableBuffer
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.linalg.Matrix
import org.wpilib.math.numbers.N1
import org.wpilib.math.numbers.N3
import java.util.Optional

class RobotState private constructor() {
    @get:AutoLogOutput
    var odometryPose: Pose2d = Pose2d.ZERO
        private set

    @get:AutoLogOutput
    var estimatedPose: Pose2d = Pose2d.ZERO
        private set

    private val poseBuffer = TimeInterpolatableBuffer.createBuffer<Pose2d>(POSE_BUFFER_SIZE_SEC)
    private val slamBuffer = TimeInterpolatableBuffer.createBuffer<Rotation2d>(SLAM_BUFFER_SIZE_SEC)
    private val rotationBuffer = TimeInterpolatableBuffer.createBuffer<Rotation3d>(POSE_BUFFER_SIZE_SEC)

    private val kinematics = RobotParameters.SwerveParameters.PhysicalParameters.kinematics
    private var lastWheelPositions =
        Array(RobotParameters.SwerveParameters.PhysicalParameters.MODULE_LOCATIONS.size) { SwerveModulePosition() }
    private var lastTheta: Rotation2d? = null

    var robotVelocity: ChassisVelocities = ChassisVelocities()
        private set
    var robotSetpointVelocity: ChassisVelocities = ChassisVelocities()
        private set

    private var pitch: Rotation2d = Rotation2d.ZERO
    private var roll: Rotation2d = Rotation2d.ZERO

    val rotation: Rotation2d
        get() = estimatedPose.rotation

    val fieldVelocity: ChassisVelocities
        get() = robotVelocity.toFieldRelative(rotation)

    val fieldSetpointVelocity: ChassisVelocities
        get() = robotSetpointVelocity.toFieldRelative(rotation)

    @AutoLogOutput
    fun getSlamAngle(timestamp: Double): Optional<Rotation2d> = slamBuffer.getSample(timestamp)

    val intakePose: Pose2d
        get() = estimatedPose.plus(GeomUtil.toTransform2d(INTAKE_REFERENCE_X, 0.0))

    val intakeFieldVelocity: ChassisVelocities
        get() {
            val field = fieldVelocity
            return ChassisVelocities(
                field.vx - (INTAKE_REFERENCE_X * rotation.sin * field.omega),
                field.vy + (INTAKE_REFERENCE_X * rotation.cos * field.omega),
                field.omega,
            )
        }

    fun addDriveSpeeds(speeds: ChassisVelocities) {
        robotVelocity = speeds
    }

    fun addDriveSetpointSpeeds(speeds: ChassisVelocities) {
        robotSetpointVelocity = speeds
    }

    fun setPitch(pitch: Rotation2d) {
        this.pitch = pitch
    }

    fun setRoll(roll: Rotation2d) {
        this.roll = roll
    }

    fun resetPose(pose: Pose2d) {
        odometryPose = pose
        estimatedPose = pose
        poseBuffer.clear()
    }

    fun addOdometryObservation(observation: OdometryObservation) {
        val twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions)
        lastWheelPositions = observation.wheelPositions.copyOf()

        var dtheta = twist.dtheta
        if (observation.yaw.isPresent && lastTheta != null) {
            dtheta = observation.yaw.get().minus(lastTheta).radians
        }
        if (observation.yaw.isPresent) {
            lastTheta = observation.yaw.get()
        }

        val appliedTwist = Twist2d(twist.dx, twist.dy, dtheta)
        odometryPose = odometryPose.plus(appliedTwist.exp())
        estimatedPose = estimatedPose.plus(appliedTwist.exp())

        poseBuffer.addSample(observation.timestamp, odometryPose)

        if (observation.yaw.isPresent) {
            rotationBuffer.addSample(
                observation.timestamp,
                Rotation3d(roll.radians, pitch.radians, observation.yaw.get().radians),
            )
        }
    }

    fun addSlamObservation(observation: SlamObservation) {
        slamBuffer.addSample(observation.timestamp, observation.slamAngle)
    }

    fun addVisionObservation(observation: VisionObservation) {
        estimatedPose = observation.visionPose.toPose2d()
    }

    fun getEstimatedPoseAtTimestamp(timestamp: Double): Optional<Pose2d> {
        val oldOdometryPose = poseBuffer.getSample(timestamp)
        if (oldOdometryPose.isEmpty) {
            return Optional.empty()
        }
        return Optional.of(
            estimatedPose.transformBy(
                Transform2d(
                    odometryPose,
                    oldOdometryPose.get(),
                ),
            ),
        )
    }

    fun getEstimatedRotation3dAtTimestamp(timestamp: Double): Optional<Rotation3d> = rotationBuffer.getSample(timestamp)

    val lastEstimatedRotation3d: Optional<Rotation3d>
        get() = Optional.ofNullable(rotationBuffer.internalBuffer.lastEntry()?.value)

    data class OdometryObservation(
        val timestamp: Double,
        val wheelPositions: Array<SwerveModulePosition>,
        val roll: Optional<Rotation2d> = Optional.empty(),
        val pitch: Optional<Rotation2d> = Optional.empty(),
        val yaw: Optional<Rotation2d> = Optional.empty(),
    )

    class VisionObservation(
        val timestamp: Double,
        val visionPose: Pose3d,
        val stdDevs: Matrix<N3, N1>,
    )

    data class SlamObservation(
        val timestamp: Double,
        val slamAngle: Rotation2d,
    )

    companion object {
        private const val POSE_BUFFER_SIZE_SEC = 2.0
        private const val SLAM_BUFFER_SIZE_SEC = 2.0
        private const val INTAKE_REFERENCE_X = 0.0

        private var instance: RobotState? = null

        @JvmStatic
        fun getInstance(): RobotState {
            if (instance == null) {
                instance = RobotState()
            }
            return instance!!
        }
    }
}
