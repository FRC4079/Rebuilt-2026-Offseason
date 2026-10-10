package frc.robot.mechanisms

import com.limelightvision.Limelight
import com.limelightvision.PoseEstimateType
import frc.robot.mechanisms.drive.Swerve
import frc.robot.utils.PoseLookup
import frc.robot.utils.RobotParameters
import org.wpilib.command3.Mechanism
import org.wpilib.math.estimator.SwerveDrivePoseEstimator
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.math.util.Units
import kotlin.math.atan2

class OdometrySupplier(
    private val swerve: Swerve,
    private val vision: Vision,
) : Mechanism {
    companion object {
        private const val MAX_VISION_POSE_DIFFERENCE_METERS = 1.0
    }

    private val poseEstimator: SwerveDrivePoseEstimator =
        SwerveDrivePoseEstimator(
            RobotParameters.SwerveParameters.PhysicalParameters.kinematics,
            swerve.gyroYaw,
            swerve.modulePositions,
            Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5.0)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30.0)),
        )

    val pose: Pose2d
        get() = poseEstimator.estimatedPosition

    fun periodic() {
        val yaw = swerve.gyroYaw
        val modulePositions = swerve.modulePositions

        Limelight.setSharedRobotOrientation(yaw.degrees)

        for (camera in vision.cameras) {
            for (estimate in camera.readAcceptedPoseEstimates(PoseEstimateType.MT2_WPIBLUE)) {
                if (isVisionPoseCloseToEstimate(estimate.pose)) {
                    poseEstimator.addVisionMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds,
                        estimate.stdDevs,
                    )
                }
            }
        }

        poseEstimator.update(yaw, modulePositions)
    }

    fun resetPose(pose: Pose2d = Pose2d()) {
        poseEstimator.resetPosition(
            swerve.gyroYaw,
            swerve.modulePositions,
            pose,
        )
    }

    fun isVisionPoseCloseToEstimate(pose: Pose2d): Boolean =
        this.pose.translation.getDistance(pose.translation) <= MAX_VISION_POSE_DIFFERENCE_METERS

    fun getDistanceToPoseMeters(targetPose: Pose2d): Double = pose.translation.getDistance(targetPose.translation)

    /**
     * Distance from the robot's current pose to the hub.
     *
     * @return the distance in meters
     */
    fun getDistanceToHub(): Double = getDistanceToPoseMeters(PoseLookup.hubPose)

    /**
     * Field relative heading from the robot's current pose to the hub that the robot needs to point
     * at in order to shoot.
     *
     * @return the heading the robot should face
     */
    fun getAngleToHub(): Rotation2d {
        val robot = pose.translation
        val hub = PoseLookup.hubPose.translation
        return Rotation2d.fromRadians(atan2(hub.y - robot.y, hub.x - robot.x))
    }
}
