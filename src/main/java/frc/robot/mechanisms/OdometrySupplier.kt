package frc.robot.mechanisms

import com.limelightvision.Limelight
import com.limelightvision.PoseEstimateType
import frc.robot.mechanisms.drive.Swerve
import frc.robot.utils.RobotParameters
import org.wpilib.command3.Mechanism
import org.wpilib.math.estimator.SwerveDrivePoseEstimator
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.math.util.Units

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

    private fun isVisionPoseCloseToEstimate(visionPose: Pose2d): Boolean {
        val distanceMeters = getDistanceToPoseMeters(visionPose)
        return distanceMeters <= MAX_VISION_POSE_DIFFERENCE_METERS
    }

    fun getDistanceToPoseMeters(targetPose: Pose2d): Double = pose.translation.getDistance(targetPose.translation)
}
