package frc.robot.utils

import org.wpilib.driverstation.Alliance
import org.wpilib.driverstation.MatchState
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d

object PoseLookup {
    // TODO: fill in the real hub poses once they are known
    private val RED_HUB_POSE: Pose2d = Pose2d(0.0, 0.0, Rotation2d())
    private val BLUE_HUB_POSE: Pose2d = Pose2d(0.0, 0.0, Rotation2d())

    val ally: Alliance
        get() = MatchState.getAlliance().orElse(Alliance.BLUE) ?: Alliance.BLUE

    val hubPose: Pose2d
        get() =
            if (ally == Alliance.RED) {
                RED_HUB_POSE
            } else {
                BLUE_HUB_POSE
            }
}
