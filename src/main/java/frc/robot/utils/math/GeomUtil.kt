package frc.robot.utils.math

import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Transform2d
import org.wpilib.math.geometry.Transform3d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.geometry.Twist2d
import org.wpilib.math.kinematics.ChassisVelocities

/** Geometry utilities for working with translations, rotations, transforms, and poses.  */
object GeomUtil {
    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    fun toTransform2d(translation: Translation2d?): Transform2d = Transform2d(translation, Rotation2d.ZERO)

    /**
     * Creates a pure translating transform
     *
     * @param x The x coordinate of the translation
     * @param y The y coordinate of the translation
     * @return The resulting transform
     */
    fun toTransform2d(
        x: Double,
        y: Double,
    ): Transform2d = Transform2d(x, y, Rotation2d.ZERO)

    /**
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    fun toTransform2d(rotation: Rotation2d?): Transform2d = Transform2d(Translation2d.ZERO, rotation)

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    fun toTransform2d(pose: Pose2d): Transform2d = Transform2d(pose.translation, pose.rotation)

    fun inverse(pose: Pose2d): Pose2d {
        val rotationInverse = pose.rotation.unaryMinus()
        return Pose2d(
            pose.translation.unaryMinus().rotateBy(rotationInverse),
            rotationInverse,
        )
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    fun toPose2d(transform: Transform2d): Pose2d = Pose2d(transform.translation, transform.rotation)

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    fun toPose2d(translation: Translation2d?): Pose2d = Pose2d(translation, Rotation2d.ZERO)

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    fun toPose2d(rotation: Rotation2d?): Pose2d = Pose2d(Translation2d.ZERO, rotation)

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    fun multiply(
        twist: Twist2d,
        factor: Double,
    ): Twist2d = Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor)

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    fun toTransform3d(pose: Pose3d): Transform3d = Transform3d(pose.translation, pose.rotation)

    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    fun toPose3d(transform: Transform3d): Pose3d = Pose3d(transform.translation, transform.rotation)

    /**
     * Converts a ChassisVelocities to a Twist2d by extracting two dimensions (Y and Z). chain
     *
     * @param speeds The original translation
     * @return The resulting translation
     */
    fun toTwist2d(speeds: ChassisVelocities): Twist2d =
        Twist2d(
            speeds.vx,
            speeds.vy,
            speeds.omega,
        )

    /**
     * Creates a new pose from an existing one using a different translation value.
     *
     * @param pose The original pose
     * @param translation The new translation to use
     * @return The new pose with the new translation and original rotation
     */
    fun withTranslation(
        pose: Pose2d,
        translation: Translation2d?,
    ): Pose2d = Pose2d(translation, pose.rotation)

    /**
     * Creates a new pose from an existing one using a different rotation value.
     *
     * @param pose The original pose
     * @param rotation The new rotation to use
     * @return The new pose with the original translation and new rotation
     */
    fun withRotation(
        pose: Pose2d,
        rotation: Rotation2d?,
    ): Pose2d = Pose2d(pose.translation, rotation)
}
