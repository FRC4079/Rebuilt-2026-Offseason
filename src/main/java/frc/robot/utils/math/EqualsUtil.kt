package frc.robot.utils.math

import org.wpilib.math.geometry.Twist2d

object EqualsUtil {
    @JvmOverloads
    fun epsilonEquals(
        a: Double,
        b: Double,
        epsilon: Double = 1e-9,
    ): Boolean = (a - epsilon <= b) && (a + epsilon >= b)

    /** Extension methods for wpi geometry objects  */
    object GeomExtensions {
        fun Twist2d.epsilonEquals(twist: Twist2d): Boolean =
            epsilonEquals(this.dx, twist.dx) &&
                epsilonEquals(this.dy, twist.dy) &&
                epsilonEquals(this.dtheta, twist.dtheta)
    }
}
