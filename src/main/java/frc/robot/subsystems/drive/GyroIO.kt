package frc.robot.subsystems.drive

import org.littletonrobotics.junction.AutoLog
import org.wpilib.math.geometry.Rotation2d

fun interface GyroIO {
    @AutoLog
    open class GyroIOInputs {
        @JvmField
        var connected: Boolean = false

        @JvmField
        var yawPosition: Rotation2d = Rotation2d()

        @JvmField
        var yawVelocityRadPerSec: Double = 0.0
    }

    fun updateInputs(inputs: GyroIOInputs?)
}
