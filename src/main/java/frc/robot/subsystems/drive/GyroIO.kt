package frc.robot.subsystems.drive

import org.littletonrobotics.junction.AutoLog
import org.wpilib.math.geometry.Rotation2d

fun interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        var connected: Boolean = false
        var yawPosition: Rotation2d = Rotation2d()
        var yawVelocityRadPerSec: Double = 0.0
    }

    fun updateInputs(inputs: GyroIOInputs?)
}
