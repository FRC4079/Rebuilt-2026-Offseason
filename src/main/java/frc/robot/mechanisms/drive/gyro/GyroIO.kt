package frc.robot.mechanisms.drive.gyro

import frc.robot.utils.logging.ReflectiveLoggableInputs
import org.littletonrobotics.junction.AutoLog
import org.wpilib.math.geometry.Rotation2d

interface GyroIO {
    @AutoLog
    open class GyroIOInputs {
        @JvmField
        var data: GyroIOData =
            GyroIOData(
                connected = false,
                yawPosition = Rotation2d.ZERO,
                yawVelocityRadPerSec = 0.0,
                pitchPosition = Rotation2d.ZERO,
                pitchVelocityRadPerSec = 0.0,
                rollPosition = Rotation2d.ZERO,
                rollVelocityRadPerSec = 0.0,
            )

        @JvmField
        var odometryYawTimestamps: DoubleArray = doubleArrayOf()

        @JvmField
        var odometryYawPositions: Array<Rotation2d> = arrayOf()
    }

    data class GyroIOData(
        val connected: Boolean,
        val yawPosition: Rotation2d,
        val yawVelocityRadPerSec: Double,
        val pitchPosition: Rotation2d,
        val pitchVelocityRadPerSec: Double,
        val rollPosition: Rotation2d,
        val rollVelocityRadPerSec: Double,
    ) : ReflectiveLoggableInputs()

    fun updateInputs(inputs: GyroIOInputs) {}
}
