package frc.robot.subsystems.drive

import frc.robot.subsystems.drive.GyroIO.GyroIOInputs
import org.wpilib.hardware.imu.OnboardIMU
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation

class RealGyroIO : GyroIO {
    private val imu = OnboardIMU(MountOrientation.FLAT)

    override fun updateInputs(inputs: GyroIOInputs?) {
        inputs?.let {
            it.connected = true
            it.yawPosition = imu.rotation2d
            it.yawVelocityRadPerSec = imu.gyroRateZ
        }
    }
}
