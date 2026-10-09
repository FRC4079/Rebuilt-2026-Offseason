package frc.robot.mechanisms.drive.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import frc.robot.utils.RobotParameters
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.util.Units
import org.wpilib.system.Timer
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity

/** IO implementation for Pigeon 2.  */
object RealGyroIO : GyroIO {
    private val pigeon =
        Pigeon2(RobotParameters.CANBusParameters.PIDGEY_ID, CANBus(RobotParameters.CANBusParameters.SWERVE_CANBUS_ID))
    private val yaw: StatusSignal<Angle?> = pigeon.yaw
    private val pitch: StatusSignal<Angle?> = pigeon.pitch
    private val roll: StatusSignal<Angle?> = pigeon.roll
    private val yawVelocity: StatusSignal<AngularVelocity?> = pigeon.angularVelocityZWorld
    private val pitchVelocity: StatusSignal<AngularVelocity?> = pigeon.angularVelocityXWorld
    private val rollVelocity: StatusSignal<AngularVelocity?> = pigeon.angularVelocityYWorld

    init {

        for (i in 0..4) {
            if (pigeon.configurator.apply(Pigeon2Configuration(), 0.25).isOK) break
        }

        for (i in 0..4) {
            if (pigeon.configurator.apply(Pigeon2Configuration(), 0.25).isOK) break
        }

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            yaw,
            pitch,
            roll,
            yawVelocity,
            pitchVelocity,
            rollVelocity,
        )

        pigeon.optimizeBusUtilization()
        pigeon.setYaw(0.0, 0.25)
    }

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        BaseStatusSignal.refreshAll(yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity)

        val yawPosition = Rotation2d.fromDegrees(yaw.valueAsDouble)

        inputs.data =
            GyroIO.GyroIOData(
                BaseStatusSignal.isAllGood(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity),
                yawPosition,
                Units.degreesToRadians(yawVelocity.valueAsDouble),
                Rotation2d.fromDegrees(pitch.valueAsDouble),
                Units.degreesToRadians(pitchVelocity.valueAsDouble),
                Rotation2d.fromDegrees(roll.valueAsDouble),
                Units.degreesToRadians(rollVelocity.valueAsDouble),
            )

        inputs.odometryYawTimestamps = doubleArrayOf(Timer.getTimestamp())
        inputs.odometryYawPositions = arrayOf(yawPosition)
    }

    override fun reset() {
        pigeon.reset()
    }
}
