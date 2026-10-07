package frc.robot.mechanisms.drive.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import frc.robot.utils.RobotParameters
import frc.robot.utils.phoenix.PhoenixOdometryThread
import frc.robot.utils.phoenix.PhoenixUtils
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import java.util.Queue

/** IO implementation for Pigeon 2.  */
object RealGyroIO : GyroIO {
    private val pigeon =
        Pigeon2(RobotParameters.CANBusParameters.PIDGEY_ID, CANBus(RobotParameters.CANBusParameters.SWERVE_CANBUS_ID))
    private val yaw: StatusSignal<Angle?> = pigeon.yaw
    private val pitch: StatusSignal<Angle?> = pigeon.pitch
    private val roll: StatusSignal<Angle?> = pigeon.roll
    private val yawPositionQueue: Queue<Double?>
    private val yawTimestampQueue: Queue<Double?>
    private val yawVelocity: StatusSignal<AngularVelocity?> = pigeon.angularVelocityZWorld
    private val pitchVelocity: StatusSignal<AngularVelocity?> = pigeon.angularVelocityXWorld
    private val rollVelocity: StatusSignal<AngularVelocity?> = pigeon.angularVelocityYWorld

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.configurator.setYaw(0.0)
        yaw.setUpdateFrequency(RobotParameters.SwerveParameters.OdometryConfig.ODOMETRY_FREQUENCY)
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pitch,
            roll,
            yawVelocity,
            pitchVelocity,
            rollVelocity,
        )
        pigeon.optimizeBusUtilization()
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue()
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.yaw)
        PhoenixUtils.registerSignals(true, yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity)
        PhoenixUtils.tryUntilOk(5, { pigeon.setYaw(0.0, 0.25) })
    }

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.data =
            GyroIO.GyroIOData(
                BaseStatusSignal.isAllGood(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity),
                Rotation2d.fromDegrees(yaw.valueAsDouble),
                Units.degreesToRadians(yawVelocity.valueAsDouble),
                Rotation2d.fromDegrees(pitch.valueAsDouble),
                Units.degreesToRadians(pitchVelocity.valueAsDouble),
                Rotation2d.fromDegrees(roll.valueAsDouble),
                Units.degreesToRadians(rollVelocity.valueAsDouble),
            )

        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble { value: Double? -> value!! }.toArray()
        inputs.odometryYawPositions =
            yawPositionQueue
                .map { degrees: Double? -> Rotation2d.fromDegrees(degrees!!) }
                .toTypedArray()
        yawTimestampQueue.clear()
        yawPositionQueue.clear()
    }
}
