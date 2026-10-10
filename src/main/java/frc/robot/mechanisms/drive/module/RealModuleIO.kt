package frc.robot.mechanisms.drive.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.robot.utils.RobotParameters.SwerveParameters
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Current
import org.wpilib.units.measure.Voltage
import java.util.concurrent.Executors

class RealModuleIO(
    config: SwerveParameters.ModuleConfig,
) : ModuleIO {
    private val brakeModeExecutor = Executors.newFixedThreadPool(8)
    private val driveTalon = TalonFX(config.driveMotorId, CANBus(config.canBUS))
    private val turnTalon = TalonFX(config.turnMotorId, CANBus(config.canBUS))
    private val encoder = CANcoder(config.encoderID, CANBus(config.canBUS))
    private val encoderOffset = config.encoderOffset
    private val driveConfig = TalonFXConfiguration()
    private val turnConfig = TalonFXConfiguration()
    private val positionVoltageRequest = PositionVoltage(0.0).withSlot(0)
    private val velocityVoltageRequest = VelocityVoltage(0.0).withSlot(0)
    private val drivePosition: StatusSignal<Angle>
    private val driveVelocity: StatusSignal<AngularVelocity>
    private val driveAppliedVolts: StatusSignal<Voltage>
    private val driveSupplyCurrentAmps: StatusSignal<Current>
    private val driveTorqueCurrentAmps: StatusSignal<Current>

    private val turnAbsolutePosition: StatusSignal<Angle>
    private val turnPosition: StatusSignal<Angle>
    private val turnVelocity: StatusSignal<AngularVelocity>
    private val turnAppliedVolts: StatusSignal<Voltage>
    private val turnSupplyCurrentAmps: StatusSignal<Current>
    private val turnTorqueCurrentAmps: StatusSignal<Current>

    init {
        // Configure drive motor
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        driveConfig.Slot0 =
            Slot0Configs()
                .withKP(SwerveParameters.PIDParameters.DRIVE_PID_TELE.p)
                .withKI(SwerveParameters.PIDParameters.DRIVE_PID_TELE.i)
                .withKD(SwerveParameters.PIDParameters.DRIVE_PID_TELE.d)

        driveConfig.Feedback.SensorToMechanismRatio = SwerveParameters.PhysicalParameters.DRIVE_MOTOR_GEAR_RATIO
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        driveConfig.CurrentLimits.StatorCurrentLimit = SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        for (i in 0..4) {
            if (driveTalon.configurator.apply(driveConfig, 0.25).isOK) break
        }
        for (i in 0..4) {
            if (driveTalon.setPosition(0.0, 0.25).isOK) break
        }

        // Configure turn motor
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        turnConfig.Slot0 =
            Slot0Configs()
                .withKP(SwerveParameters.PIDParameters.STEER_PID_TELE.p)
                .withKI(SwerveParameters.PIDParameters.STEER_PID_TELE.i)
                .withKD(SwerveParameters.PIDParameters.STEER_PID_TELE.d)

        turnConfig.Feedback.FeedbackRemoteSensorID = config.encoderID
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
        turnConfig.Feedback.RotorToSensorRatio = SwerveParameters.PhysicalParameters.STEER_MOTOR_GEAR_RATIO
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = SwerveParameters.PhysicalParameters.STEER_CURRENT_LIMIT
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveParameters.PhysicalParameters.STEER_CURRENT_LIMIT
        turnConfig.CurrentLimits.StatorCurrentLimit = SwerveParameters.PhysicalParameters.STEER_CURRENT_LIMIT
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true
        turnConfig.MotorOutput.Inverted =
            if (config.turnInverted) {
                InvertedValue.Clockwise_Positive
            } else {
                InvertedValue.CounterClockwise_Positive
            }

        for (i in 0..4) {
            if (turnTalon.configurator.apply(turnConfig, 0.25).isOK) break
        }

        // Configure CANCoder
        val cancoderConfig = CANcoderConfiguration()
        cancoderConfig.MagnetSensor.MagnetOffset = config.encoderOffset.rotations
        cancoderConfig.MagnetSensor.SensorDirection =
            if (config.encoderInverted) {
                SensorDirectionValue.Clockwise_Positive
            } else {
                SensorDirectionValue.CounterClockwise_Positive
            }

        for (i in 0..4) {
            if (encoder.configurator.apply(cancoderConfig, 0.25).isOK) break
        }

        // Create drive status signals
        drivePosition = driveTalon.position
        driveVelocity = driveTalon.velocity
        driveAppliedVolts = driveTalon.motorVoltage
        driveSupplyCurrentAmps = driveTalon.supplyCurrent
        driveTorqueCurrentAmps = driveTalon.torqueCurrent

        // Create turn status signals
        turnAbsolutePosition = encoder.absolutePosition
        turnPosition = turnTalon.position
        turnVelocity = turnTalon.velocity
        turnAppliedVolts = turnTalon.motorVoltage
        turnSupplyCurrentAmps = turnTalon.supplyCurrent
        turnTorqueCurrentAmps = turnTalon.torqueCurrent

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrentAmps,
            driveTorqueCurrentAmps,
            turnAbsolutePosition,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrentAmps,
            turnTorqueCurrentAmps,
        )

        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, encoder)
    }

    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrentAmps,
            driveTorqueCurrentAmps,
            turnAbsolutePosition,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrentAmps,
            turnTorqueCurrentAmps,
        )

        inputs.data.driveConnected =
            BaseStatusSignal.isAllGood(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrentAmps,
                driveTorqueCurrentAmps,
            )
        inputs.data.drivePositionRad = Units.rotationsToRadians(drivePosition.valueAsDouble)
        inputs.data.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.valueAsDouble)
        inputs.data.driveAppliedVolts = driveAppliedVolts.valueAsDouble
        inputs.data.driveSupplyCurrentAmps = driveSupplyCurrentAmps.valueAsDouble
        inputs.data.driveTorqueCurrentAmps = driveTorqueCurrentAmps.valueAsDouble

        inputs.data.turnConnected =
            BaseStatusSignal.isAllGood(
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrentAmps,
                turnTorqueCurrentAmps,
            )
        inputs.data.turnEncoderConnected = BaseStatusSignal.isAllGood(turnAbsolutePosition)
        inputs.data.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.valueAsDouble).minus(encoderOffset)
        inputs.data.turnPosition = Rotation2d.fromRotations(turnPosition.valueAsDouble)
        inputs.data.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.valueAsDouble)
        inputs.data.turnAppliedVolts = turnAppliedVolts.valueAsDouble
        inputs.data.turnSupplyCurrentAmps = turnSupplyCurrentAmps.valueAsDouble
        inputs.data.turnTorqueCurrentAmps = turnTorqueCurrentAmps.valueAsDouble

        inputs.odometryDrivePositionsRad = doubleArrayOf(inputs.data.drivePositionRad)
        inputs.odometryTurnPositions = arrayOf(inputs.data.turnPosition)
    }

    override fun runDriveOpenLoop(output: Double) {
        driveTalon.setVoltage(output)
    }

    override fun runTurnOpenLoop(output: Double) {
        turnTalon.setVoltage(output)
    }

    override fun runDriveVelocity(
        velocityRadPerSec: Double,
        feedforward: Double,
    ) {
        driveTalon.setControl(
            velocityVoltageRequest
                .withVelocity(Units.radiansToRotations(velocityRadPerSec))
                .withFeedForward(feedforward),
        )
    }

    override fun runTurnPosition(rotation: Rotation2d) {
        turnTalon.setControl(positionVoltageRequest.withPosition(rotation.rotations))
    }

    override fun setDrivePID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        driveConfig.Slot0.kP = kP
        driveConfig.Slot0.kI = kI
        driveConfig.Slot0.kD = kD
        for (i in 0..4) {
            if (driveTalon.configurator.apply(driveConfig, 0.25).isOK) break
        }
    }

    override fun setTurnPID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        turnConfig.Slot0.kP = kP
        turnConfig.Slot0.kI = kI
        turnConfig.Slot0.kD = kD
        for (i in 0..4) {
            if (turnTalon.configurator.apply(turnConfig, 0.25).isOK) break
        }
    }

    override fun setBrakeMode(enabled: Boolean) {
        brakeModeExecutor.execute {
            synchronized(driveConfig) {
                driveConfig.MotorOutput.NeutralMode = if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast
                for (i in 0..4) {
                    if (driveTalon.configurator.apply(driveConfig, 0.25).isOK) break
                }
            }
        }
        brakeModeExecutor.execute {
            synchronized(turnConfig) {
                turnConfig.MotorOutput.NeutralMode = if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast
                for (i in 0..4) {
                    if (turnTalon.configurator.apply(turnConfig, 0.25).isOK) break
                }
            }
        }
    }
}
