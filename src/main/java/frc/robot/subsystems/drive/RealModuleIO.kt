package frc.robot.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.robot.utils.PhoenixOdometryThread
import frc.robot.utils.PhoenixUtils
import frc.robot.utils.PhoenixUtils.tryUntilOk
import frc.robot.utils.RobotParameters.SwerveParameters
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Current
import org.wpilib.units.measure.Voltage
import java.util.Queue
import java.util.concurrent.Executors

class ModuleIOComp(
    config: SwerveParameters.ModuleConfig,
) : ModuleIO {
    companion object {
        private const val driveCurrentLimitAmps = 80.0
        private const val turnCurrentLimitAmps = 40.0
        const val driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)
        const val turnReduction = 150.0 / 7.0
        private val brakeModeExecutor = Executors.newFixedThreadPool(8)
    }

    private val driveTalon = TalonFX(config.driveMotorId, CANBus(config.CANBus))
    private val turnTalon = TalonFX(config.turnMotorId, CANBus(config.CANBus))
    private val encoder = CANcoder(config.encoderID, CANBus(config.CANBus))
    private val encoderOffset = config.encoderOffset

    private val driveConfig = TalonFXConfiguration()
    private val turnConfig = TalonFXConfiguration()

    private val torqueCurrentRequest = TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val positionTorqueCurrentRequest = PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val velocityTorqueCurrentRequest = VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)

    private val drivePosition: StatusSignal<Angle>
    private val drivePositionQueue: Queue<Double>
    private val driveVelocity: StatusSignal<AngularVelocity>
    private val driveAppliedVolts: StatusSignal<Voltage>
    private val driveSupplyCurrentAmps: StatusSignal<Current>
    private val driveTorqueCurrentAmps: StatusSignal<Current>

    private val turnAbsolutePosition: StatusSignal<Angle>
    private val turnPosition: StatusSignal<Angle>
    private val turnPositionQueue: Queue<Double>
    private val turnVelocity: StatusSignal<AngularVelocity>
    private val turnAppliedVolts: StatusSignal<Voltage>
    private val turnSupplyCurrentAmps: StatusSignal<Current>
    private val turnTorqueCurrentAmps: StatusSignal<Current>

    init {
        // Configure drive motor
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        driveConfig.Slot0 = Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0)
        driveConfig.Feedback.SensorToMechanismRatio = driveReduction
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveCurrentLimitAmps
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveCurrentLimitAmps
        driveConfig.CurrentLimits.StatorCurrentLimit = driveCurrentLimitAmps
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        tryUntilOk(5) { driveTalon.configurator.apply(driveConfig, 0.25) }
        tryUntilOk(5) { driveTalon.setPosition(0.0, 0.25) }

        // Configure turn motor
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        turnConfig.Slot0 = Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0)
        turnConfig.Feedback.FeedbackRemoteSensorID = config.encoderID
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
        turnConfig.Feedback.RotorToSensorRatio = turnReduction
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = turnCurrentLimitAmps
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -turnCurrentLimitAmps
        turnConfig.CurrentLimits.StatorCurrentLimit = turnCurrentLimitAmps
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true
        turnConfig.MotorOutput.Inverted =
            if (config.turnInverted) {
                InvertedValue.Clockwise_Positive
            } else {
                InvertedValue.CounterClockwise_Positive
            }

        tryUntilOk(5) { turnTalon.configurator.apply(turnConfig, 0.25) }

        // Configure CANCoder
        val cancoderConfig = CANcoderConfiguration()
        cancoderConfig.MagnetSensor.MagnetOffset = config.encoderOffset.rotations
        cancoderConfig.MagnetSensor.SensorDirection =
            if (config.encoderInverted) {
                SensorDirectionValue.Clockwise_Positive
            } else {
                SensorDirectionValue.CounterClockwise_Positive
            }
        tryUntilOk(5) { encoder.configurator.apply(cancoderConfig) }

        // Create drive status signals
        drivePosition = driveTalon.position
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon.position.clone())
        driveVelocity = driveTalon.velocity
        driveAppliedVolts = driveTalon.motorVoltage
        driveSupplyCurrentAmps = driveTalon.supplyCurrent
        driveTorqueCurrentAmps = driveTalon.torqueCurrent

        // Create turn status signals
        turnAbsolutePosition = encoder.absolutePosition
        turnPosition = turnTalon.position
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.position.clone())
        turnVelocity = turnTalon.velocity
        turnAppliedVolts = turnTalon.motorVoltage
        turnSupplyCurrentAmps = turnTalon.supplyCurrent
        turnTorqueCurrentAmps = turnTalon.torqueCurrent

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            SwerveParameters.OdometryConfig.ODOMETRY_FREQUENCY,
            drivePosition,
            turnPosition,
            turnAbsolutePosition,
        )
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrentAmps,
            driveTorqueCurrentAmps,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrentAmps,
            turnTorqueCurrentAmps,
        )
        tryUntilOk(5) { ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, encoder) }

        // Register signals for refresh
        PhoenixUtils.registerSignals(
            false,
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrentAmps,
            driveTorqueCurrentAmps,
            turnPosition,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrentAmps,
            turnTorqueCurrentAmps,
        )
    }

    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        inputs.data =
            ModuleIO.ModuleIOData(
                BaseStatusSignal.isAllGood(
                    drivePosition,
                    driveVelocity,
                    driveAppliedVolts,
                    driveSupplyCurrentAmps,
                    driveTorqueCurrentAmps,
                ),
                Units.rotationsToRadians(drivePosition.valueAsDouble),
                Units.rotationsToRadians(driveVelocity.valueAsDouble),
                driveAppliedVolts.valueAsDouble,
                driveSupplyCurrentAmps.valueAsDouble,
                driveTorqueCurrentAmps.valueAsDouble,
                BaseStatusSignal.isAllGood(
                    turnPosition,
                    turnVelocity,
                    turnAppliedVolts,
                    turnSupplyCurrentAmps,
                    turnTorqueCurrentAmps,
                ),
                BaseStatusSignal.isAllGood(turnAbsolutePosition),
                Rotation2d.fromRotations(turnAbsolutePosition.valueAsDouble).minus(encoderOffset),
                Rotation2d.fromRotations(turnPosition.valueAsDouble),
                Units.rotationsToRadians(turnVelocity.valueAsDouble),
                turnAppliedVolts.valueAsDouble,
                turnSupplyCurrentAmps.valueAsDouble,
                turnTorqueCurrentAmps.valueAsDouble,
            )

        inputs.odometryDrivePositionsRad = drivePositionQueue.map { Units.rotationsToRadians(it) }.toDoubleArray()
        inputs.odometryTurnPositions = turnPositionQueue.map { Rotation2d.fromRotations(it) }.toTypedArray()
        drivePositionQueue.clear()
        turnPositionQueue.clear()
    }

    override fun runDriveOpenLoop(output: Double) {
        driveTalon.setControl(torqueCurrentRequest.withOutput(output))
    }

    override fun runTurnOpenLoop(output: Double) {
        turnTalon.setControl(torqueCurrentRequest.withOutput(output))
    }

    override fun runDriveVelocity(
        velocityRadPerSec: Double,
        feedforward: Double,
    ) {
        driveTalon.setControl(
            velocityTorqueCurrentRequest
                .withVelocity(Units.radiansToRotations(velocityRadPerSec))
                .withFeedForward(feedforward),
        )
    }

    override fun runTurnPosition(rotation: Rotation2d) {
        turnTalon.setControl(positionTorqueCurrentRequest.withPosition(rotation.rotations))
    }

    override fun setDrivePID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        driveConfig.Slot0.kP = kP
        driveConfig.Slot0.kI = kI
        driveConfig.Slot0.kD = kD
        tryUntilOk(5) { driveTalon.configurator.apply(driveConfig, 0.25) }
    }

    override fun setTurnPID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        turnConfig.Slot0.kP = kP
        turnConfig.Slot0.kI = kI
        turnConfig.Slot0.kD = kD
        tryUntilOk(5) { turnTalon.configurator.apply(turnConfig, 0.25) }
    }

    override fun setBrakeMode(enabled: Boolean) {
        brakeModeExecutor.execute {
            synchronized(driveConfig) {
                driveConfig.MotorOutput.NeutralMode = if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast
                tryUntilOk(5) { driveTalon.configurator.apply(driveConfig, 0.25) }
            }
        }
        brakeModeExecutor.execute {
            synchronized(turnConfig) {
                turnConfig.MotorOutput.NeutralMode = if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast
                tryUntilOk(5) { turnTalon.configurator.apply(turnConfig, 0.25) }
            }
        }
    }
}
