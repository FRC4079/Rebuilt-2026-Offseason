package frc.robot.mechanisms.transport

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.utils.RobotParameters.TransportParameters
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle

class RealTransportIO (
    config: TransportParameters
) : TransportIO {
    private val indexerTalon = TalonFX(config.indexerMotor, CANBus(config.INDEXER_CANPORT))
    private val indexerConfig = TalonFXConfiguration()
    private val indexerVelocityRequest: VelocityVoltage = VelocityVoltage(0.0).withSlot(0)

    private val hopperTalon = TalonFX(config.hopperMotor, CANBus(config.HOPPER_CANPORT))
    private val hopperConfig = TalonFXConfiguration()
    private val hopperVelocityRequest: VelocityVoltage = VelocityVoltage(0.0).withSlot(0)

    private val indexerPosition: StatusSignal<Angle>
    private val hopperPosition: StatusSignal<Angle>

    init {
        // Configure indexer motor ouughh...
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        indexerConfig.Slot0 =
            Slot0Configs()
                .withKP(TransportParameters.PIDParameters.INDEXER_PID.p)
                .withKI(TransportParameters.PIDParameters.INDEXER_PID.i)
                .withKD(TransportParameters.PIDParameters.INDEXER_PID.d)
        indexerConfig.Feedback.SensorToMechanismRatio = TransportParameters.INDEXER_GEAR_RATIO

        // TODO: Replace these values with real values
        indexerConfig.TorqueCurrent.PeakForwardTorqueCurrent = TransportParameters.PhysicalParameters.TRANSPORT_CURRENT_LIMIT
        indexerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -TransportParameters.PhysicalParameters.TRANSPORT_CURRENT_LIMIT
        indexerConfig.CurrentLimits.StatorCurrentLimit = TransportParameters.PhysicalParameters.TRANSPORT_CURRENT_LIMIT
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true
        indexerConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        for (i in 0..4) {
            if (indexerTalon.configurator.apply(indexerConfig, 0.25).isOK) break
        }

        // Configure hopper motor ouughh...
        hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        hopperConfig.Slot0 =
            Slot0Configs()
                .withKP(TransportParameters.PIDParameters.HOPPER_PID.p)
                .withKI(TransportParameters.PIDParameters.HOPPER_PID.i)
                .withKD(TransportParameters.PIDParameters.HOPPER_PID.d)
        hopperConfig.Feedback.SensorToMechanismRatio = TransportParameters.HOPPER_GEAR_RATIO

        // TODO: Replace these values with real values
        hopperConfig.TorqueCurrent.PeakForwardTorqueCurrent = TransportParameters.PhysicalParameters.TRANSPORT_CURRENT_LIMIT
        hopperConfig.TorqueCurrent.PeakReverseTorqueCurrent = -TransportParameters.PhysicalParameters.TRANSPORT_CURRENT_LIMIT
        hopperConfig.CurrentLimits.StatorCurrentLimit = TransportParameters.PhysicalParameters.TRANSPORT_CURRENT_LIMIT
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true
        hopperConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        for (i in 0..4) {
            if (hopperTalon.configurator.apply(hopperConfig, 0.25).isOK) break
        }

        // Configure signals
        indexerPosition = indexerTalon.position
        hopperPosition = hopperTalon.position
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            indexerPosition,
            hopperPosition,
        )

        ParentDevice.optimizeBusUtilizationForAll(indexerTalon, hopperTalon)


    }

    override fun updateInputs(inputs: TransportIO.TransportIOInputs) {
        inputs.data.indexerConnected =
            BaseStatusSignal.isAllGood(
                indexerPosition,
            )
        inputs.data.indexerPositionRad = Units.rotationsToRadians(indexerTalon.position.valueAsDouble)
        inputs.data.indexerAppliedVolts = indexerTalon.motorVoltage.valueAsDouble
        inputs.data.indexerSupplyCurrentAmps = indexerTalon.supplyCurrent.valueAsDouble
        inputs.data.indexerTorqueCurrentAmps = indexerTalon.torqueCurrent.valueAsDouble
        inputs.data.indexerVelocityRadPerSec = indexerTalon.supplyCurrent.valueAsDouble

        inputs.data.hopperConnected =
            BaseStatusSignal.isAllGood(
                hopperPosition,
            )
        inputs.data.hopperPositionRad = Units.rotationsToRadians(hopperTalon.position.valueAsDouble)
        inputs.data.hopperAppliedVolts = hopperTalon.motorVoltage.valueAsDouble
        inputs.data.hopperSupplyCurrentAmps = hopperTalon.supplyCurrent.valueAsDouble
        inputs.data.hopperTorqueCurrentAmps = hopperTalon.torqueCurrent.valueAsDouble
        inputs.data.hopperVelocityRadPerSec = hopperTalon.supplyCurrent.valueAsDouble
    }

    /**
     * Sets the velocity of the transport motors based on state.
     *
     * @param state The desired state of the pivot.
     */
    override fun setTransportState(state: TransportIO.TransportState) {
        hopperTalon.setControl(hopperVelocityRequest.withVelocity(state.velocity))
        indexerTalon.setControl(indexerVelocityRequest.withVelocity(state.velocity))
    }

//    override fun setVoltage(volts: Double) {
//        intakeMotor.setControl(voltageControl.withOutput(volts))
//    }
//
//    override fun setPosition(positionRad: Double) {
//        intakeMotor.setControl(positionControl.withPosition(positionRad))
//    }
//
//    override fun setVelocity(velocityRadPerSec: Double) {
//        intakeMotor.setControl(velocityControl.withVelocity(velocityRadPerSec))
//    }

    override fun setHopperPID(
        hopkP: Double,
        hopkI: Double,
        hopkD: Double,
    ) {
        hopperConfig.Slot0.kP = hopkP
        hopperConfig.Slot0.kI = hopkI
        hopperConfig.Slot0.kD = hopkD

            for (i in 0..4) {
                if (hopperTalon.configurator.apply(hopperConfig).isOK) break
            }
    }

    override fun setIndexerPID(
        indkP: Double,
        indkI: Double,
        indkD: Double,
    ) {
        indexerConfig.Slot0.kP = indkP
        indexerConfig.Slot0.kI = indkI
        indexerConfig.Slot0.kD = indkD
        for (i in 0..4) {
            if (indexerTalon.configurator.apply(indexerConfig, 0.25).isOK) break
        }
    }
//
//    override fun stop() {
//        intakeMotor.setControl(voltageControl.withOutput(0.0))
//    }

}