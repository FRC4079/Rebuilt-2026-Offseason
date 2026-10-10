package frc.robot.mechanisms.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.utils.RobotParameters.IntakeParameters
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Current
import org.wpilib.units.measure.Voltage

class RealIntakeIO(
    config: IntakeParameters,
) : IntakeIO {
    private val intakeMotor = TalonFX(config.intakeMotor, CANBus(config.INTAKE_CANPORT))
    private val intakeConfig = TalonFXConfiguration()

    private val voltageControl = VoltageOut(0.0)
    private val positionControl = PositionVoltage(0.0).withSlot(0)
    private val velocityControl = VelocityVoltage(0.0).withSlot(0)

    private val positionSignal: StatusSignal<Angle> = intakeMotor.position
    private val velocitySignal: StatusSignal<AngularVelocity> = intakeMotor.velocity
    private val voltageSignal: StatusSignal<Voltage> = intakeMotor.motorVoltage
    private val supplyCurrentSignal: StatusSignal<Current> = intakeMotor.supplyCurrent
    private val torqueCurrentSignal: StatusSignal<Current> = intakeMotor.torqueCurrent

    init {
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
        intakeConfig.Slot0 =
            Slot0Configs()
                .withKP(IntakeParameters.PIDParameters.INTAKE_PID.p)
                .withKI(IntakeParameters.PIDParameters.INTAKE_PID.i)
                .withKD(IntakeParameters.PIDParameters.INTAKE_PID.d)

        intakeConfig.Feedback.SensorToMechanismRatio = IntakeParameters.PhysicalParameters.INTAKE_GEAR_RATIO
        intakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = IntakeParameters.PhysicalParameters.INTAKE_CURRENT_LIMIT
        intakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = -IntakeParameters.PhysicalParameters.INTAKE_CURRENT_LIMIT
        intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeParameters.PhysicalParameters.INTAKE_CURRENT_LIMIT
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true
        intakeConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        for (i in 0..4) {
            if (intakeMotor.configurator.apply(intakeConfig, 0.25).isOK) break
        }

        // TODO: set frequencyHz to real value
        BaseStatusSignal.setUpdateFrequencyForAll(
            0.0,
            positionSignal,
            velocitySignal,
            voltageSignal,
            supplyCurrentSignal,
            torqueCurrentSignal,
        )

        ParentDevice.optimizeBusUtilizationForAll(intakeMotor)
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.data.intakeConnected =
            BaseStatusSignal.isAllGood(
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                torqueCurrentSignal,
            )
        inputs.data.intakeVelocityRadPerSec = velocitySignal.valueAsDouble
        inputs.data.intakeAppliedVolts = voltageSignal.valueAsDouble
        inputs.data.intakeSupplyCurrentAmps = supplyCurrentSignal.valueAsDouble
        inputs.data.intakeTorqueCurrentAmps = torqueCurrentSignal.valueAsDouble
    }

    override fun setVoltage(volts: Double) {
        intakeMotor.setControl(voltageControl.withOutput(volts))
    }

    override fun setPosition(positionRad: Double) {
        intakeMotor.setControl(positionControl.withPosition(positionRad))
    }

    override fun setVelocity(velocityRadPerSec: Double) {
        intakeMotor.setControl(velocityControl.withVelocity(velocityRadPerSec))
    }

    override fun setIntakePID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        intakeConfig.Slot0.kP = kP
        intakeConfig.Slot0.kI = kI
        intakeConfig.Slot0.kD = kD

        for (i in 0..4) {
            if (intakeMotor.configurator.apply(intakeConfig, 0.25).isOK) break
        }
    }

    override fun stop() {
        intakeMotor.setControl(voltageControl.withOutput(0.0))
    }
}
