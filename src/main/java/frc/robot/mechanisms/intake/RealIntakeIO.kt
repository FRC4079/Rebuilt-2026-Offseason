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

class RealIntakeIO() : IntakeIO {
    private val intakeMotorLeft = TalonFX(IntakeParameters.intakeMotorLeft, CANBus(IntakeParameters.INTAKE_CANPORT))
    private  val intakeMotorRight = TalonFX(IntakeParameters.intakeMotorRight, CANBus(IntakeParameters.INTAKE_CANPORT))

    private val intakeConfig = TalonFXConfiguration()

    private val voltageControl = VoltageOut(0.0)
    private val positionControl = PositionVoltage(0.0).withSlot(0)
    private val velocityControl = VelocityVoltage(0.0).withSlot(0)

    private val positionSignalLeft: StatusSignal<Angle> = intakeMotorLeft.position
    private val velocitySignalLeft: StatusSignal<AngularVelocity> = intakeMotorLeft.velocity
    private val voltageSignalLeft: StatusSignal<Voltage> = intakeMotorLeft.motorVoltage
    private val supplyCurrentSignalLeft: StatusSignal<Current> = intakeMotorLeft.supplyCurrent
    private val torqueCurrentSignalLeft: StatusSignal<Current> = intakeMotorLeft.torqueCurrent

    private val positionSignalRight: StatusSignal<Angle> = intakeMotorRight.position
    private val velocitySignalRight: StatusSignal<AngularVelocity> = intakeMotorRight.velocity
    private val voltageSignalRight: StatusSignal<Voltage> = intakeMotorRight.motorVoltage
    private val supplyCurrentSignalRight: StatusSignal<Current> = intakeMotorRight.supplyCurrent
    private val torqueCurrentSignalRight: StatusSignal<Current> = intakeMotorRight.torqueCurrent

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
            if (intakeMotorLeft.configurator.apply(intakeConfig, 0.25).isOK) break
        }

        for (i in 0..4) {
            if (intakeMotorRight.configurator.apply(intakeConfig, 0.25).isOK) break
        }

        // TODO: set frequencyHz to real value
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            positionSignalLeft,
            velocitySignalLeft,
            voltageSignalLeft,
            supplyCurrentSignalLeft,
            torqueCurrentSignalLeft,
        )

        ParentDevice.optimizeBusUtilizationForAll(intakeMotorLeft)

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            positionSignalRight,
            velocitySignalRight,
            voltageSignalRight,
            supplyCurrentSignalRight,
            torqueCurrentSignalRight,
        )

        ParentDevice.optimizeBusUtilizationForAll(intakeMotorRight)
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.data.intakeConnected =
            BaseStatusSignal.isAllGood(
                velocitySignalLeft,
                voltageSignalLeft,
                supplyCurrentSignalLeft,
                torqueCurrentSignalLeft,
            ) &&
            BaseStatusSignal.isAllGood(
                velocitySignalRight,
                voltageSignalRight,
                supplyCurrentSignalRight,
                torqueCurrentSignalRight
            )

        inputs.data.intakeVelocityRadPerSec = velocitySignalLeft.valueAsDouble
        inputs.data.intakeAppliedVolts = voltageSignalLeft.valueAsDouble
        inputs.data.intakeSupplyCurrentAmps = supplyCurrentSignalLeft.valueAsDouble
        inputs.data.intakeTorqueCurrentAmps = torqueCurrentSignalLeft.valueAsDouble
    }

    override fun setVoltage(volts: Double) {
        intakeMotorLeft.setControl(voltageControl.withOutput(volts))
        intakeMotorRight.setControl(voltageControl.withOutput(volts))
    }

    override fun setPosition(positionRad: Double) {
        intakeMotorLeft.setControl(positionControl.withPosition(positionRad))
        intakeMotorRight.setControl(positionControl.withPosition(positionRad))
    }

    override fun setVelocity(velocityRadPerSec: Double) {
        intakeMotorLeft.setControl(velocityControl.withVelocity(velocityRadPerSec))
        intakeMotorRight.setControl(velocityControl.withVelocity(velocityRadPerSec))
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
            if (intakeMotorLeft.configurator.apply(intakeConfig, 0.25).isOK) break
        }
        for (i in 0..4) {
            if (intakeMotorRight.configurator.apply(intakeConfig, 0.25).isOK) break
        }
    }

    override fun stop() {
        intakeMotorLeft.stopMotor()
        intakeMotorRight.stopMotor()
    }
}
