package frc.robot.mechanisms.hood

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.utils.RobotParameters.HoodParameters
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Current
import org.wpilib.units.measure.Voltage

class RealHoodIO : HoodIO {
    private val hoodTalon = TalonFX(HoodParameters.hoodMotor, CANBus(HoodParameters.HOOD_CANPORT))
    private val hoodConfig = TalonFXConfiguration()
    private val hoodPositionRequest: PositionVoltage = PositionVoltage(0.0).withSlot(0)
    private val hoodPosition: StatusSignal<Angle>
    private val hoodVelocity: StatusSignal<AngularVelocity>
    private val hoodVoltage: StatusSignal<Voltage>
    private val hoodSupplyCurrent: StatusSignal<Current>
    private val hoodTorqueCurrent: StatusSignal<Current>

    init {
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        hoodConfig.Slot0 =
            Slot0Configs()
                .withKP(HoodParameters.PIDParameters.HOOD_PID.p)
                .withKI(HoodParameters.PIDParameters.HOOD_PID.i)
                .withKD(HoodParameters.PIDParameters.HOOD_PID.d)
        hoodConfig.Feedback.SensorToMechanismRatio = HoodParameters.HOOD_GEAR_RATIO

        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = HoodParameters.PhysicalParameters.HOOD_CURRENT_LIMIT
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -HoodParameters.PhysicalParameters.HOOD_CURRENT_LIMIT
        hoodConfig.CurrentLimits.StatorCurrentLimit = HoodParameters.PhysicalParameters.HOOD_CURRENT_LIMIT
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true

        hoodConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        for (i in 0..4) {
            if (hoodTalon.configurator.apply(hoodConfig, 0.25).isOK) break
        }
        for (i in 0..4) {
            if (hoodTalon.setPosition(0.0, 0.25).isOK) break
        }

        hoodPosition = hoodTalon.position
        hoodVelocity = hoodTalon.velocity
        hoodVoltage = hoodTalon.motorVoltage
        hoodSupplyCurrent = hoodTalon.supplyCurrent
        hoodTorqueCurrent = hoodTalon.torqueCurrent

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            hoodPosition,
            hoodVelocity,
            hoodVoltage,
            hoodSupplyCurrent,
            hoodTorqueCurrent,
        )
    }

    override fun updateInputs(inputs: HoodIO.HoodIOInputs) {
        inputs.data.hoodConnected =
            BaseStatusSignal.isAllGood(
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
            )
        inputs.data.hoodPositionRad = Units.rotationsToRadians(hoodPosition.valueAsDouble)
        inputs.data.hoodVelocityRadPerSec = Units.rotationsToRadians(hoodVelocity.valueAsDouble)
        inputs.data.hoodAppliedVolts = hoodVoltage.valueAsDouble
        inputs.data.hoodSupplyCurrentAmps = hoodSupplyCurrent.valueAsDouble
        inputs.data.hoodTorqueCurrentAmps = hoodTorqueCurrent.valueAsDouble
    }

    override fun setPower(power: Double) {
        hoodTalon.setVoltage(power)
    }

    override fun setHoodState(state: HoodIO.HoodPositionState) {
        hoodTalon.setControl(hoodPositionRequest.withPosition(Units.radiansToRotations(state.positionRad)))
    }

    override fun setPosition(positionRad: Double) {
        hoodTalon.setControl(hoodPositionRequest.withPosition(Units.radiansToRotations(positionRad)))
    }

    override fun disablePower() {
        hoodTalon.stopMotor()
    }

    override fun setCurrentLimit(currentLimit: Int) {
        hoodConfig.CurrentLimits.StatorCurrentLimit = currentLimit.toDouble()
        for (i in 0..4) {
            if (hoodTalon.configurator.apply(hoodConfig, 0.25).isOK) break
        }
    }
}
