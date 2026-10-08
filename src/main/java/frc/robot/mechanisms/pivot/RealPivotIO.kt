package frc.robot.mechanisms.pivot

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.utils.RobotParameters.PivotParameters
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.phoenix.PhoenixUtils
import frc.robot.utils.phoenix.PhoenixUtils.tryUntilOk
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle

class RealPivotIO : PivotIO {
    private val pivotTalon = TalonFX(PivotParameters.pivotMotor, CANBus(PivotParameters.PIVOT_CANPORT))
    private val pivotConfig = TalonFXConfiguration()
    private val pivotPositionRequest: PositionVoltage = PositionVoltage(0.0).withSlot(0)

    private val pivotPosition: StatusSignal<Angle>

    init {
        // Configure pivot motor ouughh...
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        pivotConfig.Slot0 = Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0)
        pivotConfig.Feedback.SensorToMechanismRatio = PivotParameters.PIVOT_GEAR_RATIO

        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = PivotParameters.PhysicalParameters.PIVOT_CURRENT_LIMIT
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -PivotParameters.PhysicalParameters.PIVOT_CURRENT_LIMIT
        pivotConfig.CurrentLimits.StatorCurrentLimit = PivotParameters.PhysicalParameters.PIVOT_CURRENT_LIMIT
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true

        pivotConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        tryUntilOk(5) { pivotTalon.configurator.apply(pivotConfig, 0.25) }
        tryUntilOk(5) { pivotTalon.setPosition(0.0, 0.25) }

        // Configure pivot signal
        pivotPosition = pivotTalon.position
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pivotPosition,
        )
        PhoenixUtils.registerSignals(
            false,
            pivotPosition,
        )
    }

    override fun updateInputs(inputs: PivotIO.PivotIOInputs) {
        inputs.data.pivotConnected =
            BaseStatusSignal.isAllGood(
                pivotPosition,
            )
        inputs.data.pivotPositionRad = Units.rotationsToRadians(pivotTalon.position.valueAsDouble)
        inputs.data.pivotAppliedVolts = pivotTalon.motorVoltage.valueAsDouble
        inputs.data.pivotSupplyCurrentAmps = pivotTalon.supplyCurrent.valueAsDouble
        inputs.data.pivotTorqueCurrentAmps = pivotTalon.torqueCurrent.valueAsDouble
        inputs.data.pivotVelocityRadPerSec = pivotTalon.supplyCurrent.valueAsDouble
    }

    /**
     * Sets the angle of the simPivot motor based on state. Input is * by 360 degrees since the real motor takes in rotations.
     *
     * @param state The desired state of the pivot.
     */
    override fun setPivotState(state: PivotIO.PivotPosition) {
        pivotTalon.setControl(pivotPositionRequest.withPosition(state.position))
    }
}
