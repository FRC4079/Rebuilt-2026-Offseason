package frc.robot.subsystems.pivot

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
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.robot.utils.RobotParameters
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.phoenix.PhoenixOdometryThread
import frc.robot.utils.phoenix.PhoenixUtils
import frc.robot.utils.phoenix.PhoenixUtils.tryUntilOk
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Current
import org.wpilib.units.measure.Voltage
import java.util.Queue
import java.util.concurrent.Executors
import frc.robot.utils.RobotParameters.PivotParameters
import kotlin.math.abs

class RealPivotIO (

) : PivotIO {
    private val pivotTalon = TalonFX(PivotParameters.pivotMotor, CANBus(PivotParameters.PIVOT_CANPORT))
    private val pivotConfig = TalonFXConfiguration()
    private val pivotPositionRequest: PositionVoltage = PositionVoltage(0.0).withSlot(0)

    private val pivotPosition: StatusSignal<Angle>

    init {
        // Configure pivot motor ouughh...
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        pivotConfig.Slot0 = Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0)
        pivotConfig.Feedback.SensorToMechanismRatio = SwerveParameters.PhysicalParameters.DRIVE_MOTOR_GEAR_RATIO

        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        pivotConfig.CurrentLimits.StatorCurrentLimit = SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true

        pivotConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        tryUntilOk(5) { pivotTalon.configurator.apply(pivotConfig, 0.25) }
        tryUntilOk(5) { pivotTalon.setPosition(0.0, 0.25) }

        // Configure pivot signal
        pivotPosition = pivotTalon.position
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pivotPosition
        )
        PhoenixUtils.registerSignals(
            false,
            pivotPosition
        )
    }

    override fun updateInputs(inputs: PivotIO.PivotIOInputs) {
        inputs.data.pivotConnected =
            BaseStatusSignal.isAllGood(
                pivotPosition
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
