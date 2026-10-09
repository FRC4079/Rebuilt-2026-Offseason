package frc.robot.mechanisms.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.utils.RobotParameters.ShooterParameters
import frc.robot.utils.RobotParameters.SwerveParameters
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Current
import org.wpilib.units.measure.Voltage
import java.util.function.Supplier

class RealShooterIO : ShooterIO {
    private val shooterTopTalon = TalonFX(ShooterParameters.shooterLeftMotor, CANBus(ShooterParameters.SHOOTER_CANPORT))
    private val shooterBottomTalon = TalonFX(ShooterParameters.shooterRightMotor, CANBus(ShooterParameters.SHOOTER_CANPORT))

    private val topConfig = TalonFXConfiguration()
    private val bottomConfig = TalonFXConfiguration()

    private val velocityRequest: VelocityVoltage = VelocityVoltage(0.0).withSlot(0)

    private val topPosition: StatusSignal<Angle>
    private val bottomPosition: StatusSignal<Angle>
    private val topVelocity: StatusSignal<AngularVelocity>
    private val bottomVelocity: StatusSignal<AngularVelocity>
    private val topVoltage: StatusSignal<Voltage>
    private val bottomVoltage: StatusSignal<Voltage>
    private val topSupplyCurrent: StatusSignal<Current>
    private val bottomSupplyCurrent: StatusSignal<Current>
    private val topTorqueCurrent: StatusSignal<Current>
    private val bottomTorqueCurrent: StatusSignal<Current>

    init {
        // Configure top shooter motor
        topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
        topConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        topConfig.Slot0 = Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0)
        topConfig.Feedback.SensorToMechanismRatio = ShooterParameters.SHOOTER_GEAR_RATIO

        topConfig.TorqueCurrent.PeakForwardTorqueCurrent = SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        topConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        topConfig.CurrentLimits.StatorCurrentLimit = SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        topConfig.CurrentLimits.StatorCurrentLimitEnable = true

        topConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        // Configure bottom shooter motor
        bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
        bottomConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

        bottomConfig.Slot0 = Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0)
        bottomConfig.Feedback.SensorToMechanismRatio = ShooterParameters.SHOOTER_GEAR_RATIO

        bottomConfig.TorqueCurrent.PeakForwardTorqueCurrent = SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        bottomConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        bottomConfig.CurrentLimits.StatorCurrentLimit = SwerveParameters.PhysicalParameters.DRIVE_CURRENT_LIMIT
        bottomConfig.CurrentLimits.StatorCurrentLimitEnable = true

        bottomConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02

        var error = StatusCode.OK
        for (i in 0..4) {
            if (shooterTopTalon.configurator.apply(topConfig, 0.25).isOK) break
        }
        for (i in 0..4) {
            if (shooterBottomTalon.configurator.apply(bottomConfig, 0.25).isOK) break
        }
        for (i in 0..4) {
            if (shooterTopTalon.setPosition(0.0, 0.25).isOK) break
        }
        for (i in 0..4) {
            if (shooterBottomTalon.setPosition(0.0, 0.25).isOK) break
        }

        // Configure signals
        topPosition = shooterTopTalon.position
        bottomPosition = shooterBottomTalon.position
        topVelocity = shooterTopTalon.velocity
        bottomVelocity = shooterBottomTalon.velocity
        topVoltage = shooterTopTalon.motorVoltage
        bottomVoltage = shooterBottomTalon.motorVoltage
        topSupplyCurrent = shooterTopTalon.supplyCurrent
        bottomSupplyCurrent = shooterBottomTalon.supplyCurrent
        topTorqueCurrent = shooterTopTalon.torqueCurrent
        bottomTorqueCurrent = shooterBottomTalon.torqueCurrent

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            topPosition,
            bottomPosition,
            topVelocity,
            bottomVelocity,
            topVoltage,
            bottomVoltage,
            topSupplyCurrent,
            bottomSupplyCurrent,
            topTorqueCurrent,
            bottomTorqueCurrent,
        )

    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        inputs.data.topMotorConnected =
            BaseStatusSignal.isAllGood(
                topPosition,
                topVelocity,
                topVoltage,
            )
        inputs.data.bottomMotorConnected =
            BaseStatusSignal.isAllGood(
                bottomPosition,
                bottomVelocity,
                bottomVoltage,
            )
        inputs.data.topPositionRad = Units.rotationsToRadians(topPosition.valueAsDouble)
        inputs.data.bottomPositionRad = Units.rotationsToRadians(bottomPosition.valueAsDouble)
        inputs.data.topVelocityRadPerSec = Units.rotationsToRadians(topVelocity.valueAsDouble)
        inputs.data.bottomVelocityRadPerSec = Units.rotationsToRadians(bottomVelocity.valueAsDouble)
        inputs.data.topAppliedVolts = topVoltage.valueAsDouble
        inputs.data.bottomAppliedVolts = bottomVoltage.valueAsDouble
        inputs.data.topSupplyCurrentAmps = topSupplyCurrent.valueAsDouble
        inputs.data.bottomSupplyCurrentAmps = bottomSupplyCurrent.valueAsDouble
        inputs.data.topTorqueCurrentAmps = topTorqueCurrent.valueAsDouble
        inputs.data.bottomTorqueCurrentAmps = bottomTorqueCurrent.valueAsDouble
    }

    private val openLoopRequest =
        com.ctre.phoenix6.controls
            .DutyCycleOut(0.0)
            .withUpdateFreqHz(0.0)

    override fun setPower(
        topPower: Double,
        bottomPower: Double,
    ) {
        shooterTopTalon.setControl(openLoopRequest.withOutput(topPower))
        shooterBottomTalon.setControl(openLoopRequest.withOutput(bottomPower))
    }

    override fun setShooterState(state: ShooterIO.ShooterState) {
        val velocityRotPerSec = Units.radiansToRotations(state.velocityRadPerSec)
        shooterTopTalon.setControl(velocityRequest.withVelocity(velocityRotPerSec))
        shooterBottomTalon.setControl(velocityRequest.withVelocity(velocityRotPerSec))
    }

    override fun disablePower() {
        shooterTopTalon.setControl(openLoopRequest.withOutput(0.0))
        shooterBottomTalon.setControl(openLoopRequest.withOutput(0.0))
    }

    override fun setCurrentLimit(currentLimit: Int) {
        topConfig.CurrentLimits.StatorCurrentLimit = currentLimit.toDouble()
        bottomConfig.CurrentLimits.StatorCurrentLimit = currentLimit.toDouble()
        for (i in 0..4) {
            if (shooterTopTalon.configurator.apply(topConfig, 0.25).isOK) break
        }
        for (i in 0..4) {
            if (shooterBottomTalon.configurator.apply(bottomConfig, 0.25).isOK) break
        }
    }
}
