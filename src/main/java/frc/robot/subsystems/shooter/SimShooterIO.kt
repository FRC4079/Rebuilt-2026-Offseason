package frc.robot.subsystems.shooter

import frc.robot.utils.RobotParameters
import org.wpilib.math.controller.PIDController
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.DCMotorSim
import java.lang.Math.clamp
import kotlin.math.abs

class SimShooterIO : ShooterIO {
    private val shooterMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)

    private val topSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                shooterMotorModel,
                0.1,
                RobotParameters.ShooterParameters.SHOOTER_GEAR_RATIO,
            ),
            shooterMotorModel,
        )

    private val bottomSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                shooterMotorModel,
                0.1,
                RobotParameters.ShooterParameters.SHOOTER_GEAR_RATIO,
            ),
            shooterMotorModel,
        )

    private var closedLoop = false
    private val topController = PIDController(0.0, 0.0, 0.0)
    private val bottomController = PIDController(0.0, 0.0, 0.0)
    private var targetVelocityRadPerSec = 0.0
    private var topAppliedVolts = 0.0
    private var bottomAppliedVolts = 0.0

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        if (closedLoop) {
            topAppliedVolts = topController.calculate(topSim.angularVelocity, targetVelocityRadPerSec)
            bottomAppliedVolts = bottomController.calculate(bottomSim.angularVelocity, targetVelocityRadPerSec)
            topSim.setInputVoltage(clamp(topAppliedVolts, -12.0, 12.0))
            bottomSim.setInputVoltage(clamp(bottomAppliedVolts, -12.0, 12.0))
        } else {
            topController.reset()
            bottomController.reset()
        }

        topSim.update(0.02)
        bottomSim.update(0.02)

        inputs.data.topMotorConnected = true
        inputs.data.bottomMotorConnected = true
        inputs.data.topPositionRad = topSim.angularPosition
        inputs.data.bottomPositionRad = bottomSim.angularPosition
        inputs.data.topVelocityRadPerSec = topSim.angularVelocity
        inputs.data.bottomVelocityRadPerSec = bottomSim.angularVelocity
        inputs.data.topAppliedVolts = topAppliedVolts
        inputs.data.bottomAppliedVolts = bottomAppliedVolts
        inputs.data.topSupplyCurrentAmps = abs(topSim.currentDraw)
        inputs.data.bottomSupplyCurrentAmps = abs(bottomSim.currentDraw)
        inputs.data.topTorqueCurrentAmps = 0.0
        inputs.data.bottomTorqueCurrentAmps = 0.0
    }

    override fun setPower(topPower: Double, bottomPower: Double) {
        closedLoop = false
        topSim.setInputVoltage(clamp(topPower * 12.0, -12.0, 12.0))
        bottomSim.setInputVoltage(clamp(bottomPower * 12.0, -12.0, 12.0))
    }

    override fun setShooterState(state: ShooterIO.ShooterState) {
        closedLoop = true
        targetVelocityRadPerSec = state.velocityRadPerSec
        // For simplicity in sim, we just track the target
    }

    override fun disablePower() {
        closedLoop = false
        topSim.setInputVoltage(0.0)
        bottomSim.setInputVoltage(0.0)
    }

    override fun setCurrentLimit(currentLimit: Int) {
        // Not directly simulated with current limit in this simple model
    }
}