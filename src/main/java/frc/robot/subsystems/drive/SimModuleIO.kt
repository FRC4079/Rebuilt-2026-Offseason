package frc.robot.subsystems.drive

import frc.robot.utils.RobotParameters.SwerveParameters
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.DCMotorSim
import kotlin.math.abs

/**
 * Physics sim implementation of module IO. Simulation is always based on voltage control.
 */
class ModuleIOSim : ModuleIO {
    companion object {
        private val driveMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)
        private val turnMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)
    }

    private val driveSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                driveMotorModel,
                0.025,
                SwerveParameters.PhysicalParameters.DRIVE_MOTOR_GEAR_RATIO,
            ),
            driveMotorModel,
        )

    private val turnSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                turnMotorModel,
                0.004,
                SwerveParameters.PhysicalParameters.STEER_MOTOR_GEAR_RATIO,
            ),
            turnMotorModel,
        )

    private var driveClosedLoop = false
    private var turnClosedLoop = false
    private val driveController = PIDController(0.0, 0.0, 0.0)
    private val turnController = PIDController(0.0, 0.0, 0.0)
    private var driveFFVolts = 0.0
    private var driveAppliedVolts = 0.0
    private var turnAppliedVolts = 0.0

    init {
        turnController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs?) {
        if (inputs == null) return

        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.angularVelocity)
        } else {
            driveController.reset()
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.angularPosition)
        } else {
            turnController.reset()
        }

        driveSim.setInputVoltage(driveAppliedVolts.coerceIn(-12.0, 12.0))
        turnSim.setInputVoltage(turnAppliedVolts.coerceIn(-12.0, 12.0))
        driveSim.update(0.02)
        turnSim.update(0.02)

        inputs.driveConnected = true
        inputs.drivePositionRad = driveSim.angularPosition
        inputs.driveVelocityRadPerSec = driveSim.angularVelocity
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveSupplyCurrentAmps = abs(driveSim.currentDraw)

        inputs.turnConnected = true
        inputs.turnPosition = Rotation2d(turnSim.angularPosition)
        inputs.turnAbsolutePosition = Rotation2d(turnSim.angularPosition)
        inputs.turnSupplyCurrentAmps = abs(turnSim.currentDraw)
    }

    override fun runDriveOpenLoop(output: Double) {
        driveClosedLoop = false
        driveAppliedVolts = output
    }

    override fun runTurnOpenLoop(output: Double) {
        turnClosedLoop = false
        turnAppliedVolts = output
    }

    override fun runDriveVelocity(
        velocityRadPerSec: Double,
        feedforward: Double,
    ) {
        driveClosedLoop = true
        driveFFVolts = feedforward
        driveController.setSetpoint(velocityRadPerSec)
    }

    override fun runTurnPosition(rotation: Rotation2d?) {
        turnClosedLoop = true
        turnController.setSetpoint(rotation?.radians ?: turnController.setpoint)
    }

    override fun coast() {
        driveClosedLoop = false
        turnClosedLoop = false
        driveController.reset()
        turnController.reset()
        driveAppliedVolts = 0.0
        turnAppliedVolts = 0.0
    }
}
