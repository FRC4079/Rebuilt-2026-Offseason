package frc.robot.subsystems.drive

import frc.robot.subsystems.drive.ModuleIO.ModuleIOData
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs
import frc.robot.utils.RobotParameters.SwerveParameters
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import kotlin.math.abs
import org.wpilib.simulation.DCMotorSim

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
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
        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                driveFFVolts + driveController.calculate(driveSim.angularVelocityRadPerSec)
        } else {
            driveController.reset()
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.angularPositionRad)
        } else {
            turnController.reset()
        }

        // Update simulation state
        driveSim.setInputVoltage(driveAppliedVolts.coerceIn(-12.0, 12.0))
        turnSim.setInputVoltage(turnAppliedVolts.coerceIn(-12.0, 12.0))
        driveSim.update(0.02)
        turnSim.update(0.02)

        // Update drive inputs
        inputs.data =
            ModuleIOData(
                driveConnected = true,
                drivePositionRad = driveSim.angularPositionRad,
                driveVelocityRadPerSec = driveSim.angularVelocityRadPerSec,
                driveAppliedVolts = driveAppliedVolts,
                driveSupplyCurrentAmps = abs(driveSim.currentDrawAmps),
                driveTorqueCurrentAmps = 0.0,
                turnConnected = true,
                turnEncoderConnected = true,
                turnAbsolutePosition = Rotation2d(turnSim.angularPositionRad),
                turnPosition = Rotation2d(turnSim.angularPositionRad),
                turnVelocityRadPerSec = turnSim.angularVelocityRadPerSec,
                turnAppliedVolts = turnAppliedVolts,
                turnSupplyCurrentAmps = abs(turnSim.currentDrawAmps),
                turnTorqueCurrentAmps = 0.0,
            )

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryDrivePositionsRad = doubleArrayOf(inputs.data.drivePositionRad)
        inputs.odometryTurnPositions = arrayOf<Rotation2d?>(inputs.data.turnPosition)
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

    override fun runTurnPosition(rotation: Rotation2d) {
        turnClosedLoop = true
        turnController.setSetpoint(rotation.radians)
    }

    override fun setDrivePID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        driveController.setPID(kP, kI, kD)
    }

    override fun setTurnPID(
        kP: Double,
        kI: Double,
        kD: Double,
    ) {
        turnController.setPID(kP, kI, kD)
    }
}
