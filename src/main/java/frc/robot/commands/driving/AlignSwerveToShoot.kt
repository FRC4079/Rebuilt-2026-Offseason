package frc.robot.commands.driving

import frc.robot.mechanisms.OdometrySupplier
import frc.robot.mechanisms.drive.Swerve
import frc.robot.utils.RobotParameters
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.math.geometry.Rotation2d

/**
 * Points the robot at the hub so it can shoot. The hub is chosen based on the current alliance (see
 * [frc.robot.utils.PoseLookup]). Only rotates in place; the robot does not translate.
 */
class AlignSwerveToShoot(
    private val swerve: Swerve,
    private val odometrySupplier: OdometrySupplier,
) : Command {
    var angleToTurn: Rotation2d = Rotation2d.ZERO

    init {
        RobotParameters.SwerveParameters.PIDParameters.ROTATIONAL_PID
            .enableContinuousInput(-Math.PI, Math.PI)
        RobotParameters.SwerveParameters.PIDParameters.ROTATIONAL_PID
            .setTolerance(TOLERANCE_RAD)
    }

    override fun name(): String = "AlignSwerveToShoot"

    override fun requirements(): Set<Mechanism> = setOf(swerve)

    override fun run(coroutine: Coroutine) {
        RobotParameters.SwerveParameters.PIDParameters.ROTATIONAL_PID
            .reset()

        while (!RobotParameters.SwerveParameters.PIDParameters.ROTATIONAL_PID
                .atSetpoint()
        ) {
            angleToTurn = odometrySupplier.getAngleToHub()
            val currentYaw = swerve.gyroYaw
            val output =
                RobotParameters.SwerveParameters.PIDParameters.ROTATIONAL_PID
                    .calculate(currentYaw.radians, angleToTurn.radians)

            Logger.recordOutput("AlignSwerveToShoot/AngleToTurnDegrees", angleToTurn.degrees)
            Logger.recordOutput("AlignSwerveToShoot/CurrentYawDegrees", currentYaw.degrees)
            Logger.recordOutput("AlignSwerveToShoot/Output", output)

            swerve.setDriveSpeeds(0.0, 0.0, output)
            coroutine.yield()
        }
    }

    override fun onCancel() {
        swerve.stop()
        RobotParameters.SwerveParameters.PIDParameters.ROTATIONAL_PID
            .reset()
    }

    companion object {
        private const val TOLERANCE_RAD = 0.02
    }
}
