package frc.robot.commands

import frc.robot.subsystems.drive.simple.Swerve
import frc.robot.utils.RobotParameters.SwerveParameters.PhysicalParameters.MAX_ANGULAR_SPEED
import frc.robot.utils.RobotParameters.SwerveParameters.PhysicalParameters.MAX_SPEED
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.X_DEADZONE
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.Y_DEADZONE
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.driverstation.XboxController
import kotlin.math.abs

/** Command to control the robot's swerve drive using the Xbox controller. */
class PadDrive(
    private val pad: XboxController,
) : Command {
    /**
     * Called every time the scheduler runs while the command is scheduled. This method retrieves the
     * current position from the controller, calculates the rotation, logs the joystick values, and
     * sets the drive speeds for the swerve subsystem.
     */
    override fun run(coroutine: Coroutine) {
        val position = positionSet(pad)
        val rotation = if (abs(pad.rightX) >= 0.1) -pad.rightX * MAX_ANGULAR_SPEED else 0.0

        Logger.recordOutput("PadDrive/XJoystick", position.first)
        Logger.recordOutput("PadDrive/YJoystick", position.second)
        Logger.recordOutput("PadDrive/Rotation", rotation)

        Swerve.setDriveSpeeds(position.second, position.first, rotation * 0.5)
    }

    override fun name(): String = "PadDrive"

    override fun requirements(): Set<Mechanism> = setOf(Swerve)

    companion object {
        /**
         * Sets the position based on the input from the controller.
         *
         * @param pad The controller.
         * @return The coordinate representing the position. The first element is the x-coordinate, and
         * the second element is the y-coordinate.
         */
        fun positionSet(pad: XboxController): Pair<Double, Double> {
            var x = -pad.leftX * MAX_SPEED
            if (abs(x) < X_DEADZONE * MAX_SPEED) x = 0.0

            var y = -pad.leftY * MAX_SPEED
            if (abs(y) < Y_DEADZONE * MAX_SPEED) y = 0.0

            return x to y
        }
    }
}
