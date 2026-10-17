package frc.robot.commands.driving

import frc.robot.mechanisms.drive.Swerve
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism


class ResetGyro (
    val swerve: Swerve
) : Command {
    override fun name(): String = "resetGyro"

    override fun requirements(): Set<Mechanism> = setOf(swerve)

    override fun run(coroutine: Coroutine?) {
        swerve.resetGyro()
    }

    override fun onCancel() {

    }
}