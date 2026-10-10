package frc.robot.commands

import frc.robot.mechanisms.intake.Intake
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class SpinIntake(val intake: Intake) : Command {

    override fun name(): String = "SpinIntake"

    override fun requirements(): Set<Mechanism> = setOf(intake)

    override fun run(coroutine: Coroutine) {
        while (true) {
            intake.setVelocity(600.0)
            coroutine.yield()
        }
    }

    override fun onCancel() {
        intake.stop()
    }
}