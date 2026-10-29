package frc.robot.commands.manipulating.simple

import frc.robot.mechanisms.pivot.Pivot
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class RetractIntake(
    val pivot: Pivot,
//    val transport: Transport,
//    val intake: Intake,
) : Command {
    override fun name(): String = "RetractIntake"

    override fun requirements(): Set<Mechanism> = setOf(pivot)

    override fun run(coroutine: Coroutine?) {
        while (true) {
            pivot.setPosition(0.0)
//            transport.setState(TransportIO.TransportState.ACTIVE)
//            intake.setVelocity(-600.0)
            coroutine?.yield()
        }
    }

    override fun onCancel() {
    }
}
