package frc.robot.commands.manipulating

import frc.robot.mechanisms.intake.Intake
import frc.robot.mechanisms.pivot.Pivot
import frc.robot.mechanisms.pivot.PivotIO
import frc.robot.mechanisms.transport.Transport
import frc.robot.mechanisms.transport.TransportIO
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class DeployIntake(
    val pivot: Pivot,
    val transport: Transport,
    val intake: Intake,
) : Command {
    override fun name(): String = "DeployIntake"

    override fun requirements(): Set<Mechanism> = setOf(pivot, transport)

    override fun run(coroutine: Coroutine?) {
        pivot.swapPosition()
        transport.toggleState()
        intake.setVelocity(-100.0)
        while (true) {
            coroutine?.yield()
        }
    }

    override fun onCancel() {
    }
}
