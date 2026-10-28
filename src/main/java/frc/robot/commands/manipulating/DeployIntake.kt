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
//    val transport: Transport,
//    val intake: Intake,
) : Command {
    override fun name(): String = "DeployIntake"

    override fun requirements(): Set<Mechanism> = setOf(pivot)

    override fun run(coroutine: Coroutine?) {
        while (true) {
            pivot.setState(PivotIO.PivotState.DEPLOY)
//            transport.setState(TransportIO.TransportState.ACTIVE)
//            intake.setVelocity(-600.0)
            coroutine?.yield()
        }
    }

    override fun onCancel() {
        pivot.setState(PivotIO.PivotState.STOW)
//        intake.stop()
//        transport.disable()
    }
}
