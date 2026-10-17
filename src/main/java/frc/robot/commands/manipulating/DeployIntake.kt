package frc.robot.commands.manipulating

import frc.robot.mechanisms.pivot.Pivot
import frc.robot.mechanisms.pivot.PivotIO
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class DeployIntake(
    val pivot: Pivot,
) : Command {
    override fun name(): String = "DeployIntake"

    override fun requirements(): Set<Mechanism> = setOf(pivot)

    override fun run(coroutine: Coroutine?) {
        while (true) {
            println("alkdsfjlk;asjdfl;kasjdf;lkjaslk;dfjasdf")
            pivot.setState(PivotIO.PivotState.DEPLOY)
            coroutine?.yield()
        }
    }

    override fun onCancel() {
        pivot.setState(PivotIO.PivotState.STOW)
    }
}
