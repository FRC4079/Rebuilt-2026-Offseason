package frc.robot.commands.manipulating

import frc.robot.mechanisms.intake.Intake
import frc.robot.mechanisms.transport.Transport
import frc.robot.mechanisms.transport.TransportIO
import frc.robot.mechanisms.transport.TransportIO.TransportState
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class SpinIntake(
    val intake: Intake,
    val transport: Transport,
) : Command {
    override fun name(): String = "SpinIntake"

    override fun requirements(): Set<Mechanism> = setOf(intake)

    override fun run(coroutine: Coroutine) {
        while (true) {
            intake.setVelocity(600.0)
            transport.setState(TransportState.ACTIVE)
            coroutine.yield()
        }
    }

    override fun onCancel() {
        intake.stop()
        transport.setState(TransportState.STOP)
    }
}
