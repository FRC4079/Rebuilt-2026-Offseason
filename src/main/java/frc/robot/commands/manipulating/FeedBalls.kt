package frc.robot.commands.manipulating

import frc.robot.mechanisms.transport.Transport
import frc.robot.mechanisms.transport.TransportIO
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class FeedBalls(
    val transport: Transport,
) : Command {
    override fun name(): String = "FeedBalls"

    override fun requirements(): Set<Mechanism?> = setOf(transport)

    override fun run(coroutine: Coroutine) {
        while (true) {
            transport.setState(TransportIO.TransportState.ACTIVE)
            coroutine.yield()
        }
    }

    override fun onCancel() {
        transport.setState(TransportIO.TransportState.STOP)
    }
}
