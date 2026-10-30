package frc.robot.commands.shooting

import frc.robot.mechanisms.shooter.Shooter
import frc.robot.mechanisms.transport.Transport
import frc.robot.mechanisms.transport.TransportIO
import org.wpilib.command3.Command
import org.wpilib.command3.Command.waitFor
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.units.Units
import java.sql.Time

class SimpleShoot(
    val shooter: Shooter,
    val transport: Transport
) : Command {
    override fun name(): String = "SimpleShoot"

    override fun requirements(): Set<Mechanism?> = setOf(shooter, transport)

    override fun run(coroutine: Coroutine) {

        while (true) {
            shooter.setVelocity(300.0)
            waitFor(Units.Seconds.of(0.35))
            transport.setVelocity(TransportIO.TransportState.SHOOTING.velocity)
            coroutine.yield()
        }
    }

    override fun onCancel() {
        shooter.disable()
        transport.disable()
    }
}
