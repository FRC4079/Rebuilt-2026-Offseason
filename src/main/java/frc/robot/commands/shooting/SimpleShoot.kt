package frc.robot.commands.shooting

import frc.robot.mechanisms.hood.Hood
import frc.robot.mechanisms.shooter.Shooter
import frc.robot.mechanisms.transport.Transport
import frc.robot.mechanisms.transport.TransportIO
import org.wpilib.command3.Command
import org.wpilib.command3.Command.waitFor
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.units.Units
import org.wpilib.math.util.Units as UnitUtil
import java.sql.Time

class SimpleShoot(
    val shooter: Shooter,
    val transport: Transport,
    val hood: Hood,
) : Command {
    override fun name(): String = "SimpleShoot"

    override fun requirements(): Set<Mechanism?> = setOf(shooter, transport, hood)

    override fun run(coroutine: Coroutine) {
        shooter.setVelocity(300.0)
        hood.setPosition(UnitUtil.degreesToRadians(0.0))
        waitFor(Units.Seconds.of(0.7))
        while (true) {
            shooter.setVelocity(450.0)
            hood.setPosition(UnitUtil.degreesToRadians(1850.0))
            transport.setVelocity(TransportIO.TransportState.SHOOTING.velocityRadPerSec)
            coroutine.yield()
        }
    }

    override fun onCancel() {
        shooter.disable()
        hood.setPosition(0.0)
        transport.disable()
    }
}
