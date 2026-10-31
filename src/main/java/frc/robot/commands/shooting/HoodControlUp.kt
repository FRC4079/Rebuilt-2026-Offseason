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

class HoodControlUp(
    val hood: Hood,
) : Command {
    override fun name(): String = "SimpleShoot"

    override fun requirements(): Set<Mechanism?> = setOf(hood)

    override fun run(coroutine: Coroutine) {
        var inputPlus: Double = 0.0

        while (true) {
            hood.setPosition(UnitUtil.degreesToRadians(720.0))
            coroutine.yield()
        }
    }

    override fun onCancel() {
        hood.setPosition(0.0)
    }
}
