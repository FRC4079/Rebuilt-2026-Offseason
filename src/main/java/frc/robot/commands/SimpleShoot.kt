package frc.robot.commands

import frc.robot.mechanisms.shooter.Shooter
import frc.robot.mechanisms.shooter.ShooterIO
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class SimpleShoot(val shooter: Shooter) : Command {

    override fun name(): String = "SimpleShoot"

    override fun requirements(): Set<Mechanism?> = setOf(shooter)

    override fun run(coroutine: Coroutine) {
        while (true) {
            shooter.setVelocity(600.0)
            coroutine.yield()
        }
    }
    override fun onCancel() {
        shooter.setVelocity(0.0)
    }
}