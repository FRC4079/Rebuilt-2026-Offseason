package frc.robot.commands

import frc.robot.mechanisms.OdometrySupplier
import frc.robot.mechanisms.hood.Hood
import frc.robot.mechanisms.shooter.Shooter
import frc.robot.utils.ShotPoint
import frc.robot.utils.ShotTable
import frc.robot.utils.enums.GlobalState
import frc.robot.utils.enums.State
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.math.util.Units

/**
 * Spins up the shooter and sets the hood angle based on the robot's current distance to the hub. The
 * flywheel speed and hood angle come from the distance lookup in [ShotTable]. Runs continuously until
 * interrupted.
 */
class Shoot(
    private val shooter: Shooter,
    private val hood: Hood,
    private val odometrySupplier: OdometrySupplier,
) : Command {

    override fun name(): String = "Shoot"

    override fun requirements(): Set<Mechanism> = setOf(shooter, hood)

    override fun run(coroutine: Coroutine) {
        while (true) {
            val distanceMeters = odometrySupplier.getDistanceToHub()
            val shot: ShotPoint? = ShotTable.getShot(distanceMeters)

            if (shot == null) {
                coroutine.yield()
                continue
            }

            GlobalState.state = State.SHOOT

            shooter.setState(State.SHOOT)

            shooter.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(shot.flywheelRPM))
            hood.setPosition(Units.degreesToRadians(shot.hoodAngleDegrees))

            Logger.recordOutput("Shoot/DistanceMeters", shot.distanceMeters)
            Logger.recordOutput("Shoot/HoodAngleDegrees", shot.hoodAngleDegrees)
            Logger.recordOutput("Shoot/FlywheelRPM", shot.flywheelRPM)

            coroutine.yield()
        }
    }

    override fun onCancel() {
        shooter.disable()
        hood.disable()
    }
}
