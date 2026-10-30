package frc.robot.mechanisms

import frc.robot.mechanisms.drive.Swerve
import frc.robot.mechanisms.hood.Hood
import frc.robot.mechanisms.transport.Transport
import frc.robot.mechanisms.pivot.Pivot
import frc.robot.mechanisms.intake.Intake
import frc.robot.mechanisms.shooter.Shooter
import frc.robot.mechanisms.virtual.OdometrySupplier
import frc.robot.mechanisms.virtual.Vision
import org.wpilib.math.kinematics.Odometry

class SuperSystem(
    val swerve: Swerve,
    val hood: Hood,
    val intake: Intake,
    val pivot: Pivot,
    val shooter: Shooter,
    val transport: Transport,
    val odometrySupplier: OdometrySupplier,
    val vision: Vision,
    ) {

    fun periodic() {
        swerve.periodic()
        hood.periodic()
        intake.periodic()
        pivot.periodic()
        shooter.periodic()
        transport.periodic()
        odometrySupplier.periodic()
    }

}