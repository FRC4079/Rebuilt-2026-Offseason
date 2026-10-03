package frc.robot.utils.swerve

@JvmRecord
data class ModuleLimits(
    val maxDriveVelocity: Double,
    val maxDriveAcceleration: Double,
    val maxSteeringVelocity: Double,
) 
