package frc.robot.utils.swerve

import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveModuleVelocity

data class SwerveSetpoint(
    var chassisVelocities: ChassisVelocities,
    var moduleStates: Array<SwerveModuleVelocity>,
) {
    init {
        this.chassisVelocities = chassisVelocities
        this.moduleStates = moduleStates
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as SwerveSetpoint

        if (chassisVelocities != other.chassisVelocities) return false
        if (!moduleStates.contentEquals(other.moduleStates)) return false

        return true
    }

    override fun hashCode(): Int {
        var result = chassisVelocities.hashCode()
        result = 31 * result + (moduleStates.contentHashCode())
        return result
    }
}
