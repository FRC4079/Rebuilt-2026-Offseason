package frc.robot.utils.swerve

import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveModuleVelocity

data class SwerveSetpoint(
    var chassisSpeeds: ChassisVelocities? = null,
    var moduleStates: Array<SwerveModuleVelocity?>? = null,
) {
    init {
        this.chassisSpeeds = chassisSpeeds
        this.moduleStates = moduleStates
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as SwerveSetpoint

        if (chassisSpeeds != other.chassisSpeeds) return false
        if (!moduleStates.contentEquals(other.moduleStates)) return false

        return true
    }

    override fun hashCode(): Int {
        var result = chassisSpeeds?.hashCode() ?: 0
        result = 31 * result + (moduleStates?.contentHashCode() ?: 0)
        return result
    }
}
