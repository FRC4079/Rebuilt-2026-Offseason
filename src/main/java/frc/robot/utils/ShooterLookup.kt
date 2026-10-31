package frc.robot.utils

import org.wpilib.math.util.Units

data class ShotPoint(
    val distanceMeters: Double,
    val hoodAngleDegrees: Double,
    val flywheelRPM: Double,
)

object ShotTable {
    private val table =
        listOf(
            ShotPoint(1.2,720.0, Units.radiansPerSecondToRotationsPerMinute(300.0)), // Probably change to hoodAngleRad?
            ShotPoint(3.0, 850.0, Units.radiansPerSecondToRotationsPerMinute(350.0)),
            ShotPoint(5.0, 1450.0, Units.radiansPerSecondToRotationsPerMinute(380.0)),
            ShotPoint(7.0, 1720.0, Units.radiansPerSecondToRotationsPerMinute(400.0)),
            ShotPoint(9.0, 1850.0, Units.radiansPerSecondToRotationsPerMinute(450.0)),
        )

    fun getShot(distanceMeters: Double): ShotPoint? {
        if (!distanceMeters.isFinite()) return null

        if (distanceMeters < table.first().distanceMeters) return table.first()

        if (distanceMeters > table.last().distanceMeters) return table.last()

        for (i in 0 until table.lastIndex) {
            val a = table[i]
            val b = table[i + 1]

            if (distanceMeters <= b.distanceMeters) {
                val t =
                    (distanceMeters - a.distanceMeters) /
                        (b.distanceMeters - a.distanceMeters)

                return ShotPoint(
                    distanceMeters = distanceMeters,
                    hoodAngleDegrees =
                        a.hoodAngleDegrees +
                            t * (b.hoodAngleDegrees - a.hoodAngleDegrees),
                    flywheelRPM =
                        a.flywheelRPM +
                            t * (b.flywheelRPM - a.flywheelRPM),
                )
            }
        }

        return table.last()
    }
}
