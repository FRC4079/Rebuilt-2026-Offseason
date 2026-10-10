package frc.robot.utils

data class ShotPoint(
    val distanceMeters: Double,
    val hoodAngleDegrees: Double,
    val flywheelRPM: Double,
)

object ShotTable {
    private val table =
        listOf(
            ShotPoint(1.5, 20.0, 2500.0),
            ShotPoint(2.0, 25.0, 2800.0),
            ShotPoint(2.5, 30.0, 3200.0),
            ShotPoint(3.0, 35.0, 3600.0),
            ShotPoint(3.5, 38.0, 4000.0),
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
