package frc.robot.utils

data class ShotPoint(
    val distanceMeters: Double,
    val hoodAngleDegrees: Double,
    val flywheelRPS: Double,
)

object ShotTable {
    private val table =
        listOf(
            ShotPoint(0.25,15.0,200.0), //TODO: Probably change to hoodAngleRad?
            ShotPoint(1.5, 20.0, 250.0),
            ShotPoint(2.0, 25.0, 280.0),
            ShotPoint(2.5, 30.0, 320.0),
            ShotPoint(3.0, 35.0, 300.0),
            ShotPoint(3.5, 38.0, 400.0),
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
                    flywheelRPS =
                        a.flywheelRPS +
                            t * (b.flywheelRPS - a.flywheelRPS),
                )
            }
        }

        return table.last()
    }
}
