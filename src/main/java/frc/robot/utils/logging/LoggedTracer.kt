package frc.robot.utils.logging

import org.littletonrobotics.junction.Logger
import org.wpilib.system.Timer

/** Utility class for logging code execution times.  */
object LoggedTracer {
    private var startTime = -1.0

    /** Reset the clock.  */
    fun reset() {
        startTime = Timer.getTimestamp()
    }

    /** Save the time elapsed since the last reset or record.  */
    fun record(epochName: String?) {
        val now: Double = Timer.getTimestamp()
        Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0)
        startTime = now
    }
}
