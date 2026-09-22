package frc.robot.utils.phoenix

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.ffi.CANBusNative
import frc.robot.utils.RobotParameters.SwerveParameters
import org.wpilib.system.RobotController
import org.wpilib.units.Units
import org.wpilib.units.measure.Angle
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.ReentrantLock
import java.util.function.DoubleSupplier
import kotlin.concurrent.withLock

/**
 * High-frequency odometry sampling thread.
 *
 * SAFETY: This thread runs at NORMAL priority. It only reads CAN signals
 * and pushes doubles into queues. It never touches the CommandV3 scheduler.
 */
class PhoenixOdometryThread private constructor() : Thread("PhoenixOdometryThread") {
    companion object {
        private val isCANFD = true

        @Volatile
        private var instance: PhoenixOdometryThread? = null

        fun getInstance(): PhoenixOdometryThread =
            instance ?: synchronized(this) {
                instance ?: PhoenixOdometryThread().also { instance = it }
            }
    }

    init {
        isDaemon = true
    }

    /** Lock this when draining queues from the main thread. */
    val odometryLock = ReentrantLock()

    private val signalsLock = ReentrantLock()
    private var phoenixSignals = arrayOf<BaseStatusSignal>()
    private val genericSignals = mutableListOf<DoubleSupplier>()
    private val phoenixQueues = mutableListOf<ArrayBlockingQueue<Double>>()
    private val genericQueues = mutableListOf<ArrayBlockingQueue<Double>>()
    private val timestampQueues = mutableListOf<ArrayBlockingQueue<Double>>()

    override fun start() {
        if (timestampQueues.isNotEmpty() || phoenixSignals.isNotEmpty()) {
            super.start()
        }
    }

    fun registerSignal(signal: StatusSignal<Angle>): ArrayBlockingQueue<Double> {
        val queue = ArrayBlockingQueue<Double>(20)
        signalsLock.withLock {
            odometryLock.withLock {
                phoenixSignals = arrayOf(*phoenixSignals, signal)
                phoenixQueues.add(queue)
            }
        }
        return queue
    }

    fun registerSignal(signal: DoubleSupplier): ArrayBlockingQueue<Double> {
        val queue = ArrayBlockingQueue<Double>(20)
        signalsLock.withLock {
            odometryLock.withLock {
                genericSignals.add(signal)
                genericQueues.add(queue)
            }
        }
        return queue
    }

    fun makeTimestampQueue(): ArrayBlockingQueue<Double> {
        val queue = ArrayBlockingQueue<Double>(20)
        odometryLock.withLock {
            timestampQueues.add(queue)
        }
        return queue
    }

    override fun run() {
        // The original code used Threads.setCurrentThreadPriority(true, 1)
        // to minimize CAN sampling jitter. Safe only if main 20ms loop never overruns.
        // Very dangerous, minimal optimizing for a lot of potential risk

        // Threads.setCurrentThreadPriority(true, 1)

        while (!isInterrupted) {
            // Block until CAN data arrives (CANivore) or sleep (RIO CAN)
            signalsLock.withLock {
                try {
                    if (isCANFD && phoenixSignals.isNotEmpty()) {
                        BaseStatusSignal.waitForAll(
                            2.0 / SwerveParameters.OdometryConfig.ODOMETRY_FREQUENCY,
                            *phoenixSignals,
                        )
                    } else {
                        sleep((1000.0 / SwerveParameters.OdometryConfig.ODOMETRY_FREQUENCY).toLong())
                        if (phoenixSignals.isNotEmpty()) {
                            BaseStatusSignal.refreshAll(*phoenixSignals)
                        }
                    }
                } catch (e: InterruptedException) {
                    interrupt()
                    return
                }
            }

            // Push samples to queues. Drive subsystem locks odometryLock when draining.
            odometryLock.withLock {
                val timestamp = RobotController.getMeasureMonotonicTime().`in`(Units.Seconds)
                val avgLatency =
                    if (phoenixSignals.isNotEmpty()) {
                        phoenixSignals.sumOf { it.timestamp.latency } / phoenixSignals.size
                    } else {
                        0.0
                    }

                val adjustedTimestamp = timestamp - avgLatency

                for (i in phoenixSignals.indices) {
                    phoenixQueues[i].offer(phoenixSignals[i].valueAsDouble)
                }
                for (i in genericSignals.indices) {
                    genericQueues[i].offer(genericSignals[i].asDouble)
                }
                for (i in timestampQueues.indices) {
                    timestampQueues[i].offer(adjustedTimestamp)
                }
            }
        }
    }

    fun shutdown() {
        interrupt()
        join(100)
    }
}
