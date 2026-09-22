package frc.robot.utils.phoenix

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import java.util.function.Supplier

object PhoenixUtils {
    /**
     * Tries to execute the given command until it returns an OK status.
     *
     * @param maxAttempts The maximum number of attempts to make.
     * @param command The command to execute.
     */
    fun tryUntilOk(
        maxAttempts: Int,
        command: Supplier<StatusCode>,
    ) {
        for (i in 0..<maxAttempts) {
            val error = command.get()
            if (error.isOK) break
        }
    }

    /** Signals for synchronized refresh.  */
    var canivoreSignals: Array<BaseStatusSignal?> = arrayOfNulls<BaseStatusSignal>(0)
    var rioSignals: Array<BaseStatusSignal?> = arrayOfNulls<BaseStatusSignal>(0)

    /** Registers a set of signals for synchronized refresh.  */
    fun registerSignals(
        canivore: Boolean,
        vararg signals: BaseStatusSignal?,
    ) {
        if (canivore) {
            val newSignals = arrayOfNulls<BaseStatusSignal>(canivoreSignals.size + signals.size)
            System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.size)
            System.arraycopy(signals, 0, newSignals, canivoreSignals.size, signals.size)
            canivoreSignals = newSignals
        } else {
            val newSignals = arrayOfNulls<BaseStatusSignal>(rioSignals.size + signals.size)
            System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.size)
            System.arraycopy(signals, 0, newSignals, rioSignals.size, signals.size)
            rioSignals = newSignals
        }
    }

    /** Refresh all registered signals.  */
    fun refreshAll() {
        if (canivoreSignals.isNotEmpty()) {
            BaseStatusSignal.refreshAll(*canivoreSignals)
        }
        if (rioSignals.isNotEmpty()) {
            BaseStatusSignal.refreshAll(*rioSignals)
        }
    }
}
