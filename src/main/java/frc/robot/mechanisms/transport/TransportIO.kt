package frc.robot.mechanisms.transport

import frc.robot.utils.logging.ReflectiveLoggableInputs
import org.littletonrobotics.junction.AutoLog

interface TransportIO {
    @AutoLog
    open class TransportIOInputs {
        @JvmField
        var data: TransportIOData =
            TransportIOData(
                hopperConnected = false,
                hopperPositionRad = 0.0,
                hopperVelocityRadPerSec = 0.0,
                hopperAppliedVolts = 0.0,
                hopperSupplyCurrentAmps = 0.0,
                hopperTorqueCurrentAmps = 0.0,

                indexerConnected = false,
                indexerPositionRad = 0.0,
                indexerVelocityRadPerSec = 0.0,
                indexerAppliedVolts = 0.0,
                indexerSupplyCurrentAmps = 0.0,
                indexerTorqueCurrentAmps = 0.0,

                transportState = TransportState.STOP,
            )
    }

    data class TransportIOData(
        var hopperConnected: Boolean,
        var hopperPositionRad: Double,
        var hopperVelocityRadPerSec: Double,
        var hopperAppliedVolts: Double,
        var hopperSupplyCurrentAmps: Double,
        var hopperTorqueCurrentAmps: Double,

        var indexerConnected: Boolean,
        var indexerPositionRad: Double,
        var indexerVelocityRadPerSec: Double,
        var indexerAppliedVolts: Double,
        var indexerSupplyCurrentAmps: Double,
        var indexerTorqueCurrentAmps: Double,

        var transportState: TransportState,
    ) : ReflectiveLoggableInputs()

    enum class TransportState(
        val velocity: Double,
    ) {
        ACTIVE(1.0), // stupid ass enum bro :sob:
        STOP(0.0),
    }

    /**
     * Updates the inputs for the transport subsystem. Controls all variable data for the IO.
     *
     * @param inputs Input data to be passed into transport logic
     */
    fun updateInputs(inputs: TransportIOInputs) {}

    fun setHopperPID(
        hopkP: Double,
        hopkI: Double,
        hopkD: Double,
    ) {}

    fun setIndexerPID(
        indkP: Double,
        indkI: Double,
        indkD: Double,
    ) {}

    /**
     * sets the power that will be applied to the motor
     *
     * @param power the power applied to the motor between -1 and 1.
     */
    fun setPower(power: Double) {}

    /**
     * sets the transport state, which sets position in periodic
     *
     * @param state the state to set the transport to
     */
    fun setTransportState(state: TransportState) {}

    /** Disables the motor  */
    fun disablePower() {}

    /**
     * sets the current limit for the transport
     *
     * @param currentLimit the maximum current limit
     */
    fun setCurrentLimit(currentLimit: Int) {}
}