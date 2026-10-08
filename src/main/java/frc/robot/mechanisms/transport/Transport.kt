package frc.robot.mechanisms.transport

import frc.robot.mechanisms.pivot.PivotIO
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.math.filter.Debouncer
import org.wpilib.util.Alert

class Transport (
    private val io: TransportIO,
): Mechanism {
    private val inputs = TransportIOInputsAutoLogged()

    // Connected debouncers
    private val motorConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)

    // Connection alerts
    private val alertType: String = "Transport Alerts"
    private val transportDisconnectedAlert: Alert

    init {
        transportDisconnectedAlert =
            Alert(alertType, "One of the transport motors is down oouuughhhhh", Alert.Level.MEDIUM)
    }

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Transport", inputs)

        transportDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.data.hopperConnected) || !motorConnectedDebouncer.calculate(inputs.data.indexerConnected))

        io.setTransportState(inputs.data.transportState)

//        if (DriverStation.isDisabled()) {
//            io.setPivotState(PivotIO.PivotPosition.STOW)
//        }
    }

}