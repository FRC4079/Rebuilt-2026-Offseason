package frc.robot.mechanisms.hood

import org.littletonrobotics.junction.Logger
import org.wpilib.math.filter.Debouncer
import org.wpilib.util.Alert

class Hood(
    private val io: HoodIO
) {
    private val inputs = HoodIOInputsAutoLogged()

    // Connected debouncer
    private val hoodMotorConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)

    // Connection alerts
    private val alertType: String = "Hood Alerts"
    private val hoodDisconnectedAlert: Alert

    init {
        hoodDisconnectedAlert =
            Alert(alertType, "Disconnected hood motor.", Alert.Level.MEDIUM)
    }

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Hood", inputs)

        hoodDisconnectedAlert.set(!hoodMotorConnectedDebouncer.calculate(inputs.data.hoodConnected))

        io.setHoodState(inputs.data.hoodPositionState)
    }

    fun setState(state: HoodIO.HoodPositionState) {
        inputs.data.hoodPositionState = state
    }

    fun setPower(power: Double) {
        io.setPower(power)
    }

    fun disable() {
        io.disablePower()
        inputs.data.hoodPositionState = HoodIO.HoodPositionState.HOME
    }
}