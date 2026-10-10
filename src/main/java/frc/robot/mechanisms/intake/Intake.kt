package frc.robot.mechanisms.intake

import org.littletonrobotics.junction.Logger
import org.wpilib.math.filter.Debouncer
import org.wpilib.util.Alert

class Intake(
    private val io: IntakeIO,
) {
    private val inputs = IntakeIOInputsAutoLogged()

    // Connected debouncer
    private val intakeMotorConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)

    // Connection alerts
    private val alertType: String = "Intake Alerts"
    private val intakeDisconnectedAlert: Alert

    init {
        intakeDisconnectedAlert =
            Alert(alertType, "Disconnected intake motor.", Alert.Level.MEDIUM)
    }

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)

        intakeDisconnectedAlert.set(!intakeMotorConnectedDebouncer.calculate(inputs.data.intakeConnected))
    }

    fun setVoltage(volts: Double) {
        io.setVoltage(volts)
    }

    fun setVelocity(velocityRadPerSec: Double) {
        io.setVelocity(velocityRadPerSec)
    }

    fun stop() {
        io.stop()
    }
}
