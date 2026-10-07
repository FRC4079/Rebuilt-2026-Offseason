package frc.robot.mechanisms.shooter

import org.littletonrobotics.junction.Logger
import org.wpilib.math.filter.Debouncer
import org.wpilib.util.Alert

class Shooter(
    private val io: ShooterIO,
) {
    private val inputs = ShooterIOInputsAutoLogged()

    // Connected debouncers
    private val topMotorConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)
    private val bottomMotorConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)

    // Connection alerts
    private val alertType: String = "Shooter Alerts"
    private val topDisconnectedAlert: Alert = Alert(alertType, "Disconnected top shooter motor.", Alert.Level.MEDIUM)
    private val bottomDisconnectedAlert: Alert = Alert(alertType, "Disconnected bottom shooter motor.", Alert.Level.MEDIUM)

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)

        topDisconnectedAlert.set(!topMotorConnectedDebouncer.calculate(inputs.data.topMotorConnected))
        bottomDisconnectedAlert.set(!bottomMotorConnectedDebouncer.calculate(inputs.data.bottomMotorConnected))

        io.setShooterState(inputs.data.shooterState)
    }

    fun setState(state: ShooterIO.ShooterState) {
        inputs.data.shooterState = state
    }

    fun setPower(
        topPower: Double,
        bottomPower: Double,
    ) {
        io.setPower(topPower, bottomPower)
    }

    fun disable() {
        io.disablePower()
        inputs.data.shooterState = ShooterIO.ShooterState.IDLE
    }
}
