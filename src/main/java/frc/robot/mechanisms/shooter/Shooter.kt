package frc.robot.mechanisms.shooter

import frc.robot.utils.enums.State
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.math.filter.Debouncer
import org.wpilib.util.Alert

class Shooter(
    private val io: ShooterIO,
) : Mechanism {
    private val inputs = ShooterIOInputsAutoLogged()
    private var targetVelocityRadPerSec: Double? = null

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

        val target = targetVelocityRadPerSec
        if (target != null) {
            io.setVelocity(target)
        } else {
            io.setShooterState(inputs.data.shooterState)
        }
    }

    fun setState(state: State) {
        targetVelocityRadPerSec = null
//        inputs.data.shooterState = state
    }

    fun setVelocity(velocityRadPerSec: Double) {
        targetVelocityRadPerSec = velocityRadPerSec
        io.setVelocity(velocityRadPerSec)
    }

    fun setPower(
        topPower: Double,
        bottomPower: Double,
    ) {
        io.setPower(topPower, bottomPower)
    }

    fun disable() {
        targetVelocityRadPerSec = null
        io.disablePower()
        inputs.data.shooterState = ShooterIO.ShooterState.IDLE
    }
}
