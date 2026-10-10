package frc.robot.mechanisms.hood

import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.math.filter.Debouncer
import org.wpilib.util.Alert

class Hood(
    private val io: HoodIO
) : Mechanism {
    private val inputs = HoodIOInputsAutoLogged()
    private var targetPositionRad: Double? = null

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

        val target = targetPositionRad
        if (target != null) {
            io.setPosition(target)
        } else {
            io.setHoodState(inputs.data.hoodPositionState)
        }
    }

    fun setState(state: HoodIO.HoodPositionState) {
        targetPositionRad = null
        inputs.data.hoodPositionState = state
    }

    fun setPosition(positionRad: Double) {
        targetPositionRad = positionRad
        io.setPosition(positionRad)
    }

    fun setPower(power: Double) {
        io.setPower(power)
    }

    fun disable() {
        targetPositionRad = null
        io.disablePower()
        inputs.data.hoodPositionState = HoodIO.HoodPositionState.HOME
    }
}