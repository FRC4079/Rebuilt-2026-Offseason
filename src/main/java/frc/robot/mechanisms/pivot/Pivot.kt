package frc.robot.mechanisms.pivot

import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.math.filter.Debouncer
import org.wpilib.util.Alert
// import edu.wpi.first.wpilibj.DriverStation;

class Pivot(
    private val io: PivotIO,
): Mechanism {
    private val inputs = PivotIOInputsAutoLogged()

    // Connected debouncers
    private val pivotMotorConnectedDebouncer: Debouncer = Debouncer(0.5, Debouncer.DebounceType.FALLING)

    // Connection alerts
    private val alertType: String = "Pivot Alerts"
    private val pivotDisconnectedAlert: Alert

    init {
        pivotDisconnectedAlert =
            Alert(alertType, "Disconnected pivot motor.", Alert.Level.MEDIUM)
    }

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Pivot", inputs)

        pivotDisconnectedAlert.set(!pivotMotorConnectedDebouncer.calculate(inputs.data.pivotConnected))

        io.setPivotState(inputs.data.pivotPositionState)

//        if (DriverStation.isDisabled()) {
//            io.setPivotState(PivotIO.PivotPosition.STOW)
//        }
    }
}
