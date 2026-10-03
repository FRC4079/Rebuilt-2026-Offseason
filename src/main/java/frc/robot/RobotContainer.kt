package frc.robot

import org.wpilib.command3.Command
import org.wpilib.driverstation.Gamepad

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    val pad: Gamepad = Gamepad(0)

    var autonomous: Command? = null

    /** The container for the robot. Contains subsystems, IO devices, and commands.  */
    init {
//        Swerve.defaultCommand = drive(pad)
        configureBindings()
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger] or our [JoystickButton] constructor with an arbitrary predicate, or via
     * the named factories in [CommandGenericHID]'s subclasses for [ ]/[CommandPS4Controller] controllers or [CommandJoystick].
     */
    private fun configureBindings() {
    }

    val autonomousCommand: Command?
        get() = autonomous
}
