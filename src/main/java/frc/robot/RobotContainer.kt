package frc.robot

import frc.robot.commands.PadDrive
import frc.robot.mechanisms.OdometrySupplier
import frc.robot.mechanisms.Vision
import frc.robot.mechanisms.drive.Swerve
import frc.robot.mechanisms.drive.gyro.RealGyroIO
import frc.robot.mechanisms.drive.module.ModuleIO
import frc.robot.mechanisms.drive.module.RealModuleIO
import frc.robot.mechanisms.hood.Hood
import frc.robot.mechanisms.intake.IntakeIO
import frc.robot.mechanisms.pivot.Pivot
import frc.robot.mechanisms.shooter.Shooter
import frc.robot.mechanisms.transport.Transport
import frc.robot.utils.RobotParameters
import frc.robot.utils.enums.State
import org.wpilib.command3.Command
import org.wpilib.driverstation.XboxController
import org.wpilib.math.kinematics.Odometry

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    val pad: XboxController = XboxController(0)

    val moduleIOs: Array<ModuleIO> =
        RobotParameters.SwerveParameters.MODULE_CONFIGS
            .map { config -> RealModuleIO(config) }
            .toTypedArray()

    // CREATE ALL SUBSYSTEMS HERE

    val swerve: Swerve = Swerve(moduleIOs, RealGyroIO)
//    val intake: IntakeIO = IntakeIO()
//    val transport: Transport = Transport()
//    val shooter: Shooter = Shooter()
//    val pivot: Pivot = Pivot()

    // VIRTUAL SUBSYSTEMS
    val vision: Vision = Vision(RobotParameters.VisionParameters.CAMERAS)
    val odometrySupplier: OdometrySupplier = OdometrySupplier(swerve, vision)

    var autonomous: Command? = null

    /** The container for the robot. Contains subsystems, IO devices, and commands.  */
    init {
        swerve.defaultCommand = PadDrive(swerve, pad)
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
