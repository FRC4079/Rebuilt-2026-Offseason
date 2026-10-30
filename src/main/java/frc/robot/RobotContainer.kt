package frc.robot

import frc.robot.commands.TransportTest
import frc.robot.commands.driving.PadDrive
import frc.robot.commands.driving.ResetGyro
import frc.robot.commands.manipulating.DeployIntake
import frc.robot.commands.manipulating.simple.RetractIntake
import frc.robot.commands.manipulating.simple.SpinIntake
import frc.robot.commands.shooting.SimpleShoot
import frc.robot.mechanisms.SuperSystem
import frc.robot.mechanisms.virtual.OdometrySupplier
import frc.robot.mechanisms.virtual.Vision
import frc.robot.mechanisms.drive.Swerve
import frc.robot.mechanisms.drive.gyro.RealGyroIO
import frc.robot.mechanisms.drive.module.ModuleIO
import frc.robot.mechanisms.drive.module.RealModuleIO
import frc.robot.mechanisms.hood.Hood
import frc.robot.mechanisms.hood.HoodIO
import frc.robot.mechanisms.hood.RealHoodIO
import frc.robot.mechanisms.intake.Intake
import frc.robot.mechanisms.intake.IntakeIO
import frc.robot.mechanisms.intake.RealIntakeIO
import frc.robot.mechanisms.pivot.Pivot
import frc.robot.mechanisms.pivot.PivotIO
import frc.robot.mechanisms.pivot.RealPivotIO
import frc.robot.mechanisms.shooter.RealShooterIO
import frc.robot.mechanisms.shooter.Shooter
import frc.robot.mechanisms.shooter.ShooterIO
import frc.robot.mechanisms.transport.RealTransportIO
import frc.robot.mechanisms.transport.Transport
import frc.robot.mechanisms.transport.TransportIO
import frc.robot.utils.RobotParameters
import org.wpilib.command3.Command
import org.wpilib.command3.button.CommandXboxController

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    val pad: CommandXboxController = CommandXboxController(0)

    // CREATE IOS

    val moduleIOs: Array<ModuleIO> =
        RobotParameters.SwerveParameters.MODULE_CONFIGS
            .map { config -> RealModuleIO(config) }
            .toTypedArray()

    val intakeIO: IntakeIO = RealIntakeIO()
    val pivotIO: PivotIO = RealPivotIO()
    val shooterIO: ShooterIO = RealShooterIO()
    val transportIO: TransportIO = RealTransportIO()
    val hoodIO: HoodIO = RealHoodIO()

    // CREATE ALL SUBSYSTEMS HERE

    val swerve: Swerve = Swerve(moduleIOs, RealGyroIO)
    val intake: Intake = Intake(intakeIO)
    val pivot: Pivot = Pivot(pivotIO)
    val shooter: Shooter = Shooter(shooterIO)
    val transport: Transport = Transport(transportIO)
    val hood: Hood = Hood(hoodIO)

    // VIRTUAL SUBSYSTEMS
    val vision: Vision = Vision(RobotParameters.VisionParameters.CAMERAS)
    val odometrySupplier: OdometrySupplier = OdometrySupplier(swerve, vision)

//    val superSystem: SuperSystem = SuperSystem(
//        swerve,
//        hood,
//        intake,
//        pivot,
//        shooter,
//        transport,
//        odometrySupplier,
//        vision,
//    )

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
        pad.a().whileTrue(SpinIntake(intake))
        pad.b().onTrue(ResetGyro(swerve))
//        pad.x().whileTrue(ShootWithDistance(shooter, hood, odometrySupplier))
        pad.x().onTrue(RetractIntake(pivot))
        pad.y().whileTrue(DeployIntake(pivot, transport, intake))
        pad.rightBumper().whileTrue(TransportTest(transport))
        pad.leftBumper().whileTrue(SimpleShoot(shooter, transport))
    }

    val autonomousCommand: Command?
        get() = autonomous
}

