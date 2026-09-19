package frc.robot

import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.wpilib.command3.Command
import org.wpilib.command3.Scheduler
import org.wpilib.command3.SchedulerEvent
import org.wpilib.hardware.bus.CANPort
import org.wpilib.hardware.power.PowerDistribution

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null

    /**
     * Use for any initialization code.
     */
    init {
        Logger.recordMetadata("Reefscape", "Logging") // Set a metadata value

        if (isReal()) {
            // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            // WARNING: PowerDistribution resource leak
            PowerDistribution(CANPort.CAN_D0, 1, PowerDistribution.ModuleType.REV) // Enables power distribution logging
        } else {
            setUseTiming(false) // Run as fast as possible
            val logPath =
                LogFileUtil
                    .findReplayLog() // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
            Logger.addDataReceiver(
                WPILOGWriter(
                    LogFileUtil.addPathSuffix(logPath, "_sim"),
                ),
            ) // Save outputs to a new log
        }

        Logger.start()
        robotContainer = RobotContainer()
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        Scheduler.getDefault().run()
    }

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        autonomousCommand = robotContainer!!.autonomousCommand
        Scheduler.getDefault().schedule(autonomousCommand!!)
    }

    /** This function is called once when teleop mode is initialized.  */
    override fun teleopInit() {
        if (autonomousCommand != null) {
            Scheduler.getDefault().cancel(autonomousCommand!!)
        }
    }
}
