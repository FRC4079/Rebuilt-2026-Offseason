package frc.robot.utils

import com.ctre.phoenix6.signals.InvertedValue
import com.limelightvision.Limelight
import frc.robot.utils.RobotParameters.CANBusParameters.SWERVE_CANBUS_ID
import org.wpilib.hardware.bus.CANPort
import org.wpilib.math.controller.PIDController
import org.wpilib.math.controller.SimpleMotorFeedforward
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.geometry.Translation3d
import org.wpilib.math.kinematics.SwerveDriveKinematics

/** Class containing global values for the robot.  */
@Suppress("PropertyName")
object RobotParameters {
    object GlobalParameters {
        val DISABLE_HAL: Boolean = false
        val TUNING_MODE: Boolean = false
        val MODE: Mode = Mode.REAL
    }

    enum class Mode { REAL, SIM }

    /** Class containing global values related to motors.  */
    object CANBusParameters {
        // Motor CAN ID Values
        const val FRONT_LEFT_STEER_ID: Int = 1
        const val FRONT_LEFT_DRIVE_ID: Int = 2
        const val FRONT_RIGHT_STEER_ID: Int = 3
        const val FRONT_RIGHT_DRIVE_ID: Int = 4
        const val BACK_LEFT_STEER_ID: Int = 5
        const val BACK_LEFT_DRIVE_ID: Int = 6
        const val BACK_RIGHT_STEER_ID: Int = 7
        const val BACK_RIGHT_DRIVE_ID: Int = 8
        const val FRONT_LEFT_CAN_CODER_ID: Int = 9
        const val FRONT_RIGHT_CAN_CODER_ID: Int = 10
        const val BACK_LEFT_CAN_CODER_ID: Int = 11
        const val BACK_RIGHT_CAN_CODER_ID: Int = 12
        const val PIDGEY_ID: Int = 13
        const val INTAKE_LEFT_ID: Int = 14
        const val INTAKE_RIGHT_ID: Int = 41
        const val INTAKE_PIVOT_MOTOR_ID: Int = 15
        const val SHOOTER_LEFT_MOTOR_ID: Int = 16
        const val SHOOTER_RIGHT_MOTOR_ID: Int = 17
        const val INDEXER_MOTOR_ID: Int = 18
        const val TRANSPORT_MOTOR_ID: Int = 19
        const val HOOD_MOTOR_ID: Int = 20

        val SWERVE_CANBUS_ID: CANPort = CANPort.CAN_S0
    }

    object IntakeParameters {
        val intakeMotorLeft: Int = CANBusParameters.INTAKE_LEFT_ID
        val intakeMotorRight: Int = CANBusParameters.INTAKE_RIGHT_ID
        val INTAKE_CANPORT: CANPort = CANPort.CAN_S1

        object PhysicalParameters {
            const val INTAKE_GEAR_RATIO: Double = 1.0
            const val INTAKE_CURRENT_LIMIT: Double = 40.0
        }

        object PIDParameters {
            val INTAKE_PID = PIDController(5.0, 0.0, 0.0)
        }
    }

    object PivotParameters {
        val pivotMotor: Int = CANBusParameters.INTAKE_PIVOT_MOTOR_ID
        val PIVOT_GEAR_RATIO: Double = 1.0
        val PIVOT_CANPORT: CANPort = CANPort.CAN_S1

        object PhysicalParameters {
            const val PIVOT_CURRENT_LIMIT: Double = 40.0
        }

        object PIDParameters {
            val PIVOT_PID = PIDController(5.0, 0.0, 0.0)
        }
    }

    object ShooterParameters {
        val shooterLeftMotor: Int = CANBusParameters.SHOOTER_LEFT_MOTOR_ID
        val shooterRightMotor: Int = CANBusParameters.SHOOTER_RIGHT_MOTOR_ID
        val SHOOTER_GEAR_RATIO: Double = 1.0
        val SHOOTER_CANPORT: CANPort = CANPort.CAN_S1

        object PhysicalParameters {
            const val SHOOTER_CURRENT_LIMIT: Double = 40.0
        }

        object PIDParameters {
            val SHOOTER_PID = PIDController(5.0, 0.0, 0.0)
        }
    }

    object HoodParameters {
        val hoodMotor: Int = CANBusParameters.HOOD_MOTOR_ID
        val HOOD_GEAR_RATIO: Double = 1.0
        val HOOD_CANPORT: CANPort = CANPort.CAN_S1

        object PhysicalParameters {
            const val HOOD_CURRENT_LIMIT: Double = 40.0
        }

        object PIDParameters {
            val HOOD_PID = PIDController(5.0, 0.0, 0.0)
        }
    }

    object TransportParameters {
        var indexerMotor: Int = CANBusParameters.INDEXER_MOTOR_ID
        val INDEXER_GEAR_RATIO: Double = 1.0
        val hopperMotor: Int = CANBusParameters.TRANSPORT_MOTOR_ID
        val HOPPER_GEAR_RATIO: Double = 1.0
        val HOPPER_CANPORT: CANPort = CANPort.CAN_S1
        val INDEXER_CANPORT: CANPort = CANPort.CAN_S1

        object PhysicalParameters {
            const val TRANSPORT_CURRENT_LIMIT: Double = 40.0
        }

        object PIDParameters {
            val INDEXER_PID = PIDController(5.0, 0.0, 0.0)
            val HOPPER_PID = PIDController(5.0, 0.0, 0.0)
        }
    }

    /** Class containing global values related to the swerve drive system.  */
    object SwerveParameters {
        data object OdometryConfig {
            const val ODOMETRY_FREQUENCY: Double = 250.0
        }

        fun getMode(): Mode = GlobalParameters.MODE

        data class ModuleConfig(
            val driveMotorId: Int,
            val turnMotorId: Int,
            val encoderID: Int,
            val encoderOffset: Rotation2d,
            val turnInverted: Boolean,
            val driveInverted: Boolean,
            val encoderInverted: Boolean,
            val canBUS: CANPort,
        )

        val MODULE_CONFIGS: Array<ModuleConfig> =
            arrayOf(
                // FL (kInvertLeftSide = false)
                ModuleConfig(
                    driveMotorId = CANBusParameters.FRONT_LEFT_DRIVE_ID,
                    turnMotorId = CANBusParameters.FRONT_LEFT_STEER_ID,
                    encoderID = CANBusParameters.FRONT_LEFT_CAN_CODER_ID,
                    encoderOffset = Rotation2d.fromRotations(0.247802734375),
                    turnInverted = false,
                    driveInverted = false,
                    encoderInverted = false,
                    canBUS = SWERVE_CANBUS_ID,
                ),
                // FR (kInvertRightSide = true)
                ModuleConfig(
                    driveMotorId = CANBusParameters.FRONT_RIGHT_DRIVE_ID,
                    turnMotorId = CANBusParameters.FRONT_RIGHT_STEER_ID,
                    encoderID = CANBusParameters.FRONT_RIGHT_CAN_CODER_ID,
                    encoderOffset = Rotation2d.fromRotations(-0.34326171875),
                    turnInverted = false,
                    driveInverted = true,
                    encoderInverted = false,
                    canBUS = SWERVE_CANBUS_ID,
                ),
                // BL (kInvertLeftSide = false)
                ModuleConfig(
                    driveMotorId = CANBusParameters.BACK_LEFT_DRIVE_ID,
                    turnMotorId = CANBusParameters.BACK_LEFT_STEER_ID,
                    encoderID = CANBusParameters.BACK_LEFT_CAN_CODER_ID,
                    encoderOffset = Rotation2d.fromRotations(0.450927734375),
                    turnInverted = false,
                    driveInverted = false,
                    encoderInverted = false,
                    canBUS = SWERVE_CANBUS_ID,
                ),
                // BR (kInvertRightSide = true)
                ModuleConfig(
                    driveMotorId = CANBusParameters.BACK_RIGHT_DRIVE_ID,
                    turnMotorId = CANBusParameters.BACK_RIGHT_STEER_ID,
                    encoderID = CANBusParameters.BACK_RIGHT_CAN_CODER_ID,
                    encoderOffset = Rotation2d.fromRotations(-0.032958984375),
                    turnInverted = false,
                    driveInverted = true,
                    encoderInverted = false,
                    canBUS = SWERVE_CANBUS_ID,
                ),
            )

        /** Class containing PID constants for the swerve drive system.
         *
         * NOTE: these gains are in TalonFX/CTRE units (volts, mechanism rotations),
         * taken directly from the Tuner X project. They are NOT WPILib PIDController
         * gains — if you ever run PID on the Rio instead of the Talons, retune them.
         */
        object PIDParameters {
            @JvmField
            val STEER_FF = SimpleMotorFeedforward(0.1, 2.49)

            @JvmField
            val DRIVE_FF = SimpleMotorFeedforward(0.0, 0.124)

            @JvmField
            val STEER_PID_TELE = PIDController(45.0, 0.0, 2.0)

            @JvmField
            val STEER_PID_AUTO = PIDController(45.0, 0.0, 2.0)

            @JvmField
            val DRIVE_PID_AUTO = PIDController(100.0, 0.0, 0.0)

            @JvmField
            val DRIVE_PID_TELE = PIDController(100.0, 0.0, 0.0)

            @JvmField
            val ROTATIONAL_PID = PIDController(1.0, 0.0, 0.0)

            @JvmField
            val Y_PID = PIDController(0.2, 0.0, 0.0)

            @JvmField
            val DIST_PID: PIDController = PIDController(0.2, 0.0, 0.0)

            val PASS_ROTATIONAL_PID: PIDController = PIDController(0.1, 0.0, 0.0)
        }

        /** Class containing physical dimensions and kinematics for the swerve drive system.  */
        object PhysicalParameters {
            const val ROBOT_SIZE: Double = 0.43105229381

            // 9.475 inches in meters (from Tuner X)
            private const val MODULE_POS: Double = 0.240665
            private val FRONT_LEFT: Translation2d = Translation2d(MODULE_POS, MODULE_POS)
            private val FRONT_RIGHT: Translation2d = Translation2d(MODULE_POS, -MODULE_POS)
            private val BACK_LEFT: Translation2d = Translation2d(-MODULE_POS, MODULE_POS)
            private val BACK_RIGHT: Translation2d = Translation2d(-MODULE_POS, -MODULE_POS)
            val MODULE_LOCATIONS: Array<Translation2d> = arrayOf(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT)

            // Motor Property Values
            // Measured speed at 12V from Tuner X (not theoretical free speed)
            const val MAX_SPEED: Double = 4.39
            const val MAX_ANGULAR_SPEED: Double = (14 * Math.PI) / 3
            const val STEER_MOTOR_GEAR_RATIO: Double = 287.0 / 11.0
            const val DRIVE_MOTOR_GEAR_RATIO: Double = 7.03125
            const val WHEEL_DIAMETER: Double = 0.1016
            const val METERS_PER_REV: Double = WHEEL_DIAMETER * Math.PI * 0.975

            // Every 1 rotation of the azimuth = kCoupleRatio drive motor turns
            const val COUPLE_RATIO: Double = 4.5

            // Limit Values (from Tuner X: 120A slip, 60A steer stator)
            const val DRIVE_CURRENT_LIMIT: Double = 120.0
            const val STEER_CURRENT_LIMIT: Double = 60.0

            @JvmField
            val kinematics: SwerveDriveKinematics =
                SwerveDriveKinematics(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT)
        }

        /** Class containing various thresholds and constants for the swerve drive system.  */
        object Thresholds {
            const val STATE_SPEED_THRESHOLD: Double = 0.05

            const val TURN_DEADBAND_DEGREES: Double = 0.3

            const val JOYSTICK_DEADBAND: Double = 0.05
            const val USING_VISION: Boolean = false
            const val AUTO_ALIGN: Boolean = false
            const val MOTOR_DEADBAND: Double = 0.05
            const val IS_FIELD_ORIENTED: Boolean = true
            const val SHOULD_INVERT: Boolean = false
            const val ENCODER_OFFSET: Double = (0 / 360.0)
            const val X_DEADZONE: Double = 0.15
            const val Y_DEADZONE: Double = 0.15
            const val OFF_BALANCE_ANGLE_THRESHOLD: Double = 10.0
            const val ON_BALANCE_ANGLE_THRESHOLD: Double = 5.0

            // Testing boolean for logging (to not slow down the robot)
            const val TEST_MODE: Boolean = true
        }
    }

    /** Class containing constants for the Vision subsystem.  */
    object VisionParameters {
        val CAMERAS: Array<Limelight> =
            arrayOf(
                Limelight(
                    "limelight-shooter",
                    Pose3d(
                        Translation3d(
                            0.116557,
                            -0.009852,
                            0.410349,
                        ),
                        Rotation3d(
                            180.0,
                            0.0,
                            0.0,
                        ),
                    ),
                ),
                Limelight(
                    "limelight-indexer",
                    Pose3d(
                        Translation3d(
                            0.216760,
                            -0.191,
                            0.336850,
                        ),
                        Rotation3d(
                            0.0,
                            0.0,
                            0.0,
                        ),
                    ),
                ),
            )
    }

    object PositionConstants {
        const val DEPLOY_INTAKE_POSITION = 0.0
        const val RETRACT_INTAKE_POSITION = 0.0
    }
}
