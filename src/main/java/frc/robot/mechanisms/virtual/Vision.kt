package frc.robot.mechanisms.virtual

import com.limelightvision.Limelight
import org.wpilib.command3.Mechanism

class Vision(
    val cameras: Array<Limelight>,
) : Mechanism
