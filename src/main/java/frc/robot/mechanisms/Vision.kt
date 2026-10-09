package frc.robot.mechanisms

import com.limelightvision.Limelight
import com.limelightvision.PoseEstimateType
import org.wpilib.command3.Mechanism
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.linalg.Matrix
import org.wpilib.math.numbers.N1
import org.wpilib.math.numbers.N3
import java.util.ArrayDeque
import java.util.Queue

class Vision(
    val cameras: Array<Limelight>,
) : Mechanism
