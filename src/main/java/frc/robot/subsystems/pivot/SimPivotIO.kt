package frc.robot.subsystems.pivot

import frc.robot.subsystems.pivot.PivotIO
import frc.robot.utils.RobotParameters
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.DCMotorSim

class SimPivotIO(

) : PivotIO {
    private val pivotMotorModel: DCMotor = DCMotor.getKrakenX60Foc(1)

    private val pivotSim =
        DCMotorSim(
    Models.singleJointedArmFromPhysicalConstants(
                pivotMotorModel,
                0.25,
                RobotParameters.PivotParameters.PIVOT_GEAR_RATIO,
            ),
        pivotMotorModel,
        )


}