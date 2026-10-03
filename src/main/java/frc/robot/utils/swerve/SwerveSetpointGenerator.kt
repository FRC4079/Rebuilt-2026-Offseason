package frc.robot.utils.swerve

import frc.robot.utils.math.EqualsUtil.GeomExtensions.epsilonEquals
import frc.robot.utils.math.EqualsUtil.epsilonEquals
import frc.robot.utils.math.GeomUtil
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.geometry.Twist2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveDriveKinematics
import org.wpilib.math.kinematics.SwerveModuleVelocity
import java.util.Optional
import kotlin.collections.get
import kotlin.math.*

/**
 * Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path
 * follower), and outputs a new setpoint that respects all the kinematic constraints on module
 * rotation speed and wheel velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 */

open class SwerveSetpointGenerator(
    private val kinematics: SwerveDriveKinematics,
    private val moduleLocations: Array<Translation2d>,
) {
    /**
     * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
     * direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state (i.e.
     * prev.inverse().rotateBy(goal)).
     * @return True if the shortest path to achieve this rotation involves flipping the drive
     * direction.
     */
    private fun flipHeading(prevToGoal: Rotation2d): Boolean = abs(prevToGoal.radians) > Math.PI / 2.0

    private fun unwrapAngle(
        ref: Double,
        angle: Double,
    ): Double {
        val diff = angle - ref
        return if (diff > Math.PI) {
            angle - 2.0 * Math.PI
        } else if (diff < -Math.PI) {
            angle + 2.0 * Math.PI
        } else {
            angle
        }
    }

    private fun interface Function2d {
        fun f(
            x: Double,
            y: Double,
        ): Double
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
     * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
     * while being robust in ways that e.g. the Newton-Raphson method isn't.
     *
     * @param func The Function 2d to take the root of.
     * @param point0 [Triple] Containing the function, x value, and y value of the lower bracket.
     * - 'first': [Double] value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during recursion)
     * - 'second': [Double] x value of the lower bracket.
     * - 'third': [Double] y value of the lower bracket.
     * @param point1 [Triple] Containing the function, x value, and y value of the upper bracket.
     * - 'first': [Double] value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during recursion)
     * - 'second': [Double] x value of the upper bracket.
     * - 'third': [Double] y value of the upper bracket.

     * @param iterationsLeft Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
     * (approximate) root.
     */
    private fun findRoot(
        func: Function2d,
        point0: Triple<Double, Double, Double>,
        point1: Triple<Double, Double, Double>,
        iterationsLeft: Int,
    ): Double {
        val f0 = point0.first
        val x0 = point0.second
        val y0 = point0.third
        val f1 = point1.first
        val x1 = point1.second
        val y1 = point1.third

        if (iterationsLeft < 0 || epsilonEquals(f0, f1)) {
            return 1.0
        }

        val sGuess = max(0.0, min(1.0, -f0 / (f1 - f0)))
        val xGuess = (x1 - x0) * sGuess + x0
        val yGuess = (y1 - y0) * sGuess + y0
        val fGuess = func.f(xGuess, yGuess)

        return if (sign(f0) == sign(fGuess)) {
            // 0 and guess on same side of root, so use upper bracket.
            sGuess + (1.0 - sGuess) * findRoot(func, Triple(fGuess, xGuess, yGuess), point1, iterationsLeft - 1)
        } else {
            // Use lower bracket.
            sGuess * findRoot(func, Triple(f0, x0, y0), Triple(fGuess, xGuess, yGuess), iterationsLeft - 1)
        }
    }

    protected open fun findSteeringMaxS(
        point0: Triple<Double, Double, Double>,
        point1: Triple<Double, Double, Double>,
        maxDeviation: Double,
        maxIterations: Int,
    ): Double {
        val f0 = point0.first
        val x0 = point0.second
        val y0 = point0.third

        var f1 = point1.first
        val x1 = point1.second
        val y1 = point1.third

        f1 = unwrapAngle(f0, f1)
        val diff = f1 - f0

        if (abs(diff) <= maxDeviation) {
            // Can go all the way to s=1.
            return 1.0
        }

        val offset = f0 + sign(diff) * maxDeviation
        val func =
            Function2d { x: Double, y: Double ->
                unwrapAngle(f0, atan2(y, x)) - offset
            }

        return findRoot(
            func,
            Triple(f0 - offset, x0, y0),
            Triple(f1 - offset, x1, y1),
            maxIterations,
        )
    }

    protected fun findDriveMaxS(
        point0: Triple<Double, Double, Double>,
        point1: Triple<Double, Double, Double>,
        maxVelStep: Double,
        maxIterations: Int,
    ): Double {
        val f0 = point0.first
        val x0 = point0.second
        val y0 = point0.third

        val f1 = point1.first
        val x1 = point1.second
        val y1 = point1.third

        val diff = f1 - f0

        if (abs(diff) <= maxVelStep) {
            // Can go all the way to s=1.
            return 1.0
        }

        val offset = f0 + sign(diff) * maxVelStep
        val func =
            Function2d { x: Double, y: Double ->
                hypot(x, y) - offset
            }

        return findRoot(
            func,
            Triple(f0 - offset, x0, y0),
            Triple(f1 - offset, x1, y1),
            maxIterations,
        )
    }

    /**
     * Generate a new setpoint.
     *
     * @param limits The kinematic limits to respect for this setpoint.
     * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
     * iteration setpoint instead of the actual measured/estimated kinematic state.
     * @param desiredState The desired state of motion, such as from the driver sticks or a path
     * following algorithm.
     * @param dt The loop time.
     * @return A Setpoint object that satisfies all the KinematicLimits while converging to
     * desiredState quickly.
     */
    fun generateSetpoint(
        limits: ModuleLimits,
        prevSetpoint: SwerveSetpoint,
        desiredState: ChassisVelocities,
        dt: Double,
    ): SwerveSetpoint? {
        var desiredState: ChassisVelocities = desiredState
        val modules: Array<Translation2d> = moduleLocations

        var desiredModuleState: Array<SwerveModuleVelocity> = kinematics.toSwerveModuleVelocities(desiredState)
        // Make sure desiredState respects velocity limits.
        if (limits.maxDriveVelocity > 0.0) {
            val desaturatedModuleState =
                SwerveDriveKinematics.desaturateWheelVelocities(desiredModuleState, limits.maxDriveVelocity)
            desiredModuleState = desaturatedModuleState
            desiredState = kinematics.toChassisVelocities(*desaturatedModuleState)
        }

        // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
        // just use the previous angle.
        var needToSteer = true
        if (GeomUtil.toTwist2d(desiredState).epsilonEquals(Twist2d())) {
            needToSteer = false
            for (i in modules.indices) {
                desiredModuleState[i].angle = prevSetpoint.moduleStates[i].angle
                desiredModuleState[i].velocity = 0.0
            }
        }

        // For each module, compute local Vx and Vy vectors.
        val prevVx = DoubleArray(modules.size)
        val prevVy = DoubleArray(modules.size)
        val prevHeading: Array<Rotation2d> = arrayOf(*Array(modules.size) { Rotation2d() })
        val desiredVx = DoubleArray(modules.size)
        val desiredVy = DoubleArray(modules.size)
        val desiredHeading: Array<Rotation2d> = arrayOf(*Array(modules.size) { Rotation2d() })
        var allModulesShouldFlip = true
        for (i in modules.indices) {
            prevVx[i] =
                prevSetpoint.moduleStates[i].angle.cos *
                prevSetpoint.moduleStates[i].velocity
            prevVy[i] =
                prevSetpoint.moduleStates[i].angle.sin *
                prevSetpoint.moduleStates[i].velocity
            prevHeading[i] = prevSetpoint.moduleStates[i].angle

            if (prevSetpoint.moduleStates[i].velocity < 0.0) {
                prevHeading[i] = prevHeading[i].rotateBy(Rotation2d.fromRadians(Math.PI))
            }
            desiredVx[i] =
                desiredModuleState[i].angle.cos * desiredModuleState[i].velocity
            desiredVy[i] =
                desiredModuleState[i].angle.sin * desiredModuleState[i].velocity
            desiredHeading[i] = desiredModuleState[i].angle
            if (desiredModuleState[i].velocity < 0.0) {
                desiredHeading[i] = desiredHeading[i].rotateBy(Rotation2d.fromRadians(Math.PI))
            }
            if (allModulesShouldFlip) {
                val requiredRotationRad: Double =
                    abs(prevHeading[i].unaryMinus().rotateBy(desiredHeading[i]).radians)
                if (requiredRotationRad < Math.PI / 2.0) {
                    allModulesShouldFlip = false
                }
            }
        }
        if (allModulesShouldFlip &&
            !GeomUtil.toTwist2d(prevSetpoint.chassisVelocities).epsilonEquals(Twist2d()) &&
            !GeomUtil.toTwist2d(desiredState).epsilonEquals(Twist2d())
        ) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to the complement
            // of the desired
            // angle, and accelerate again.
            return generateSetpoint(limits, prevSetpoint, ChassisVelocities(), dt)
        }

        // Compute the deltas between start and goal. We can then interpolate from the start state to
        // the goal state; then
        // find the amount we can move from start towards goal in this cycle such that no kinematic
        // limit is exceeded.
        val dx: Double = desiredState.vx - prevSetpoint.chassisVelocities.vx
        val dy: Double = desiredState.vy - prevSetpoint.chassisVelocities.vy
        val dtheta: Double =
            desiredState.omega - prevSetpoint.chassisVelocities.omega

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
        // desiredState.
        var minS = 1.0

        // In cases where an individual module is stopped, we want to remember the right steering angle
        // to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
        val overrideSteering: MutableList<Optional<Rotation2d>> = ArrayList(modules.size)
        // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
        // the current angle,
        // and then backing out the maximum interpolant between start and goal states. We remember the
        // minimum across all modules, since
        // that is the active constraint.
        val maxThetaStep: Double = dt * limits.maxSteeringVelocity
        for (i in modules.indices) {
            if (!needToSteer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates[i].angle))
                continue
            }
            overrideSteering.add(Optional.empty<Rotation2d>())
            if (epsilonEquals(prevSetpoint.moduleStates[i].velocity, 0.0)) {
                // If module is stopped, we know that we will need to move straight to the final steering
                // angle, so limit based
                // purely on rotation in place.
                if (epsilonEquals(desiredModuleState[i].velocity, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering[i] = Optional.of(prevSetpoint.moduleStates[i].angle)
                    continue
                }

                var necessaryRotation: Rotation2d =
                    prevSetpoint.moduleStates[i]
                        .angle
                        .unaryMinus()
                        .rotateBy(desiredModuleState[i].angle)

                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d.fromRadians(Math.PI))
                }
                // getRadians() bounds to +/- Pi.
                val numStepsNeeded: Double = abs(necessaryRotation.radians) / maxThetaStep

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering[i] = Optional.of(desiredModuleState[i].angle)
                    // Don't limit the global min_s;
                    continue
                } else {
                    // Adjust steering by max_theta_step.
                    overrideSteering[i] =
                        Optional.of(
                            prevSetpoint.moduleStates[i].angle.rotateBy(
                                Rotation2d.fromRadians(
                                    sign(necessaryRotation.radians) * maxThetaStep,
                                ),
                            ),
                        )
                    minS = 0.0
                    continue
                }
            }
            if (minS == 0.0) {
                // s can't get any lower. Save some CPU.
                continue
            }

            val kMaxIterations = 8
            val s =
                findSteeringMaxS(
                    Triple(
                        prevHeading[i].radians,
                        prevVx[i],
                        prevVy[i],
                    ),
                    Triple(
                        desiredHeading[i].radians,
                        desiredVx[i],
                        desiredVy[i],
                    ),
                    maxThetaStep,
                    kMaxIterations,
                )
            minS = min(minS, s)
        }

        // Enforce drive wheel acceleration limits.
        val maxVelStep: Double = dt * limits.maxDriveAcceleration
        for (i in modules.indices) {
            if (minS == 0.0) {
                // No need to carry on.
                break
            }
            val vxMinS =
                if (minS == 1.0) desiredVx[i] else (desiredVx[i] - prevVx[i]) * minS + prevVx[i]
            val vyMinS =
                if (minS == 1.0) desiredVy[i] else (desiredVy[i] - prevVy[i]) * minS + prevVy[i]
            // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
            // already know we can't go faster
            // than that.
            val kMaxIterations = 10
            val s =
                (
                    minS
                        *
                        findDriveMaxS(
                            Triple(
                                hypot(prevVx[i], prevVy[i]),
                                prevVx[i],
                                prevVy[i],
                            ),
                            Triple(
                                hypot(vxMinS, vyMinS),
                                vxMinS,
                                vyMinS,
                            ),
                            maxVelStep,
                            kMaxIterations,
                        )
                )
            minS = min(minS, s)
        }

        val retSpeeds: ChassisVelocities =
            ChassisVelocities(
                prevSetpoint.chassisVelocities.vx + minS * dx,
                prevSetpoint.chassisVelocities.vy + minS * dy,
                prevSetpoint.chassisVelocities.omega + minS * dtheta,
            )
        val retStates: Array<SwerveModuleVelocity> = kinematics.toSwerveModuleVelocities(retSpeeds)

        for (i in modules.indices) {
            val maybeOverride: Optional<Rotation2d> = overrideSteering[i]
            if (maybeOverride.isPresent) {
                val override: Rotation2d = maybeOverride.get()
                if (flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
                    retStates[i].velocity *= -1.0
                }
                retStates[i].angle = override
            }
            val deltaRotation: Rotation2d =
                prevSetpoint
                    .moduleStates[i]
                    .angle
                    .unaryMinus()
                    .rotateBy(retStates[i].angle)
            if (flipHeading(deltaRotation)) {
                retStates[i].angle = retStates[i].angle.rotateBy(Rotation2d.fromRadians(Math.PI))
                retStates[i].velocity *= -1.0
            }
        }
        return SwerveSetpoint(retSpeeds, retStates)
    }
}
