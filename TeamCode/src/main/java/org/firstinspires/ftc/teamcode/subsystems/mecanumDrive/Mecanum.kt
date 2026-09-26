package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.pedropathing.follower.Follower
import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.geometry.Vector2d
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.extensions.toPose2d
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Distance
import org.firstinspires.ftc.teamcode.utils.units.LinearVelocity
import java.util.Optional
import kotlin.math.atan2

class Mecanum(
    private val follower: Follower,
    private val controller: GamepadEx,
    private val alliance: Alliance
): SubsystemBase() {

    /**
     * Runs in every loop. Follower and telemetry get updated
     */
    override fun periodic() {
        follower.update()
    }

    /**
     * Retrieves the [controller]'s axis readings and converts them into robot's velocity.
     * The axis get multiplied by each [MecanumConstants.Control] Multiplier and its respective alliance multiplier.
     * @return a [RunCommand] which set the [Follower]'s TeleOp drive to the [controller]'s axis.
     */
    fun driveFollowingDriverInput(): Command {
        return RunCommand({
            val fieldCentricDrive = ManualDrive.fieldCentric(
                -controller.leftY * MecanumConstants.Control.FORWARD_VELOCITY_MULTIPLIER * alliance.multiplier,
                controller.leftX * MecanumConstants.Control.LATERAL_VELOCITY_MULTIPLIER * alliance.multiplier,
                controller.rightX * MecanumConstants.Control.TURN_VELOCITY_MULTIPLIER,
                follower.pose().heading()
            )

            follower.manual(fieldCentricDrive)
        })
            .addRequirements(this)
    }

    /**
     * Gets the Follower's current position.
     * @return a [Pose2D] containing the robot's current position in the standard FTC Coordinates
     */
    fun getPose(): Pose2d {
        return follower.pose().toPose2d()
    }

    /**
     * Gets the [Follower]'s rotation component.
     * @return a [Rotation2d] as the robot's current heading in radians.
     */
    fun getRotation(): Rotation2d {
        return Rotation2d(getPose().heading)
    }

    /**
     * Gets the current [Follower]'s velocity as a [ChassisSpeeds].
     * This represents the velocity of the robot in the field.
     * @return the current in the field's frame
     */
    fun getFieldRelativeVelocity(): ChassisSpeeds {
        return ChassisSpeeds(follower.velocity().vx, follower.velocity().vy, follower.velocity().omega)
    }

    /**
     * Gets the current [Follower]'s velocity as a [ChassisSpeeds].
     * This represents the velocity of the robot.
     * @return the current robot's velocity in the robot's frame
     */
    fun getRobotRelativeVelocity(): ChassisSpeeds {
        return ChassisSpeeds(follower.twist().vx, follower.twist().vy, follower.twist().omega)
    }

    /**
     * Constructs a vector from the robot to a target and returns the projection of the velocity vector onto the distance unit vector.
     * If the result is positive, then the robot is driving towards the target.
     * If the result is negative, then the robot is driving away from the target.
     */
    fun getRobotRadialVelocity(fieldToTarget: Translation2d): LinearVelocity {
        val fieldRelativeVelocity = getFieldRelativeVelocity()
        val robotToTargetVector = fieldToTarget.minus(getPose().translation)
        val robotToTargetDistance = robotToTargetVector.norm

        if (robotToTargetDistance < 1e-5) return LinearVelocity(0.0)

        val radialUnitTranslation = robotToTargetVector.div(robotToTargetDistance)
        val radialUnitVector = Vector2d(radialUnitTranslation.x, radialUnitTranslation.y)

        val velocityVector = Vector2d(fieldRelativeVelocity.vxMetersPerSecond, fieldRelativeVelocity.vyMetersPerSecond)
        val radialVectorMagnitude = velocityVector.dot(radialUnitVector)

        return LinearVelocity.fromMps(radialVectorMagnitude)
    }

    /**
     * Constructs a vector from the robot to a target, the rotates it by 90.0 degrees and returns the projection of the velocity vector onto the tangential unit vector.
     * If the result is positive, then the robot is driving towards the target.
     * If the result is negative, then the robot is driving away from the target.
     */
    fun getRobotTangentialVelocity(fieldToTarget: Translation2d): LinearVelocity {
        val fieldRelativeSpeeds = getFieldRelativeVelocity()

        val robotToTargetVector = fieldToTarget.minus(getPose().translation)
        val robotToTargetDistance = robotToTargetVector.norm

        if (robotToTargetDistance < 1e-5) return LinearVelocity(0.0)

        val radialUnitTranslation = robotToTargetVector.div(robotToTargetDistance)
        val tangentialUnitTranslation = radialUnitTranslation.rotateBy(Rotation2d.fromDegrees(90.0))
        val tangentialUnitVector = Vector2d(tangentialUnitTranslation.x, tangentialUnitTranslation.y)

        val velocityVector = Vector2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
        val tangentialVectorMagnitude = velocityVector.dot(tangentialUnitVector)

        return LinearVelocity.fromMps(tangentialVectorMagnitude)
    }

    /**
     * Constructs a vector from the robot to a target and returns its angle plus an [Optional] [Rotation2d]
     * @return the angle of the vector plus the offset
     */
    fun getAngleFromRobotToTarget(fieldToTarget: Translation2d, headingOffset: Optional<Rotation2d>): Angle {
        val robotToTargetVector = fieldToTarget.minus(getPose().translation)

        val targetAngle = Rotation2d(
            atan2(robotToTargetVector.y, robotToTargetVector.x)
        ).plus(headingOffset.orElse(Rotation2d()))

        return Angle.fromRadians(targetAngle.radians)
    }

    /**
     * Gets the distance of the chassis to any target passed to this function.
     * Uses the [Pose.distance] method to calculate the distance.
     * @param target the target to get the distance from
     * @return the distance from the robot's center to the specified [target]
     */
    fun getDistanceTo(target: Pose): Distance {
        val distance = follower.pose().distance(target)

        return Distance.fromInches(distance)
    }

    fun followPathCMD(path: Path, holdEnd: Boolean, maxPower: Double): Command {
        return FollowPathCommand(follower, path, holdEnd, maxPower)
            .addRequirements(this)
    }

    /**
     * Sets a new [Pose] to our robot's chassis.
     * @param pose a pose representing the new robot's [Pose].
     */
    fun setPose(pose: Pose) {
        follower.setPose(pose)
    }
}