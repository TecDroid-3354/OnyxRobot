package org.firstinspires.ftc.teamcode.subsystems.vision

import com.pedropathing.math.Matrix
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.AprilTagUtilities.aprilTagFieldLayoutDecode
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.AprilTagUtilities.ambiguity
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.AprilTagUtilities.maxZError
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.AprilTagUtilities.linearStdDevBaseline
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.AprilTagUtilities.angularStdDevBaseline
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.AprilTagUtilities.cameraStdDevFactors
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.AprilTagUtilities.linearStdDevMegatag2Factor
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.AprilTagUtilities.angularStdDevMegatag2Factor
import org.firstinspires.ftc.teamcode.constants.FieldConstants.FIELD_LENGTH
import org.firstinspires.ftc.teamcode.constants.FieldConstants.FIELD_WIDTH
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Pose2d
import org.firstinspires.ftc.teamcode.utils.extensions.toPose2dPSI
import org.firstinspires.ftc.teamcode.utils.extensions.toPose3d
import org.psilynx.psikit.core.AutoLogOutputManager
import org.psilynx.psikit.core.Logger
import org.psilynx.psikit.core.wpi.math.Pose3d
import java.util.LinkedList
import kotlin.math.abs
import kotlin.math.pow


class VisionSubsystem(private val visionConsumer: VisionConsumer, private vararg var io: VisionIO): SubsystemBase() {

    private var inputs = mutableListOf<VisionIO.VisionIOInputs>()

    init {
        for (index in io.indices) {
            inputs[index] = VisionIO.VisionIOInputs()
        }

        for (index in inputs.indices) {
            AutoLogOutputManager.addObject(inputs[index])
        }
    }

    override fun periodic() {
        for (cameraIndex in io.indices) {
            io[cameraIndex].updateInputs(inputs[cameraIndex])
        }

        val allTagPoses                     = LinkedList<Pose3d>()
        val allRobotPoses                   = LinkedList<Pose3d>()
        val allRobotPosesAccepted           = LinkedList<Pose3d>()
        val allRobotPosesRejected           = LinkedList<Pose3d>()

        for (cameraIndex in io.indices) {
            val tagPoses                    = LinkedList<Pose3d>()
            val robotPoses                  = LinkedList<Pose3d>()
            val robotPosesAccepted          = LinkedList<Pose3d>()
            val robotPosesRejected          = LinkedList<Pose3d>()

            for (tagId in inputs[cameraIndex].tagIds) {
                val tagPose = aprilTagFieldLayoutDecode.lookupTag(tagId).toPose3d()
                if (tagPose.isPresent) {
                    tagPoses.add(tagPose.get())
                }
            }

            for (observation in inputs[cameraIndex].poseObservations) {
                val rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount == 1 && observation.ambiguity() > ambiguity) // Cannot be to ambiguous
                        || abs(observation.pose.translation.z) > maxZError // Must have realistic z coordinates
                        // Must be within field boundaries
                        || observation.pose.translation.x < 0.0
                        || observation.pose.translation.x > FIELD_LENGTH.meters
                        || observation.pose.translation.y < 0.0
                        || observation.pose.translation.y > FIELD_WIDTH.meters

                robotPoses.add(observation.pose)
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose)
                } else {
                    robotPosesAccepted.add(observation.pose)
                }

                if (rejectPose) {
                    continue
                }

                val stdDevFactor    = observation.averageTagDistance.pow(2.0) / observation.tagCount
                var linearStdDev    = linearStdDevBaseline * stdDevFactor
                var angularStdDev   = angularStdDevBaseline * stdDevFactor

                if (observation.type == VisionIO.PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor
                    angularStdDev *= angularStdDevMegatag2Factor
                }
                if (cameraIndex < cameraStdDevFactors.size) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex]
                    angularStdDev *= cameraStdDevFactors[cameraIndex]
                }

                val stdDev = Matrix(3, 1)
                stdDev.set(1, 1, linearStdDev)
                stdDev.set(2, 1, linearStdDev)
                stdDev.set(3, 1, angularStdDev)

                visionConsumer.accept(
                    observation.pose.toPose2d().toPose2dPSI(),
                    observation.timestamp,
                    stdDev
                )
            }

            // Log camera datadata
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/TagPoses",
                tagPoses.toArray(arrayOfNulls<Pose3d>(tagPoses.size)))
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/RobotPoses",
                robotPoses.toArray(arrayOfNulls<Pose3d>(robotPoses.size)))
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/RobotPosesAccepted",
                robotPosesAccepted.toArray(arrayOfNulls<Pose3d>(robotPosesAccepted.size)))
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/RobotPosesRejected",
                robotPosesRejected.toArray(arrayOfNulls<Pose3d>(robotPosesRejected.size)))

            allTagPoses.addAll(tagPoses)
            allRobotPoses.addAll(robotPoses)
            allRobotPosesAccepted.addAll(robotPosesAccepted)
            allRobotPosesRejected.addAll(robotPosesRejected)
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(arrayOfNulls<Pose3d>(allTagPoses.size)))
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(arrayOfNulls<Pose3d>(allRobotPoses.size)))
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(arrayOfNulls<Pose3d>(allRobotPosesAccepted.size)))
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(arrayOfNulls<Pose3d>(allRobotPosesRejected.size)))
    }


    fun interface VisionConsumer {
        fun accept(
            visionRobotPoseMeters: Pose2d,
            timestampSeconds: Double,
            visionMeasurementStdDevs: Matrix
        )
    }
}