package org.firstinspires.ftc.teamcode.subsystems.vision

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary

object VisionConstants {

    object Identification {
        const val LIMELIGHT_ID: String = "limelight"
    }

    object Control {
        const val POLL_RATE_HZ: Int = 250;
    }

    object Pipelines {
        const val APRIL_TAG_PIPELINE: Int = 0
        const val COLOR_PIPELINE: Int = 1
        const val CLASSIFIER_PIPELINE: Int = 2
        const val NEURAL_NETWORK_PIPELINE: Int = 3
        const val RETROREFLECTIVE_PIPELINE: Int = 4
    }

    object AprilTagUtilities {
        val aprilTagFieldLayoutDecode   : AprilTagLibrary   = AprilTagGameDatabase.getDecodeTagLibrary()
        val ambiguity                   : Double            = 0.18
        val maxZError                   : Double            = 0.75
        var linearStdDevBaseline: Double = 0.02 // Meters
        var angularStdDevBaseline: Double = 0.06 // Radians
        var cameraStdDevFactors: DoubleArray = doubleArrayOf(
            1.0,  // Camera 0
        )
        var linearStdDevMegatag2Factor: Double = 0.5 // More stable than full 3D solve
        var angularStdDevMegatag2Factor: Double = Double.POSITIVE_INFINITY // No rotation data available
    }

    object Telemetry {
        const val VISION_TAB: String = "Vision"
        const val CONNECTED_FIELD: String = "${VISION_TAB}/Connnected"
        const val TARGET_OBSERVATION_FIELD: String = "${VISION_TAB}/Target Observation"
        const val POSE_OBSERVATIONS_FIELD: String = "${VISION_TAB}/Pose Observations"
        const val APRIL_TAG_IDS_FIELD: String = "${VISION_TAB}/April tag Ids"
    }
}