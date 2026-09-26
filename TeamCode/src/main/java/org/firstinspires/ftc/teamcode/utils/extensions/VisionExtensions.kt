package org.firstinspires.ftc.teamcode.utils.extensions

import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata
import org.psilynx.psikit.core.wpi.math.Pose3d
import org.psilynx.psikit.core.wpi.math.Rotation3d
import org.psilynx.psikit.core.wpi.math.Translation3d
import java.util.Optional

fun AprilTagMetadata.toPose3d(): Optional<Pose3d> {
    if (this == null) {
        return Optional.empty()
    }

    val translation3d = this.fieldPosition.toTranslation3d()
    val rotation3d = Rotation3d(this.fieldOrientation.toQuaternionPSI())
    return Optional.of(Pose3d(translation3d, rotation3d))
}

fun Quaternion.toQuaternionPSI(): org.psilynx.psikit.core.wpi.math.Quaternion {
    return org.psilynx.psikit.core.wpi.math.Quaternion(this.w.toDouble(), this.x.toDouble(), this.y.toDouble(), this.z.toDouble())
}

fun VectorF.toTranslation3d(): Translation3d {
    if (this.length() < 3) {
        return Translation3d()
    }

    val xMeters = Distance.fromInches(this.get(0).toDouble()).meters
    val yMeters = Distance.fromInches(this.get(1).toDouble()).meters
    val zMeters = Distance.fromInches(this.get(2).toDouble()).meters

    return Translation3d(xMeters, yMeters, zMeters)
}