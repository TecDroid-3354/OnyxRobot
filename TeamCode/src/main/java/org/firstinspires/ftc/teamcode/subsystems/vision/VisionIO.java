package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.psilynx.psikit.core.AutoLogOutput;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

public interface VisionIO {

    class VisionIOInputs {
        @AutoLogOutput(key = VisionConstants.Telemetry.CONNECTED_FIELD) public boolean connected = false;
        @AutoLogOutput(key = VisionConstants.Telemetry.TARGET_OBSERVATION_FIELD) public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(0.0), new Rotation2d(0.0));
        @AutoLogOutput(key = VisionConstants.Telemetry.POSE_OBSERVATIONS_FIELD) public PoseObservation[] poseObservations = new PoseObservation[0];
        @AutoLogOutput(key = VisionConstants.Telemetry.APRIL_TAG_IDS_FIELD) public int[] tagIds = new int[0];
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /**
     * Represents a robot pose sample used for pose estimation.
     */
    record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount,
                           double averageTagDistance, PoseObservationType type) {
        public PoseObservation() {
                this(0.0, new Pose3d(new Translation3d(), new Rotation3d()), 0.0, 0, 0.0, PoseObservationType.UNKNOWN);
        }
    }

    enum PoseObservationType {
        MEGATAG_1, MEGATAG_2, UNKNOWN;
    }

    Runnable start();

    Boolean isCameraResultValid();

    default void updateInputs(VisionIOInputs inputs) {}
}
