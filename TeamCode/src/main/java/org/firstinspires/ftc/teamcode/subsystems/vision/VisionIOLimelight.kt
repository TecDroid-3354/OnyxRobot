@file:Suppress("JoinDeclarationAndAssignment")

package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.utils.extensions.toPose3d
import java.util.LinkedList
import java.util.function.Supplier

class VisionIOLimelight(
    hardwareMap: HardwareMap,
    private val rotationSupplier: Supplier<Rotation2d>
) : VisionIO {

    private var limelight: Limelight3A

    private var limelightResult: LLResult? = null

    init {
        limelight = hardwareMap.get(Limelight3A::class.java, VisionConstants.Identification.LIMELIGHT_ID)
        limelight.setPollRateHz(VisionConstants.Control.POLL_RATE_HZ)
    }

    override fun isCameraResultValid(): Boolean {
        return limelightResult!!.isValid && limelightResult != null
    }

    override fun start(): Runnable {
        return Runnable{ limelight.start() }
    }

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        val tagIds = hashSetOf<Int>()
        val poseObservations = LinkedList<VisionIO.PoseObservation>()

        limelight.updateRobotOrientation(rotationSupplier.get().degrees)
        limelightResult = limelight.latestResult

        inputs.connected = limelight.isConnected

        if (isCameraResultValid) {
            inputs.latestTargetObservation = VisionIO.TargetObservation(
                Rotation2d(limelightResult!!.tx),
                Rotation2d(limelightResult!!.ty)
            )


            for (fiducial in limelightResult!!.fiducialResults) {
                tagIds.add(fiducial.fiducialId)
            }

            poseObservations.add(
                VisionIO.PoseObservation(
                    limelightResult!!.timestamp,
                    limelightResult!!.botpose_MT2.toPose3d(),
                    0.0,
                    limelightResult!!.fiducialResults.size,
                    limelightResult!!.botposeAvgDist,
                    VisionIO.PoseObservationType.MEGATAG_2
                )
            )
        }

        for (index in poseObservations.indices) {
            inputs.poseObservations[index] = poseObservations[index]
        }

        for (index in tagIds.indices) {
            inputs.tagIds[index] = tagIds.elementAt(index)
        }
    }
}