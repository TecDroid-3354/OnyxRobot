package org.firstinspires.ftc.teamcode.subsystems.ShooterFlicker

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx

class ShooterFlicker(hardwareMap: HardwareMap) {

    private var shooterFlicker: CRServoEx

    init {
        shooterFlicker =
            CRServoEx(hardwareMap, ShooterFlickerConstants.identification.shooterFlickerId)

        shooterFlicker.setInverted(ShooterFlickerConstants.configuration.isShooterFlickerInverted)
        shooterFlicker.setRunMode(ShooterFlickerConstants.configuration.shooterFlickerMode)
    }

    fun openShooterFlicker() {
        shooterFlicker.set(90.0)
    }

    fun closeShooterFlicker() {
        shooterFlicker.set(0.0)
    }
}