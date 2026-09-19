package org.firstinspires.ftc.teamcode.subsystems.ShooterFlicker

import com.seattlesolvers.solverslib.hardware.motors.CRServoEx

object ShooterFlickerConstants {

    object identification {
        val shooterFlickerId = "shooterFlicker"
    }

    object configuration {
        val isShooterFlickerInverted = false
        val shooterFlickerMode = CRServoEx.RunMode.RawPower
    }
}