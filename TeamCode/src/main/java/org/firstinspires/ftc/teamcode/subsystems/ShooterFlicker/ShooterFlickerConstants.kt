package org.firstinspires.ftc.teamcode.subsystems.ShooterFlicker

import com.seattlesolvers.solverslib.hardware.motors.CRServoEx

object ShooterFlickerConstants {

    object identification {
        val shooterFlickerId = "shooterFlicker"
    }

    object configuration {
        val isShooterFlickerInverted = false //update later pls
        val shooterFlickerMode = CRServoEx.RunMode.OptimizedPositionalControl
    }
}