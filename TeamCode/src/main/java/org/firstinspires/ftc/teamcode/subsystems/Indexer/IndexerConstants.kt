package org.firstinspires.ftc.teamcode.subsystems.Indexer

import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import com.seattlesolvers.solverslib.hardware.motors.Motor

object IndexerConstants {
    object identification {
        val indexerMotorId = "indexerMotor"
    }

    object configuration {
        val isIndexMotorInverted = false //Update later pls
        val indexMotorMode = Motor.RunMode.RawPower
        val indexMotorZeroBeheavior = Motor.ZeroPowerBehavior.FLOAT
    }
}