package org.firstinspires.ftc.teamcode.subsystems.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.hardware.motors.Motor

object ShooterConstants {

    object identification {
        val shooterMotorId = "shooterMotor"
    }

    object configuration {
        val isSooterMotorInverted = false //Update later pls
        val shooterMotorMode = Motor.RunMode.VelocityControl //Change mode to RawPower
        val shooterMotorZeroBeheavior = Motor.ZeroPowerBehavior.FLOAT
    }

    @Configurable

    object Tunables {
        @JvmField
        var pidCoefficients = PIDCoefficients(1.0, 0.0, 0.0) //NEED TO UPDATE
        @JvmField
        var feedforward = SimpleMotorFeedforward(0.0,15.5) //NEED TO UPDATE
    }

}