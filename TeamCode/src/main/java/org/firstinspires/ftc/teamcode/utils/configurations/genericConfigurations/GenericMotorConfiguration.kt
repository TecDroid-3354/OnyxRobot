package org.firstinspires.ftc.teamcode.utils.configurations.genericConfigurations

import com.seattlesolvers.solverslib.hardware.motors.Motor

class GenericMotorConfiguration {
    var inverted                : Boolean                   = false
    var ticksPerRev             : Double                    = 28.0
    var reduction               : Double                    = 1.0
    var cachingTolerance        : Double                    = 0.012
    var zeroPowerBehavior       : Motor.ZeroPowerBehavior   = Motor.ZeroPowerBehavior.FLOAT

    fun withInverted(value: Boolean)    : GenericMotorConfiguration {
        this.inverted = value
        return this
    }

    fun withTicksPerRev(value: Double)  : GenericMotorConfiguration {
        this.ticksPerRev = value
        return this
    }

    fun withGearRatio(value: Double)    : GenericMotorConfiguration {
        this.reduction = value
        return this
    }

    fun withCachingTolerance(value: Double)                     : GenericMotorConfiguration {
        this.cachingTolerance = value
        return this
    }

    fun withZeroPowerBehavior(value: Motor.ZeroPowerBehavior)   : GenericMotorConfiguration {
        this.zeroPowerBehavior = value
        return this
    }
}