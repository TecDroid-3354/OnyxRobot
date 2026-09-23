package org.firstinspires.ftc.teamcode.subsystems.ShooterFlicker

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
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


    fun openShooterFlickerCMD(): Command {
        return InstantCommand({openShooterFlicker()})
    }

    fun closeShooterFlickerCMD(): Command {
        return InstantCommand({closeShooterFlicker()})
    }
}