package org.firstinspires.ftc.teamcode.subsystems.Shooter

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.teamcode.utils.units.AngularVelocity

class Shooter (hardwareMap: HardwareMap) {

    private val shooterMotor: MotorEx

    init {
        shooterMotor = MotorEx(hardwareMap, ShooterConstants.identification.shooterMotorId)

        shooterMotor.setInverted(ShooterConstants.configuration.isSooterMotorInverted)
        shooterMotor.setRunMode(ShooterConstants.configuration.shooterMotorMode)
        shooterMotor.setZeroPowerBehavior(ShooterConstants.configuration.shooterMotorZeroBeheavior)
    }

    fun shoot () {
        shooterMotor.velocity = 28 * 5000.0
    }

    fun calibrateShooter(velocity: AngularVelocity) {
        shooterMotor.velocity = velocity.rps * 28
    }

    fun stopShooter() {
        shooterMotor.set(0.0)
    }
    //this will not have CMD because we are going to return the shooter's velocity to print it
    fun getVelocity(): AngularVelocity {
        return AngularVelocity.fromRps(shooterMotor.velocity/28)//Change formula to solve gear ratio
    }


    fun shootCMD(): Command {
        return InstantCommand({shoot()})
    }

    fun setShooterVelocityCMD(velocity: AngularVelocity): Command {
        return RunCommand({calibrateShooter(velocity)})
    }

    fun stopShooterCMD(): Command {
        return InstantCommand({stopShooter()})
    }


}