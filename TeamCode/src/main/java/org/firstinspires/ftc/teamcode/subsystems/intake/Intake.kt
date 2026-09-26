package org.firstinspires.ftc.teamcode.subsystems.Intake

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.MotorEx

class Intake (hardwareMap: HardwareMap) {

    private val intakeMotor : MotorEx



    init {
        intakeMotor = MotorEx(hardwareMap, IntakeConstants.identification.intakeMotorId)

    }
}