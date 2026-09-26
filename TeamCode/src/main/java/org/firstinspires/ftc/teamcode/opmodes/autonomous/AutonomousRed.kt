package org.firstinspires.ftc.teamcode.opModes.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.builder.AutonomousBuilder
import org.firstinspires.ftc.teamcode.utils.Alliance

@Autonomous(group = "Auto", name = "Auto - Red")
class AutonomousRed: AutonomousBuilder(Alliance.RED)