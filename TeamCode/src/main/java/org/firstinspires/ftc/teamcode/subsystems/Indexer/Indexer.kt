package org.firstinspires.ftc.teamcode.subsystems.Indexer

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.MotorEx

class Indexer(hardwareMap: HardwareMap) {
    private val indexerMotor : MotorEx


    init {
        indexerMotor = MotorEx(hardwareMap, IndexerConstants.identification.indexerMotorId)

        indexerMotor.setInverted(IndexerConstants.configuration.isIndexMotorInverted)
        indexerMotor.setRunMode(IndexerConstants.configuration.indexMotorMode)
        indexerMotor.setZeroPowerBehavior(IndexerConstants.configuration.indexMotorZeroBeheavior)
    }

    fun enableIndexer () {
        indexerMotor.set(1.0)
    }

    fun disableIndexer () {
        indexerMotor.set(0.0)
    }

    fun reverseIndexer () {
        indexerMotor.set(-1.0)
    }
}