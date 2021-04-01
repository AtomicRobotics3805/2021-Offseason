package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Intake {
    private lateinit var motor: DcMotorEx

    fun initialize() {
        motor = opMode.hardwareMap.get(DcMotorEx::class.java, MechanismConstants.INTAKE_NAME)
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun start(): AtomicCommand {
        return CustomCommand(_start = {motor.power = MechanismConstants.INTAKE_SPEED})
    }

    fun stop(): AtomicCommand {
        return CustomCommand(_start = {motor.power = 0.0})
    }
}