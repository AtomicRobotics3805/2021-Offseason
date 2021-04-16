package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Intake : Subsystem {
    @JvmField
    var INTAKE_NAME = "Intake"
    @JvmField
    var INTAKE_SPEED = 1.0

    var on = false

    val switch: AtomicCommand
        get() = if (on) start else stop
    val start: AtomicCommand
        get() = powerIntake(INTAKE_SPEED)
    val stop: AtomicCommand
        get() = powerIntake(0.0)

    private lateinit var motor: DcMotorEx

    fun initialize() {
        motor = opMode.hardwareMap.get(DcMotorEx::class.java, INTAKE_NAME)
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun powerIntake(power: Double) = CustomCommand(_start = {
        motor.power = power
        on = power != 0.0
    })
}