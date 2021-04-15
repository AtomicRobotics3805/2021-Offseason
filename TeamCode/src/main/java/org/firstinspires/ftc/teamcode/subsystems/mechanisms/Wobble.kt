package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.*
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Wobble : Subsystem {
    @JvmField
    var WOBBLE_ARM_NAME = "Wobble"
    @JvmField
    var WOBBLE_CLAW_NAME = "Hand"
    @JvmField
    var CLAW_SPEED = 1.0
    @JvmField
    var ARM_HIGH_ENCODER_POSITION = -300
    @JvmField
    var ARM_MIDDLE_ENCODER_POSITION = -800
    @JvmField
    var ARM_LOW_ENCODER_POSITION = -1050

    val openClaw: AtomicCommand
        get() = moveClaw(-CLAW_SPEED, 0.8)
    val closeClaw: AtomicCommand
        get() = moveClaw(CLAW_SPEED, 0.8)
    val idleClaw: AtomicCommand
        get() = moveClaw(0.0, 0.0)

    val raiseArmHigh: AtomicCommand
        get() = moveArm(-ARM_HIGH_ENCODER_POSITION)
    val raiseArm: AtomicCommand
        get() = moveArm(-ARM_MIDDLE_ENCODER_POSITION)
    val lowerArm: AtomicCommand
        get() = moveArm(-ARM_LOW_ENCODER_POSITION)

    val grab: AtomicCommand
        get() = parallel {
            +closeClaw
            +sequential {
                +Delay(0.8)
                +raiseArm
            }
        }

    private lateinit var arm: DcMotorEx
    private lateinit var claw: CRServo

    fun initialize() {
        arm = Constants.opMode.hardwareMap.get(DcMotorEx::class.java, WOBBLE_ARM_NAME)
        claw = Constants.opMode.hardwareMap.get(CRServo::class.java, WOBBLE_CLAW_NAME)

        arm.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun moveArm(position: Int) = CustomCommand(
            _start = {
                arm.targetPosition = position
            },
            getDone = {
                !arm.isBusy
            })

    fun moveClaw(power: Double, time: Double) = TimedCustomCommand(
            _start = {
                claw.power = power
            }, time = time)
}