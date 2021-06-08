package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.*
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Wobble : Subsystem {
    @JvmField
    var WOBBLE_ARM_NAME = "Wobble"
    @JvmField
    var WOBBLE_CLAW_NAME = "Hand"
    @JvmField
    var CLAW_OPEN_POSITION = 0.3
    @JvmField
    var CLAW_CLOSED_POSITION = 0.7
    @JvmField
    var PICK_UP_ENCODER_POSITION = -1000
    @JvmField
    var DROP_ENCODER_POSITION = -950
    @JvmField
    var HIGH_ENCODER_POSITION = -300

    val openClaw: AtomicCommand
        get() = moveClaw(CLAW_OPEN_POSITION, 0.8)
    val closeClaw: AtomicCommand
        get() = moveClaw(CLAW_CLOSED_POSITION, 0.8)

    val raiseArmHigh: AtomicCommand
        get() = moveArm(HIGH_ENCODER_POSITION)
    val raiseArm: AtomicCommand
        get() = moveArm(DROP_ENCODER_POSITION)
    val lowerArm: AtomicCommand
        get() = moveArm(PICK_UP_ENCODER_POSITION)

    val grab: AtomicCommand
        get() = parallel {
            +closeClaw
            +sequential {
                +Delay(0.8)
                +raiseArm
            }
        }

    private lateinit var arm: DcMotorEx
    private lateinit var claw: Servo

    fun initialize() {
        arm = Constants.opMode.hardwareMap.get(DcMotorEx::class.java, WOBBLE_ARM_NAME)
        claw = Constants.opMode.hardwareMap.get(Servo::class.java, WOBBLE_CLAW_NAME)

        arm.targetPosition = 0
        arm.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun moveArm(position: Int) = CustomCommand(
            _start = {
                arm.targetPosition = position
                arm.power = 1.0
            },
            getDone = {
                !arm.isBusy
            })

    fun moveClaw(position: Double, time: Double) = TimedCustomCommand(
            _start = {
                claw.position = position
            }, time = time)
}