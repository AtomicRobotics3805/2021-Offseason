package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.sequential
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Shooter : Subsystem {
    @JvmField
    var SHOOTER_NAME = "Shooter"
    @JvmField
    var LEFT_INDEXER_NAME = "LeftTrigger"
    @JvmField
    var RIGHT_INDEXER_NAME = "RightTrigger"
    @JvmField
    var LEFT_INDEXER_RETRACTED_POSITION = 0.925
    @JvmField
    var RIGHT_INDEXER_RETRACTED_POSITION = 0.25
    @JvmField
    var LEFT_INDEXER_EXTENDED_POSITION = 0.6
    @JvmField
    var RIGHT_INDEXER_EXTENDED_POSITION = 0.575
    @JvmField
    var SHOOTER_SPEED = 0.0
    @JvmField
    var RING_DELAY = 0.8

    private var on = false

    val switch: AtomicCommand
        get() = if (on) start else stop
    val start: AtomicCommand
        get() = powerMotor(SHOOTER_SPEED)
    val stop: AtomicCommand
        get() = powerMotor(0.0)

    val extendIndexerServos: AtomicCommand
        get() = moveIndexerServos(LEFT_INDEXER_EXTENDED_POSITION, RIGHT_INDEXER_EXTENDED_POSITION)
    val retractIndexerServos: AtomicCommand
        get() = moveIndexerServos(LEFT_INDEXER_RETRACTED_POSITION, RIGHT_INDEXER_RETRACTED_POSITION)

    val shootRing: AtomicCommand
        get() = sequential {
            +extendIndexerServos
            +Delay(RING_DELAY / 2)
            +retractIndexerServos
            +Delay(RING_DELAY / 2)
        }

    private lateinit var shooterMotor: DcMotorEx
    private lateinit var leftIndexerServo: Servo
    private lateinit var rightIndexerServo: Servo


    fun initialize() {
        shooterMotor = Constants.opMode.hardwareMap.get(DcMotorEx::class.java, SHOOTER_NAME)
        leftIndexerServo = Constants.opMode.hardwareMap.get(Servo::class.java, LEFT_INDEXER_NAME)
        rightIndexerServo = Constants.opMode.hardwareMap.get(Servo::class.java, RIGHT_INDEXER_NAME)

        shooterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun powerMotor(power: Double): AtomicCommand = CustomCommand(
            _start = {
                shooterMotor.power = power
                on = power != 0.0
            })

    fun moveIndexerServos(leftIndexerServoPosition: Double, rightIndexerServoPosition: Double):
            AtomicCommand = CustomCommand(
            _start = {
                leftIndexerServo.position = leftIndexerServoPosition
                rightIndexerServo.position = rightIndexerServoPosition
            })
}