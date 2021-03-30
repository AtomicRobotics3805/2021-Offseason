package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Singleton
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Suppress("Unused", "MemberVisibilityCanBePrivate")
class Shooter {
    private lateinit var shooterMotor: DcMotorEx
    private lateinit var leftIndexerServo: Servo
    private lateinit var rightIndexerServo: Servo


    fun initialize() {
        shooterMotor = Singleton.opMode.hardwareMap.get(DcMotorEx::class.java, MechanismConstants.SHOOTER_NAME)
        leftIndexerServo = Singleton.opMode.hardwareMap.get(Servo::class.java, MechanismConstants.LEFT_INDEXER_NAME)
        rightIndexerServo = Singleton.opMode.hardwareMap.get(Servo::class.java, MechanismConstants.RIGHT_INDEXER_NAME)

        shooterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun startMotor(): AtomicCommand = CustomCommand(
            _start = { shooterMotor.power = 1.0 })

    fun stopMotor(): AtomicCommand = CustomCommand(
            _start = { shooterMotor.power = MechanismConstants.SHOOTER_SPEED })

    fun extendIndexerServos(): AtomicCommand = CustomCommand(
            _start = {
                leftIndexerServo.position = 0.6
                rightIndexerServo.position = 0.575
            })

    fun retractIndexerServos(): AtomicCommand = CustomCommand(
            _start = {
                leftIndexerServo.position = 0.925
                rightIndexerServo.position = 0.25
            })

    fun shootRing(): AtomicCommand {
        return sequential {
            +extendIndexerServos()
            +Delay(MechanismConstants.RING_DELAY / 2)
            +retractIndexerServos()
        }
    }
}