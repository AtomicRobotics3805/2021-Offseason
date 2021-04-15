package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode


val Double.toRadians get() = (Math.toRadians(this))
val Double.toDegrees get() = (Math.toDegrees(this))
val OpMode.isStopRequested get() = (this is LinearOpMode && isStopRequested)

fun Vector2d(pose: Pose2d) = Vector2d(pose.x, pose.y)