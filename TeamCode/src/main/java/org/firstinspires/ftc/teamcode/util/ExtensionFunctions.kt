package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d


val Double.toRadians get() = (Math.toRadians(this))

fun Vector2d(pose: Pose2d): Vector2d = Vector2d(pose.x, pose.y)