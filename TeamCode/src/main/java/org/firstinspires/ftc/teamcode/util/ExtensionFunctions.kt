@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation


val Double.toRadians get() = Math.toRadians(this)
val Double.toDegrees get() = Math.toDegrees(this)
val Double.inchesToMm get() = this * 25.4
val Double.mmToInches get() = this / 25.4
val OpMode.isStopRequested get() = this is LinearOpMode && isStopRequested

fun Vector2d(pose: Pose2d) = Vector2d(pose.x, pose.y)
fun Pose2d(matrix: OpenGLMatrix) = Pose2d(
        matrix.translation.get(0).toDouble().mmToInches,
        // TODO("Figure out where this is matrix.get(1) (y) or matrix.get(2) (z)")
        matrix.translation.get(1).toDouble().mmToInches,
        Orientation.getOrientation(matrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)
                .thirdAngle.toDouble().toRadians
)