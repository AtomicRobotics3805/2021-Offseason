package org.firstinspires.ftc.teamcode.subsystems.driving

import org.firstinspires.ftc.teamcode.Singleton.drive

@Suppress("unused")
class TurnRelative(angle: Double): Turn(angle + drive.poseEstimate.heading)