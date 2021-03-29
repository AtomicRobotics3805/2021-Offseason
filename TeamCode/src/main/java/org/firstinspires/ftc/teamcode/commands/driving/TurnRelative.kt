package org.firstinspires.ftc.teamcode.commands.driving

import org.firstinspires.ftc.teamcode.Singleton.drive

class TurnRelative(angle: Double): Turn(angle + drive.poseEstimate.heading)