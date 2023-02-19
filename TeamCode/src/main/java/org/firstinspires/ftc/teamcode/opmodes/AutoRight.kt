package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveController
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "AutoRight")
class AutoRight : LinearOpMode() {
    override fun runOpMode() {
        runAuto(true)
    }
}