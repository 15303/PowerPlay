package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "Forward")
@Disabled
class Forward : LinearOpMode() {
    lateinit var doggy: Robot

    override fun runOpMode() {
        doggy = Robot(this)

        waitForStart()

        doggy.drive(0.4)
        sleep(1500)
        doggy.drive(0.0)

    }
}