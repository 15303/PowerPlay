package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "Auto")
class Auto : LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)

        while (!isStarted) {
            telemetry.addData("current id:", robot.camera?.lastRecognition?.id)
            sleep(50)
        }

        val id = robot.camera?.lastRecognition?.id ?: 1
        telemetry.addLine("recognized id $id")
        telemetry.update()

        robot.drive(0.4)
        sleep(1500)
        robot.drive(0.0)

    }
}