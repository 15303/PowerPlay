package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot

@TeleOp(name = "ApriltagTest")
class CameraTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("recognized id", robot.camera?.lastRecognition?.id)
            telemetry.update()
        }
    }
}
