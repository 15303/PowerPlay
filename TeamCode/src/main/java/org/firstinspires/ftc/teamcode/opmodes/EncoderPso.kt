package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot

@TeleOp(name="EncoderPso")
class EncoderPso : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)

        waitForStart()

        robot.reset()
        while (opModeIsActive()) {
            telemetry.addData("lifter", robot.lifter.currentPosition)
            telemetry.addData("fl", robot.fl.currentPosition)
            telemetry.addData("fr", robot.fr.currentPosition)
            telemetry.addData("bl", robot.bl.currentPosition)
            telemetry.addData("br", robot.br.currentPosition)
            telemetry.update()

        }
    }
}