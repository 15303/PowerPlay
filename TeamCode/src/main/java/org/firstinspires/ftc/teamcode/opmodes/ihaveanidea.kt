package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "ihaveanidea")
class ihaveanidea : LinearOpMode() {
    override fun runOpMode() {
        val robot: Robot = Robot(this)
        waitForStart()

        robot.motors.forEach { it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER }
        robot.motors.forEach { it.mode = DcMotor.RunMode.RUN_USING_ENCODER }
        robot.motors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        robot.drive(0.3)
        val target = robot.currentPos() + 2500
        while (robot.currentPos() < target) {
//            telemetry.addData("pos", robot.currentPos())
//            telemetry.addData("target", target)
//            telemetry.update()
        }
        robot.drive(0)
        sleep(500)
        telemetry.addData("pos", robot.currentPos())
        telemetry.addData("target", target)
        telemetry.addData("error", target - robot.currentPos())
        telemetry.update()
        sleep(1000)
    }

    fun Robot.currentPos(): Int {
        return this.motors.sumOf { it.currentPosition }
    }
}