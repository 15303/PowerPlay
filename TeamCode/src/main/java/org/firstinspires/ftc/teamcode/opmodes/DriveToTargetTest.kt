package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.DriveController
import org.firstinspires.ftc.teamcode.PIDDriveController
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "DriveToTargetTest")
class DriveToTargetTest: LinearOpMode() {
    override fun runOpMode() {
        val robot: Robot = Robot(this)
        val controller: DriveController = PIDDriveController(robot)
        controller.enableAngleCorrection()
        waitForStart()
        robot.reset()

        controller.driveToTargetPos(
            0.4,
            targetPos = controller.currentPos + 3000,
            time = 10000
        )
        sleep(2000)
        controller.driveToTargetPos(
            0.4,
            targetPos = controller.currentPos - 3000,
            time = 10000
        )
    }

}