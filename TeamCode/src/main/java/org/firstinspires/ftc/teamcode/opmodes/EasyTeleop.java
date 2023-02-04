package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "EasyTeleOp")
public class EasyTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        robot.getLifter().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLifter().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            double drive = -gamepad1.right_stick_y * 0.5;
            double turn = gamepad1.left_stick_x * 0.5;

            robot.power(
                    drive+turn,
                    drive-turn,
                    drive+turn,
                    drive-turn
            );

            //0 to 4440
            double lifter = 0;
            if (robot.getLifter().getCurrentPosition() > 0 && gamepad1.dpad_down) {
                lifter = -0.5;
            }
            if (robot.getLifter().getCurrentPosition() < 4440 && gamepad1.dpad_up) {
                lifter = 0.5;
            }
            robot.getLifter().setPower(lifter);

            robot.getGrabber().setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            telemetry.addData("lifterpos", robot.getLifter().getCurrentPosition());
            telemetry.update();
        }
    }
}
