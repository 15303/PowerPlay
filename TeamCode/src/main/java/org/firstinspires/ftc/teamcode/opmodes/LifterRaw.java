package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LifterRaw")
public class LifterRaw extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lifter = hardwareMap.get(DcMotor.class, "lifter");
        waitForStart();
        while (opModeIsActive()) {
            lifter.setPower(gamepad1.left_stick_y);
        }
    }
}
