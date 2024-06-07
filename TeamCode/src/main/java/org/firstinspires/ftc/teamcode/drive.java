package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="drive")
// @Disabled

public class drive extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {

        swerveRobotHardware robot = new swerveRobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            robot.swerveDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);

        }
    }
}
