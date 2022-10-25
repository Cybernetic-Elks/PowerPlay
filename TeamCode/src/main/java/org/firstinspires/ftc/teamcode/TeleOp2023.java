package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "2023 TeleOp", group = "TeleOp")
/**
 * Programmer:
 * Date Created:  7/30/2022
 * Purpose: This is going to be our main teleop for PowerPlay, but for now is just to test on the new base when it is finished.
 */
public class TeleOp2023 extends LinearOpMode
{
    OpMode opmode;

    @Override
    public void runOpMode() {
        Hardware h = new Hardware();

        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();
        boolean slow = false;
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("motorFrontLeft: ", h.motorFrontLeft.getDirection());
            telemetry.addData("motorFrontRight: ", h.motorFrontRight.getDirection());
            telemetry.addData("motorBackLeft: ", h.motorBackLeft.getDirection());
            telemetry.addData("motorBackRight: ", h.motorBackRight.getDirection());
            telemetry.update();
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            slow = gamepad1.a;
            /**Start drive system**/
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                h.motorFrontLeft.setPower(-.3);
                h.motorFrontRight.setPower(.3);
                h.motorBackLeft.setPower(-.3);
                h.motorBackRight.setPower(.3);
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                h.motorFrontLeft.setPower(.3);
                h.motorFrontRight.setPower(-.3);
                h.motorBackLeft.setPower(.3);
                h.motorBackRight.setPower(-.3);
            }
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                h.motorFrontLeft.setPower(.3);
                h.motorFrontRight.setPower(.3);
                h.motorBackLeft.setPower(.3);
                h.motorBackRight.setPower(.3);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                h.motorFrontLeft.setPower(-.3);
                h.motorFrontRight.setPower(-.3);
                h.motorBackLeft.setPower(-.3);
                h.motorBackRight.setPower(-.3);
            }

            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, slow, 3);

        }
    }
}
