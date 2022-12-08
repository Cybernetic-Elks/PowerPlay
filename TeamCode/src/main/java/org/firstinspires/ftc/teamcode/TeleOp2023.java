package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "2023 TeleOp - CHOOSE THIS ONE", group = "TeleOp")
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
        ElapsedTime runtime = new ElapsedTime();

        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();
        boolean pressedLastIterationOuttake = false;
        boolean slow = false;
        boolean slow2 = false;
        boolean descending = false;
        final int UPPER_LIMIT = 12500;
        final int HIGH_GOAL = 10530;
        final int MID_GOAL = 7420;
        final int LOW_GOAL = 4400;
        final int CONE_HEIGHT = 0;


        waitForStart();
        while (opModeIsActive()) {
            boolean pressedOutake = gamepad2.a;
            telemetry.addData("motorFrontLeft: ", h.motorFrontLeft.getDirection());
            telemetry.addData("motorFrontRight: ", h.motorFrontRight.getDirection());
            telemetry.addData("motorBackLeft: ", h.motorBackLeft.getDirection());
            telemetry.addData("motorBackRight: ", h.motorBackRight.getDirection());
            telemetry.addData("servoIntakeClose: ", h.servoIntakeClose.getPower());
            telemetry.addData("servoIntakeFar: ", h.servoIntakeFar.getPower());
            telemetry.addData("motorLift current Pos: ", h.motorLift.getCurrentPosition());
            telemetry.addData("touchSensor is pressed: ", h.touch.isPressed());
            telemetry.update();
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            slow = gamepad1.right_trigger > 0.01 ? true: false;
            slow2 = gamepad2.y;

            /**Start drive system**/
            if (gamepad1.dpad_left) {
                h.motorFrontLeft.setPower(-.4);
                h.motorFrontRight.setPower(.4);
                h.motorBackLeft.setPower(-.4);
                h.motorBackRight.setPower(.4);
            } else if (gamepad1.dpad_right) {
                h.motorFrontLeft.setPower(.4);
                h.motorFrontRight.setPower(-.4);
                h.motorBackLeft.setPower(.4);
                h.motorBackRight.setPower(-.4);
            }
            if (gamepad1.dpad_up) {
                h.motorFrontLeft.setPower(.4);
                h.motorFrontRight.setPower(.4);
                h.motorBackLeft.setPower(.4);
                h.motorBackRight.setPower(.4);
            } else if (gamepad1.dpad_down) {
                h.motorFrontLeft.setPower(-.4);
                h.motorFrontRight.setPower(-.4);
                h.motorBackLeft.setPower(-.4);
                h.motorBackRight.setPower(-.4);
            }

            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, slow, 5, 2);
            /*h.motorLift.setPower(-gamepad1.left_trigger);
            h.motorLift.setPower(gamepad1.right_trigger);*/

            //Motor Lift Controls
            if(gamepad2.dpad_up /* && h.motorLift <= UPPER_LIMIT */)
            {
                if (slow2)
                {
                    h.motorLift.setPower(.7);

                }
                else
                {
                    h.motorLift.setPower(1);
                }
            }
            if(gamepad2.dpad_down && !h.touch.isPressed())
            {
                if (slow2)
                {
                    h.motorLift.setPower(-.6);

                }
                else
                {
                    h.motorLift.setPower(-1);
                }

            }
            if(!gamepad2.dpad_up && h.touch.isPressed())
            {
                h.motorLift.setPower(0);
            }
            if(!gamepad2.dpad_up && !gamepad2.dpad_down)
            {
                h.motorLift.setPower(0);
            }
            if (h.touch.isPressed())
            {
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            /*
            if (gamepad2.dpad_up)
            {
                h.motorLift.setTargetPosition(HIGH_GOAL);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower(1);
            }
            if (gamepad2.dpad_left)
            {
                h.motorLift.setTargetPosition(MID_GOAL);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower(1);
            }
            if (gamepad2.dpad_down)
            {
                h.motorLift.setTargetPosition(LOW_GOAL);
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower(1);
            }
            if (gamepad2.dpad_right)
            {
                h.motorLift.setPower(-1);
                descending = true;
            }
            if(descending)
            {
                if(h.touchSensor.isPressed())
                {
                    descending = false;
                    h.motorLift.setPower(0);
                }
            }
            */

            //Outake
            if(pressedOutake & !pressedLastIterationOuttake) {
                runtime.reset();
                while (runtime.time() < 1) {
                    h.servoIntakeClose.setPower(-1);
                    h.servoIntakeFar.setPower(1);
                }
            }
            //Intake
            if(gamepad2.b)
            {
                    h.servoIntakeClose.setPower(1);
                    h.servoIntakeFar.setPower(-1);

            }
            if(!gamepad2.a && !gamepad2.b)
            {
                h.servoIntakeClose.setPower(0);
                h.servoIntakeFar.setPower(0);
            }


            pressedLastIterationOuttake = pressedOutake;



        }
    }
}
