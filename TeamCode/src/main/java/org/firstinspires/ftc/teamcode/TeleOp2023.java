package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "2023 TeleOp - CHOOSE THIS ONE", group = "TeleOp")
/**
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

        final int LEFT_TABLE_POS = 0;
        final int RIGHT_TABLE_POS = 0;
        final int FRONT_TABLE_POS = 0;
        final int BACK_TABLE_POS = 0;


        waitForStart();
        while (opModeIsActive()) {
            boolean pressedOutake = gamepad2.a;
            telemetry.addData("servoIntakeClose: ", h.servoIntakeClose.getPower());
            telemetry.addData("servoIntakeFar: ", h.servoIntakeFar.getPower());
            telemetry.addData("motorLift current Pos: ", h.motorLift.getCurrentPosition());
            telemetry.addData("motorLift2 current Pos: ", h.motorLift2.getCurrentPosition());
            telemetry.addData("motorTable current Pos: ", h.motorTable.getCurrentPosition());
            telemetry.update();
            slow = gamepad1.right_trigger > 0.01;
            slow2 = gamepad2.y;

            /**Start drive system**/
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, slow, 5, 2);

            /*h.motorLift.setPower(-gamepad1.left_trigger);
            h.motorLift.setPower(gamepad1.right_trigger);*/

            //Motor Lift Controls
            if(gamepad2.dpad_up /* && h.motorLift <= UPPER_LIMIT */)
            {
                if (slow2)
                {
                    h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    h.motorLift.setPower(.7);

                }
                else
                {
                    h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    h.motorLift.setPower(1);
                }
            }
            if(gamepad2.dpad_down && !h.touch.isPressed())
            {
                if (slow2)
                {
                    h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    h.motorLift.setPower(-.6);

                }
                else
                {
                    h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    h.motorLift.setPower(-1);
                }
            }
            
            if((!gamepad2.dpad_up && h.touch.isPressed()) || (!gamepad2.dpad_up && !gamepad2.dpad_down))
            {
                h.motorLift.setTargetPosition(h.motorLift.getCurrentPosition());
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower(1);
            }
            /*if (h.touch.isPressed())
            {
                h.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }*/

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
            if(gamepad2.a) {
                h.servoIntakeClose.setPower(-1);
                h.servoIntakeFar.setPower(1);
            }
            //Intake
            else if(gamepad2.b)
            {
                    h.servoIntakeClose.setPower(1);
                    h.servoIntakeFar.setPower(-1);

            }
            //Stop Intake if not in use
            else
            {
                h.servoIntakeClose.setPower(0);
                h.servoIntakeFar.setPower(0);
            }

            //Turn table
            if(gamepad1.left_bumper)
            {
                h.motorTable.setPower(1);
            }
            else if(gamepad1.right_bumper)
            {
                h.motorTable.setPower(-1);
            }
            else
            {
                h.motorTable.setPower(0);
            }


            pressedLastIterationOuttake = pressedOutake;

            /*if (gamepad1.dpad_left) //Move table to the left of the robot
            {
                h.motorTable.setTargetPosition(LEFT_TABLE_POS);
                h.motorTable.setPower(1);
            }
            else if (gamepad1.dpad_right) //Move table to right of the robot
            {
                h.motorTable.setTargetPosition(RIGHT_TABLE_POS);
                h.motorTable.setPower(1);
            }
            else if (gamepad1.dpad_up) //Move table to in front of the robot
            {
                h.motorTable.setTargetPosition(FRONT_TABLE_POS);
                h.motorTable.setPower(1);
            }
            if(!h.motorTable.isBusy() && !(gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up))
            {
                h.motorTable.setPower(0);
            }*/

        }
    }
}
