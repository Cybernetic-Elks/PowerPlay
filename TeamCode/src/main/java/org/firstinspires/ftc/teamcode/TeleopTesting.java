package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Testing", group = "TeleOp")
/**
 * Date Created: 11/19/2022
 * Purpose: Testing teleop code before moving it into the main teleop
 * */
public class TeleopTesting extends LinearOpMode
{
    OpMode opmode;

    public enum TableMovementState {
        GOING_FRONT,
        GOING_LEFT,
        GOING_RIGHT,
        GOING_BACK,
        RESTING
    }
    public enum TableState {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }
    @Override
    public void runOpMode() {
        Hardware h = new Hardware();
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime outtake = new ElapsedTime();

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
        boolean dropping = false;

        final int LEFT_TABLE_POS = -1022;
        final int RIGHT_TABLE_POS = 1026;
        final int FRONT_TABLE_POS = 0;
        final int BACK_TABLE_POS = 2526;
        boolean turnTabling;

        double currentArmDegree;
        /*
        public enum IntakeState =
        {
            DROPPING,
            STOPPED,
            GRABBING
        }
        IntakeState outtakeState = STOPPED;
         */


        waitForStart();
        while (opModeIsActive()) {
            boolean pressedOutake = gamepad2.a;
            //telemetry.addData("Intake State: ", outtakeState;
            telemetry.addData("Intrinsic: ", h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Extrinsic: ", h.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("motorLift current Pos: ", h.motorLift.getCurrentPosition());
            telemetry.addData("touchSensor is pressed: ", h.touch.isPressed());
            telemetry.update();
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            slow = gamepad1.right_trigger > 0.01;
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
            //Motor Lift Controls
            if(gamepad2.dpad_up /* && h.motorLift <= UPPER_LIMIT */)
            {
                if (slow2)
                {
                    h.motorLift.setPower(.7);
                    h.motorLift2.setPower(.7);

                }
                else
                {
                    h.motorLift.setPower(1);
                    h.motorLift2.setPower(1);
                }
            }
            if(gamepad2.dpad_down && !h.touch.isPressed())
            {
                if (slow2)
                {
                    h.motorLift.setPower(-.6);
                    h.motorLift2.setPower(-.6);

                }
                else
                {
                    h.motorLift.setPower(-1);
                    h.motorLift2.setPower(-1);
                }
            }
            if((!gamepad2.dpad_up && h.touch.isPressed()) || (!gamepad2.dpad_up && !gamepad2.dpad_down))
            {
                h.motorLift.setPower(0);
                h.motorLift2.setPower(0);
            }


            //Auto Arm
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


            //Intake FSM
            /*
                switch(IntakeState)
                {
                    case STOPPED:
                        if (gamepad2.b) //If gampad2.b Start GRABBING
                        {
                            outtakeState = IntakeState.GRABBING;
                        }
                        else if (pressedOutake && !pressedLastIterationOuttake) //if gamepad2.a start DROPPING
                        {
                            outtake.reset();
                            outtakeState = IntakeState.DROPPING;
                        }
                        else //Stop servos if no buttons are pressed and not dropping or grabbing
                        {
                            h.servoIntakeClose.setPower(0);
                            h.servoIntakeFar.setPower(0);
                        }
                    break;
                    case GRABBING: //Run intake while B is held
                        h.servoIntakeClose.setPower(1);
                        h.servoIntakeFar.setPower(-1);
                        outtakeState = IntakeState.STOPPED;
                    break;
                    case DROPPING: //Run outtake until timer is up or B is pressed
                        if (outtake.time() < 1)
                                {
                                    h.servoIntakeClose.setPower(-1);
                                    h.servoIntakeFar.setPower(1);
                                }
                        else if (gamepad2.b)
                        {
                            outtakeState = IntakeState.INTAKE;
                        }
                        else
                        {
                            outtakeState = IntakeState.STOPPED;
                        }
                    break;
                }*/



            //Intake
            if(gamepad2.b)
            {
                h.servoIntakeClose.setPower(1);
                h.servoIntakeFar.setPower(-1);
                dropping = false;
            }

            //Outtake
            if (pressedOutake && !pressedLastIterationOuttake)
            {
                dropping = true;
                outtake.reset();
            }
            if (outtake.time() < 1 && dropping)
            {
                h.servoIntakeClose.setPower(-1);
                h.servoIntakeFar.setPower(1);
            }
            else
            {
                dropping = false;
            }

            if(!gamepad2.a && !gamepad2.b && !dropping)
            {
                h.servoIntakeClose.setPower(0);
                h.servoIntakeFar.setPower(0);
            }


            pressedLastIterationOuttake = pressedOutake;

            //TurnTable
            if (gamepad1.left_bumper) //Move table to the left of the robot
            {
                h.motorTable.setTargetPosition(LEFT_TABLE_POS);
                h.motorTable.setPower(1);
            }
            else if (gamepad1.left_trigger > 0.01) //Move table to the right of the robot
            {
                h.motorTable.setTargetPosition(RIGHT_TABLE_POS);
                h.motorTable.setPower(1);
            }
            else if (gamepad1.right_trigger > 0.01) //Move table to in front of the robot
            {
                h.motorTable.setTargetPosition(FRONT_TABLE_POS);
                h.motorTable.setPower(1);
            }
            else if (gamepad2.right_bumper) //Move table to the back of the robot
            {
                h.motorTable.setTargetPosition(BACK_TABLE_POS);
                h.motorTable.setPower(1);
            }
            if(!h.motorTable.isBusy() && !(gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down))
            {
                h.motorTable.setPower(0);
            }





        }
    }
}
