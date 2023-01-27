package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "2023 TeleOp - CHOOSE THIS ONE", group = "TeleOp")
/**
 * Date Created:  7/30/2022
 * Purpose: This is going to be our main teleop for PowerPlay
 */
public class TeleOp2023 extends LinearOpMode {

    OpMode opmode;

    @Override
    public void runOpMode() {
        Hardware h = new Hardware();
        ElapsedTime outtake = new ElapsedTime();

        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        h.imu = hardwareMap.get(BNO055IMU.class, "imu");
        h.imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !h.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", h.imu.getCalibrationStatus().toString());
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

        final int LEFT_TABLE_POS = -1022;
        final int RIGHT_TABLE_POS = 1026;
        final int FRONT_TABLE_POS = 0;
        final int BACK_TABLE_POS = 2526;

        boolean dropping = false;

        double servoPos = 0;


        waitForStart();
        while (opModeIsActive()) {
            boolean pressedOutake = gamepad2.a;
            telemetry.addData("servoIntakeClose: ", h.servoIntakeClose.getPower());
            telemetry.addData("servoIntakeFar: ", h.servoIntakeFar.getPower());
            telemetry.addData("motorLift current Pos: ", h.motorLift.getCurrentPosition());
            telemetry.addData("touchSensor is pressed: ", h.touch.isPressed());
            telemetry.addData("motorTable current Pos: ", h.motorTable.getCurrentPosition());
            telemetry.addData("Intrinsic: ", h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Extrinsic: ", h.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            slow = gamepad1.right_trigger > 0.01;
            slow2 = gamepad2.y;

            /**Start drive system**/
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, slow, 5, 2);

            /*h.motorLift.setPower(-gamepad1.left_trigger);
            h.motorLift.setPower(gamepad1.right_trigger);*/

            //Motor Lift Controls
            if (gamepad2.dpad_up /* && h.motorLift <= UPPER_LIMIT */) {
                if (slow2) {
                    h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    h.motorLift.setPower(.7);

                } else {
                    h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    h.motorLift.setPower(1);
                }
            }
            if (gamepad2.dpad_down && !h.touch.isPressed()) {
                if (slow2) {
                    h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    h.motorLift.setPower(-.6);

                } else {
                    h.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    h.motorLift.setPower(-1);
                }
            }

            if ((!gamepad2.dpad_up && h.touch.isPressed()) || (!gamepad2.dpad_up && !gamepad2.dpad_down)) {
                h.motorLift.setTargetPosition(h.motorLift.getCurrentPosition());
                h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorLift.setPower(.5);
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
            //Intake
            if (gamepad2.b) {
                h.servoIntakeClose.setPower(-1);
                h.servoIntakeFar.setPower(1);
                dropping = false;
            }

            //Outtake
            if (pressedOutake && !pressedLastIterationOuttake) {
                dropping = true;
                outtake.reset();
            }
            if (outtake.time() < 1 && dropping) {
                h.servoIntakeClose.setPower(1);
                h.servoIntakeFar.setPower(-1);
            } else {
                dropping = false;
            }

            if (!gamepad2.a && !gamepad2.b && !dropping) {
                h.servoIntakeClose.setPower(0);
                h.servoIntakeFar.setPower(0);
            }


            pressedLastIterationOuttake = pressedOutake;

            //Turn table
            //if(gamepad2.left_bumper /* && h.motorLift <= UPPER_LIMIT */)
            /*{
                if (slow2)
                {
                    h.motorTable.setPower(.7);

                }
                else
                {
                    h.motorTable.setPower(1);
                }
            }
            else if(gamepad2.right_bumper)
            {
                if (slow2)
                {
                    h.motorTable.setPower(-.6);

                }
                else
                {
                    h.motorTable.setPower(-1);
                }
            }
            else
            {
                h.motorTable.setPower(0);
            }*/


            pressedLastIterationOuttake = pressedOutake;

            //Turn Table Manual

            if (gamepad2.right_stick_x > 0.01) {
                h.motorTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                h.motorTable.setPower(.4);
            } else if (gamepad2.right_stick_x < -0.01) {
                h.motorTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                h.motorTable.setPower(-.4);
            }

            //Turn Table Auto
            if (gamepad2.left_bumper) //Move table to the left of the robot
            {
                h.motorTable.setTargetPosition(LEFT_TABLE_POS);
                h.motorTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorTable.setPower(1);
            } else if (gamepad2.left_trigger > 0.01) //Move table to the right of the robot
            {
                h.motorTable.setTargetPosition(RIGHT_TABLE_POS);
                h.motorTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorTable.setPower(1);
            } else if (gamepad2.right_trigger > 0.01) //Move table to in front of the robot
            {
                h.motorTable.setTargetPosition(FRONT_TABLE_POS);
                h.motorTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorTable.setPower(1);
            } else if (gamepad2.right_bumper) //Move table to the back of the robot
            {
                h.motorTable.setTargetPosition(BACK_TABLE_POS);
                h.motorTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorTable.setPower(1);
            }
            if (!h.motorTable.isBusy() && !(gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down) && !(gamepad2.right_stick_x > 0.01) && !(gamepad2.right_stick_x < -0.01))
            {
                h.motorTable.setPower(0);
            }

            if (gamepad2.dpad_left)
            {
                h.servoExtension.setPower(1);
            }
            if (gamepad2.dpad_right)
            {
                h.servoExtension.setPower(-1);
            }
            if(!gamepad2.dpad_left && !gamepad2.dpad_right)
            {
                h.servoExtension.setPower(0);
            }

        }
    }
}
// Loren was here hi hou7 r u?///