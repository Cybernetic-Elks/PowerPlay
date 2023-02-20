/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByGyro_Linear;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="LEFT POWERPLAY", group = "Auto")
public class LeftPowerPlayAuto extends LinearOpMode
{
    Hardware h = new Hardware();
    public enum Side
    {
        LEFT,
        RIGHT,
        MIDDLE
    }
    Side parkingSide;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    RobotAutoDriveByGyro_Linear driveByGyro_linear;

    static final double FEET_PER_METER = 3.28084;

    double motorPower;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    final int LEFT = 1;
    final int MIDDLE = 2;
    final int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error", "Camera not able to open");
            }
        });

        telemetry.setMsTransmissionInterval(50);


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        h.imu = hardwareMap.get(BNO055IMU.class, "imu");
        h.imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !h.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Tag Detection looping");
        telemetry.addData("imu calib status", h.imu.getCalibrationStatus().toString());
        telemetry.update();

        h.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("See tags, but don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            parkingSide = Side.LEFT;
        }
        else
        {
            /* Actually do something useful */
            switch (tagOfInterest.id)
            {
                case LEFT: //LEFT tag is showing so I should park on the left.
                    parkingSide = Side.LEFT;
                    break;
                case MIDDLE: //MIDDLE tag is showing so I should park on the left.
                    parkingSide = Side.MIDDLE;
                    break;
                case RIGHT: //RIGHT tag is showing so I should park on the left.
                    parkingSide = Side.RIGHT;
                    break;
                default: //tagOfInterest wasn't found so go LEFT and hope for the best.
                    parkingSide = Side.LEFT;
                    break;
            }
        }
        telemetry.addData("Parking: ", parkingSide);
        telemetry.update();


        //Start raising arm to low tower position
        h.motorLift.setTargetPosition(1300);
        h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorLift.setPower(1);

        //Align at center of mat
        /*h.drivePureEncoder(true,h.calculateTicks(2),.2);
        h.sleep(2500);*/

        h.strafePureEncoder(true,h.calculateTicks(5),.35);
        h.sleep(750);

        //Drive to row of the high pole
        //h.driveStraight(.3,55,0);
        //h.drivePureEncoder(true,h.calculateTicks(55),.2);
        h.driveHeading(60,0,.2);
        h.sleep(250);

        h.drivePureEncoder(false,h.calculateTicks(5),.2);
        h.sleep(250);

        //Line up with high pole
        h.strafePureEncoder(true,h.calculateTicks(12),.3);
        h.sleep(230);

        //Start raising arm to high tower position
        h.motorLift.setTargetPosition(5300);
        h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorLift.setPower(1);
        h.sleep(2500);

        //Run towards the pole
        h.drivePureEncoder(true,h.calculateTicks(8),.2);
        h.sleep(250);

        //Back up a bit to get better alignment
        h.drivePureEncoder(false,h.calculateTicks(2.25),.2);
        h.sleep(250);

        //Drop cone
        h.servoIntakeClose.setPower(-1);
        h.servoIntakeFar.setPower(1);
        h.sleep(1700);
        h.servoIntakeClose.setPower(0);
        h.servoIntakeFar.setPower(0);

        h.drivePureEncoder(false, h.calculateTicks(5),.3);
        h.sleep(250);

        //Start lowering arm to ground tower position
        h.motorLift.setTargetPosition(1300);
        h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorLift.setPower(1);

        h.sleep(500);


        h.strafePureEncoder(true,h.calculateTicks(5.5),.2);
        h.sleep(2500);


        h.turnIMU(90,.3,.2);


        //Park in correct zone
        h.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        switch (parkingSide)
        {
            case LEFT:
                h.setDrivePower((float).07);
                while(Math.abs(5 - h.distance.getDistance(DistanceUnit.INCH)) >= 0.3 ) {
                    PIDController pidController = new PIDController(.03,0,0,.25);
                    double correction = pidController.output(90,h.getRawHeading());
                    motorPower = 5 - h.distance.getDistance(DistanceUnit.INCH) > 0 ? -.1 : .1;

                    double leftPower = Range.clip(motorPower - correction, -1, 1);
                    double rightPower = Range.clip(motorPower + correction,-1,1);

                    h.setIndividualDrivePower(leftPower, leftPower, rightPower, rightPower);
                    telemetry.addLine("Driving...");
                    telemetry.addData("Motor power", h.motorFrontRight.getPower());
                    telemetry.addData("Current Distance", h.distance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                break;
            case MIDDLE:
                h.setDrivePower((float).07);
                while(Math.abs(33.5 - h.distance.getDistance(DistanceUnit.INCH)) >= 0.3 ) {
                    PIDController pidController = new PIDController(.03,0,0,.25);
                    double correction = pidController.output(90,h.getRawHeading());
                    motorPower = 33.5 - h.distance.getDistance(DistanceUnit.INCH) > 0 ? -.1 : .1;;

                    double leftPower = Range.clip(motorPower - correction, -1, 1);
                    double rightPower = Range.clip(motorPower + correction,-1,1);

                    h.setIndividualDrivePower(leftPower, leftPower, rightPower, rightPower);
                    telemetry.addLine("Driving...");
                    telemetry.addData("Motor power", h.motorFrontRight.getPower());
                    telemetry.addData("Current Distance", h.distance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                h.strafePureEncoder(true, h.calculateTicks(2), .2);
                h.sleep(500);
                break;
            case RIGHT:
                h.drivePureEncoder(false,h.calculateTicks(8),.2);
                h.sleep(250);
                h.strafePureEncoder(true, h.calculateTicks(4), .2);
                h.sleep(500);
                break;
        }
        h.setDrivePower(0);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}