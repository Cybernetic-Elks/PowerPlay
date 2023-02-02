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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name=" Proto powerplay", group = "Auto")
public class PowerPlayTagAuto extends LinearOpMode
{
    Hardware h = new Hardware();
    ElapsedTime elapsedTime = new ElapsedTime();

    public enum Side
    {
        LEFT,
        RIGHT,
        MIDDLE
    }
    Side parkingSide;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        h.drivePureEncoder(true,h.calculateTicks(60),.7);

        h.sleep(250);

        h.drivePureEncoder(false, h.calculateTicks(7),.4);

        h.sleep(250);

        //TODO a bit off might need a PID (more likely PD) or messing with the powers to try and get it more accurate
        h.turnIMU(-90, .35,.15);

        h.servoExtension.setPower(-1);

        h.motorTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double error = 57 - h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //TODO The table can be turned a bit less as well to correct for the misalignment
        while(Math.abs(error) >= 1 ) {
            error = 57 - h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double tablePower = error > 0 ? -.30 : .30;
            h.motorTable.setPower(tablePower);

            telemetry.addLine("Turning...");
            telemetry.addData("tablePower: ", tablePower);
            telemetry.addData("Current Angle", h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("error: ", error);
            telemetry.update();
        }
        h.motorTable.setPower(0);
        /*double error = -90 - h.getIntegratedHeading();
        while(Math.abs(error) >= 1 ) {
            error = -90 - h.getIntegratedHeading();
            double turnPower = error < 0 ? -.3 : .3;
            h.motorFrontLeft.setPower(-turnPower);
            h.motorBackLeft.setPower(-turnPower);
            h.motorFrontRight.setPower(turnPower);
            h.motorBackRight.setPower(turnPower);

            telemetry.addLine("Turning...");
            telemetry.addData("turnPower: ", turnPower);
            telemetry.addData("Current Angle", h.getIntegratedHeading());
            telemetry.addData("error: ", error);
            telemetry.update();
        }
        h.setDrivePower(0);*/

        h.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        h.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        h.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        h.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        h.drivePureEncoder(true, h.calculateTicks(13), .6);

        h.sleep(900);

        h.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        h.motorLift.setTargetPosition(5500);


        //double distanceError = target - curr;

        //h.drivePureEncoder(true, h.calculateTicks(6.2), .3);

        h.servoExtension.setPower(0);

        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        h.setDrivePower((float)-.07);
        while(Math.abs(17.5 - h.distance.getDistance(DistanceUnit.INCH)) >= .3 ) {

            telemetry.addLine("Driving...");
            telemetry.addData("Motor power", h.motorFrontRight.getPower());
            telemetry.addData("Current Distance", h.distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        h.setDrivePower(0);

        h.sleep(200);

        h.servoExtension.setPower(1);
        h.sleep(4000);
        /*elapsedTime.reset();
        while(elapsedTime.time() < 5) {

        }*/
        h.servoExtension.setPower(0);
        /*elapsedTime.reset();
        while(elapsedTime.time() < 5) {

        }*/

        h.servoIntakeClose.setPower(-1);
        h.servoIntakeFar.setPower(1);
        h.sleep(1400);
        h.servoIntakeClose.setPower(0);
        h.servoIntakeFar.setPower(0);

        for (int i = 0; i < 1; i++)
        {
            h.motorTable.setPower(.2);

            h.sleep(1000);
            h.motorLift.setTargetPosition(1220);
            h.motorLift.setPower(1);
            h.servoExtension.setPower(-1);

            error = 2;
            while(Math.abs(error) >= 1 ) {
                error = -90 - h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double tablePower = error > 0 ? -.30 : .30;
                h.motorTable.setPower(tablePower);

                telemetry.addLine("Turning to -90...");
                telemetry.addData("tablePower: ", tablePower);
                telemetry.addData("Current Angle", h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("error: ", error);
                telemetry.update();
            }
            h.motorTable.setPower(0);
            h.motorLift.setPower(0);

            h.servoExtension.setPower(1);
            h.sleep(4000);
            h.servoExtension.setPower(0);

            h.drivePureEncoder(true,h.calculateTicks(2),.1);

            h.servoIntakeClose.setPower(1);
            h.servoIntakeFar.setPower(-1);
            h.sleep(1400);
            h.servoIntakeClose.setPower(0);
            h.servoIntakeFar.setPower(0);

            h.servoExtension.setPower(-1);

            h.motorLift.setTargetPosition(5500);

            h.drivePureEncoder(false,h.calculateTicks(2),.1);
            
            error = 2;
            while(Math.abs(error) >= 1 ) {
                error = 57 - h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double tablePower = error > 0 ? -.30 : .30;
                h.motorTable.setPower(tablePower);

                telemetry.addLine("Turning to 0...");
                telemetry.addData("tablePower: ", tablePower);
                telemetry.addData("Current Angle", h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("error: ", error);
                telemetry.update();
            }

            h.servoExtension.setPower(1);
            h.sleep(4000);
            h.servoExtension.setPower(0);

            h.servoIntakeClose.setPower(-1);
            h.servoIntakeFar.setPower(1);
            h.sleep(1700);
            h.servoIntakeClose.setPower(0);
            h.servoIntakeFar.setPower(0);

        }
        //Park in correct zone
        /*switch (parkingSide)
        {
            case LEFT:

                h.strafePureEncoder(true, h.calculateTicks(16),.5);
                h.sleep(2500);
                h.drivePureEncoder(true, h.calculateTicks(46),.4);
                h.sleep(2500);
                h.turnIMU(90,.4,.2);
                h.sleep(1000);
                h.drive(true,41,.5);
                break;
            case MIDDLE:
                h.strafePureEncoder(true, h.calculateTicks(16),.5);
                h.sleep(2500);
                h.drivePureEncoder(true, h.calculateTicks(46),.4);
                h.sleep(2500);
                h.turnIMU(90,.4,.2);
                h.sleep(1000);
                h.drive(true,17,.5);

                break;
            case RIGHT:
                h.strafePureEncoder(true, h.calculateTicks(16),.5);
                h.sleep(2500);
                h.drivePureEncoder(true, h.calculateTicks(33),.5);
                h.sleep(2500);
                h.turnIMU(90,.4,.2);
                h.sleep(1000);
                break;
        }*/
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