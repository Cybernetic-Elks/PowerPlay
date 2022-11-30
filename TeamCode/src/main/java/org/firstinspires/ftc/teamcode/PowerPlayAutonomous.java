package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="POWERPLAY", group="Auto")
public class PowerPlayAutonomous extends LinearOpMode {
    Hardware h = new Hardware();
    OpenCvCamera webCam;
    public enum Side {
        ONE,
        TWO,
        THREE
    }
    Side side;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SignalDetector detector = new SignalDetector(telemetry);
        webCam.setPipeline(detector);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error", "Camera not able to open");
            }
        });
        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", h.imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();
        webCam.stopStreaming();
            telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());
            telemetry.addData("Color:", detector.getSide());
            telemetry.update();
            switch (detector.getSide()) {
                case GREEN: //bottom reversed if blue
                    side = Side.ONE;
                    break;
                case PURPLE://middle reversed if blue
                    side = Side.TWO;
                    break;
                case YELLOW://top reversed if blue
                    side = Side.THREE;
            }
            telemetry.addData("ZONE:", detector.getSide());
            //Start raising arm to low tower position
            h.motorLift.setTargetPosition(4041);
            h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorLift.setPower(1);

            //Drive straight forward to be in line with low pole
            h.strafePureEncoder(true, h.calculateTicks(4),.5);
            h.sleep(2000);
            h.drivePureEncoder(true, h.calculateTicks(13),.25);
            h.sleep(1500);

            //Turn to line up with the low pole
            h.turnIMU(-90,.5,.2);
            h.sleep(1000);

            //Drive towards pole to ready for drop
            h.drivePureEncoder(true, h.calculateTicks(6),.6);
            h.sleep(1500);

            //Drop cone
            h.servoIntakeClose.setPower(-1);
            h.servoIntakeFar.setPower(1);
            h.sleep(1500);
            h.servoIntakeClose.setPower(0);
            h.servoIntakeFar.setPower(0);

            //Return to mid parking zone.
            h.drivePureEncoder(false, h.calculateTicks(6), .6);
            h.sleep(1000);

            h.turnIMU(90,.5,.2);
            h.sleep(2000);

            h.strafePureEncoder(true, h.calculateTicks(15),.5);
            h.sleep(2500);

            h.drivePureEncoder(true, h.calculateTicks(24),.25);
            h.sleep(2000);

            h.motorLift.setTargetPosition(4041);
            h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorLift.setPower(1);
            
            h.motorLift.setPower(-1);
            if(h.touch.isPressed())
            {
                h.motorLift.setPower(0);
            }
            h.sleep(10000);

            //Park in correct zone
            switch (side)
            {
                case ONE:
                    /*h.strafePureEncoder(false, h.calculateTicks(24),.5);
                    h.sleep(2500);
                    h.drivePureEncoder(true, h.calculateTicks(24),.4);*/
                    break;
                case TWO:
                    /*h.strafePureEncoder(true, h.calculateTicks(4),.5);
                    h.sleep(2500);
                    h.drivePureEncoder(true, h.calculateTicks(24),.6);*/
                    break;
                case THREE:
                    /*h.strafePureEncoder(true, h.calculateTicks(28),.5);
                    h.sleep(2500);
                    h.drivePureEncoder(true, h.calculateTicks(24),.4);
                    */
                    break;
            }
    }
}
