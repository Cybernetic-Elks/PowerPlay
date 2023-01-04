package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
//TODO Make it go to a high pole
@Disabled
@Autonomous(name="LEFT POWERPLAY OLD", group="Auto")
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
            h.motorLift.setTargetPosition(2000);
            h.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorLift.setPower(1);

            h.drivePureEncoder(true,h.calculateTicks(4),.2);
            h.sleep(2500);

            h.strafePureEncoder(true,h.calculateTicks(15),.5);
            h.sleep(2500);

            h.drivePureEncoder(true,h.calculateTicks(6),.2);
            h.sleep(2500);

            //Drop cone
            h.servoIntakeClose.setPower(-1);
            h.servoIntakeFar.setPower(1);
            h.sleep(1700);
            h.servoIntakeClose.setPower(0);
            h.servoIntakeFar.setPower(0);

            h.drivePureEncoder(false, h.calculateTicks(5),.6);
            h.sleep(2500);
            //Park in correct zone
            switch (side)
            {
                case ONE:
                    h.strafePureEncoder(true, h.calculateTicks(14),.5);
                    h.sleep(2500);
                    h.drivePureEncoder(true, h.calculateTicks(44),.4);
                    h.sleep(2500);
                    h.turnIMU(90,.5,.3);
                    h.sleep(1000);
                    h.drive(true,40,.5);
                    //TODO make it drive a bit more forward maybe
                    break;
                case TWO:
                    h.strafePureEncoder(true, h.calculateTicks(14),.5);
                    h.sleep(2500);
                    h.drivePureEncoder(true, h.calculateTicks(44),.4);
                    h.sleep(2500);
                    h.turnIMU(90,.5,.3);
                    h.sleep(1000);
                    h.drive(true,18,.5);

                    break;
                case THREE:
                    h.strafePureEncoder(true, h.calculateTicks(14),.5);
                    h.sleep(2500);
                    h.drivePureEncoder(true, h.calculateTicks(44),.5);
                    h.sleep(2500);
                    h.turnIMU(90,.5,.3);
                    h.sleep(1000);
                    break;
            }
    }
}
