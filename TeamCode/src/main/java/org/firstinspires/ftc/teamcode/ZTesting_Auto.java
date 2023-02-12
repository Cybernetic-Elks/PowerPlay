package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Hardware.Kd;
import static org.firstinspires.ftc.teamcode.Hardware.Ki;
import static org.firstinspires.ftc.teamcode.Hardware.Kp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="ZTesting Auto", group="Auto")
public class ZTesting_Auto extends LinearOpMode {
    static Hardware h = new Hardware();
    public static PIDController pidController = new PIDController(h.Kp,h.Ki,h.Kd,.25);


    public static double targetPos = 1000;
    public double targetAngle = 90;
    double output = 0;
    public static double PID_max = .3;
    double slow = .25;
    double rotateSpeed = .35;
    double rotateSpeedSlow = .25;
    double leftPower = 0;
    double rightPower = 0;

    //PIDController pid = new PIDController();
    OpenCvCamera webCam;
    @Override
    public void runOpMode() throws InterruptedException {
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



        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", h.imu.getCalibrationStatus().toString());
        telemetry.update();


        waitForStart();

        /*h.setDrivePower((float).5);
        h.sleep(5000);
        h.setDrivePower(0);*/

        h.resetHeading();

        while(opModeIsActive())
        {
            if(Math.abs(h.getRawHeading()) <= 3)
            {
                leftPower = slow - (h.getRawHeading() / 15);
                rightPower = slow - (h.getRawHeading() / 15);
                telemetry.addLine("IMU is within 3 degrees of 0");
                telemetry.addData("leftPower: ", leftPower);
                telemetry.addData("rightPower: ", rightPower);
                telemetry.update();
            }
            else if (Math.abs(h.getRawHeading()) < 10)
            {
                telemetry.addLine("IMU is within 10 degrees of 0");

                if (h.getIntegratedHeading() > 0) {
                    telemetry.addLine("IMU is positive, accelerating right motor");
                    leftPower = slow;
                    rightPower = 1.1 * slow;
                } else if (h.getIntegratedHeading() < 0) {
                    telemetry.addLine("IMU is positive, accelerating left motor");
                    leftPower = 1.1 * slow;
                    rightPower = slow;
                }
                telemetry.addData("leftPower: ", leftPower);
                telemetry.addData("rightPower: ", rightPower);
                telemetry.update();
            }
            else
            {
                telemetry.addLine("IMU is farther then 10 degrees of 0");
                if(h.getIntegratedHeading() > 0)
                {
                    while(h.getIntegratedHeading() > 10 && !isStopRequested())
                    {
                        telemetry.addLine("IMU is greater then 10: turning left");
                        telemetry.update();
                        leftPower = -rotateSpeed;
                        rightPower = rotateSpeed;
                    }
                }
                while (h.getIntegratedHeading() > 0 && !isStopRequested())
                {
                    telemetry.addLine("IMU is > 0, slowly turning left");
                    telemetry.update();
                    leftPower = -rotateSpeedSlow;
                    rightPower = rotateSpeedSlow;
                }
                while (h.getIntegratedHeading() < 0 && !isStopRequested())
                {
                    telemetry.addLine("IMU is < 0, rotating right");
                    telemetry.update();
                    leftPower = rotateSpeedSlow;
                    rightPower = -rotateSpeedSlow;
                }
            }
            /*else
            {
                while (gyro.getAngle() ＜-10 && isAutonomous())
                {
                victorLeft.set(rotateSpeed);
                victorRight.set(rotateSpeed);
                }
                while (gyro.getAngle() ＜0 && isAutonomous())
                {
                victorLeft.set(rotateSpeedSlow);
                victorRight.set(rotateSpeedSlow);
                }
                while (gyro.getAngle() ＞0 && isAutonomous())
                {
                victorLeft.set(-rotateSpeedSlow);
                victorRight.set(-rotateSpeedSlow);
                }
            }*/;
            telemetry.addData("leftPower: ", leftPower);
            telemetry.addData("rightPower: ", rightPower);
            telemetry.addData("IMU: ", h.getRawHeading());
            telemetry.update();
            h.motorFrontLeft.setPower(leftPower);
            h.motorBackLeft.setPower(leftPower);
            h.motorFrontRight.setPower(rightPower);
            h.motorBackRight.setPower(rightPower);
        }

    }
}
// Hello this is Loren just wanting to say hello and your doing great.