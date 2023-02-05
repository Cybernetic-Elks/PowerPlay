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
        h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        h.resetHeading();

        /*h.drivePureEncoder(true, h.calculateTicks(10), .3);
        h.sleep(500);*/

        h.driveStraight(.5, 25, 0.0);
        h.sleep(5000);

        /*h.drivePureEncoder(true, h.calculateTicks(10), .3);
       h.sleep(500);*/

    }
}
// Hello this is Loren just wanting to say hello and your doing great.