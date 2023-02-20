package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Config
public class Hardware extends LinearOpMode
{
    /**
     * Programmer:    Aiden Smith (Sean Pakros after Aiden has left the team)
     * Date Created:  Sometime in 2017?
     * Purpose:       Contains all our hardware and functions, basically everything we need to run.
     **/

    /** Setup hardware variables **/
    public DcMotor motorFrontRight;//In use
    public DcMotor motorBackRight;//In use
    public DcMotor motorBackLeft;//In use
    public DcMotor motorFrontLeft;//In use
    public DcMotor motorCarousel;
    public DcMotor motorArm;
    public DcMotor motorWinch;
    public DcMotor motorLift;    //In use
    public DcMotor motorLift2;   //In use
    public DcMotor motorTable;   //In use

    Servo servoIntake;
    Servo servoWrist;


    //CRServo servoWrist;
    CRServo servoIntakeClose; //In use
    CRServo servoIntakeFar; //In use
    CRServo servoExtension;

    //ColorSensor colorSensor;
    DistanceSensor distance;
    //Rev2mDistanceSensor distance;
    TouchSensor touch; //In use

    ModernRoboticsI2cGyro MRgyro;
    ModernRoboticsI2cRangeSensor MRRange;

    BNO055IMU imu;
    Orientation angles;
    private double previousHeading = 0;
    private double integratedHeading = 0;

    DcMotor.RunMode initialMode = null;

    public static double Kp = 0.03;
    public static double Kd = 0;
    public static double Ki = 0;

    int driveTime;

    //software elements mainly for field relative
    double rotate;
    double forward;
    double strafe;
    double gyro_degrees;
    double gyro_radians;
    double temp;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = .5;
    private double  turnSpeed     = 0;
    double frontLeftSpeed = 0;
    double frontRightSpeed = 0;
    double backLeftSpeed = 0;
    double backRightSpeed = 0;
    int frontLeftTarget = 0;
    int backLeftTarget = 0;
    int frontRightTarget = 0;
    int backRightTarget = 0;


    /**
     * The variables and Equation used to turn the current wheels' encoders into inches
     */
    final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: Gobuilda 13.7:1 Ratio, 435 RPM
    final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP (2 rotations at input = 1 at output)
    final double     WHEEL_DIAMETER_INCHES   = 96 / 25.4;     // For figuring circumference (96mm / 25.4 = ~3.78)
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    //                                                             384.5                       /     11.8677        ~=      32.3988
    //

    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.0208;     // Larger is more responsive, but also less stable

    private Telemetry telemetry;


    @Override
    public void runOpMode()
    {
    }

    public void init(HardwareMap aMap, Telemetry inputTelemetry)
    {
        telemetry = inputTelemetry;
        
        motorFrontRight = aMap.dcMotor.get("motorFrontRight");
        motorBackRight = aMap.dcMotor.get("motorBackRight");
        motorBackLeft = aMap.dcMotor.get("motorBackLeft");
        motorFrontLeft = aMap.dcMotor.get("motorFrontLeft");

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

        touch = aMap.touchSensor.get("touchSensor");
        distance = aMap.get(DistanceSensor.class, "distance");
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;


        servoIntakeClose = aMap.crservo.get("servoIntakeClose");
        servoIntakeFar = aMap.crservo.get("servoIntakeFar");
        servoExtension = aMap.crservo.get("servoExtension");

        motorLift = aMap.dcMotor.get("motorLift");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);







        /*
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         */



        /*motorLift2 = aMap.dcMotor.get("motorLift2");
        motorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift2.setDirection(DcMotorSimple.Direction.REVERSE);*/

        motorTable = aMap.dcMotor.get("motorTable");
        motorTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drives the robot forward/backwards a set number of inches at a set power.
     *
     * <p>Issues: Recently fixed the inch being innacurate now it is about three inches over every time, need to look into this some.</p>
     *
     *
     * @param forward sets direction the robot will drive in
     * @param distanceInches distance in inches the robot will drive [positive]
     * @param power power it will drive at [-1,1]
     */
    public void drive(boolean forward, int distanceInches, double power)
    {

        int distanceEncodeVal;

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //distanceEncodeVal = (int) Math.round((distanceInches/(4* Math.PI))*1120);
        distanceEncodeVal = (int) Math.round((distanceInches*(COUNTS_PER_INCH)));
        driveTime = (distanceInches/10)*1000;



        if(forward)
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */

        //telemetry.addData("Running", "...");
        //telemetry.update();

        if(forward)
        {

            while (motorFrontRight.getCurrentPosition() < distanceEncodeVal - 20 && !isStopRequested())
            {

            }
        }
        else
        {

            while (motorFrontRight.getCurrentPosition() > -distanceEncodeVal + 20 && !isStopRequested())
            {

            }

        }

        //telemetry.addData("Finished", ".");
        //telemetry.update();



        /*motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);*/

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);


    }

    /**
     * Drives the robot forward/backwards a set number of encoderticks at a set power.
     *
     * <p>Issues: Working fine right now, fairly consistent</p>
     *
     *
     * @param forward sets direction the robot will drive in
     * @param distanceEncodeVal distance in encoder ticks the robot will drive [0,∞?]
     * @param power power it will drive at [-1,1]
     */
    public void drivePureEncoder(boolean forward, int distanceEncodeVal, double power)
    {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(forward)
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */

        //telemetry.addData("Running", "...");
        //telemetry.update();

        if(forward)
        {

            while (motorFrontRight.getCurrentPosition() < distanceEncodeVal - 20 && !isStopRequested())
            {

            }
        }
        else
        {

            while (motorFrontRight.getCurrentPosition() > -distanceEncodeVal + 20 && !isStopRequested())
            {

            }

        }

        //telemetry.addData("Finished", ".");
        //telemetry.update();



        /*motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);*/

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);


    }

    /**
     * Drives the robot left/right a set number of inches at a set power
     *
     * <p>Issues: Gets stuck running sometimes as it never reaches the target value. I'm not using this right now till I fix it.
     * Inch values are also not accurate probably due to switching the wheels.</p>
     *
     *
     *
     * @param left direction the robot will drive
     * @param distanceInches distance in inches the robot will drive [0,∞?]
     * @param power power it will drive at [-1,1]
     */
    public void strafe(boolean left, int distanceInches,double power)
    {

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distanceEncodeVal = (int) Math.round((distanceInches*(COUNTS_PER_INCH)));


        if(left)
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */
        if(left)
        {

            while (motorFrontRight.getCurrentPosition() > distanceEncodeVal - 20 && !isStopRequested()/* && end > System.currentTimeMillis()*/)
            {

            }

        }
        else
        {

            while (motorFrontRight.getCurrentPosition() < -distanceEncodeVal + 20 && !isStopRequested()/* && end > System.currentTimeMillis()*/)
            {

            }

        }



        /*motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);*/

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    /**
     * Drives the robot left/right a set number of inches at a set power
     *
     * <p>Issues: Seems to be working fine though I need to do some further testing on it
     *
     *
     *
     * @param left direction the robot will drive
     * @param distanceEncodeVal distance in inches the robot will drive [0,∞?]
     * @param power power it will drive at [-1,1]
     */
    public void strafePureEncoder(boolean left, int distanceEncodeVal,double power)
    {

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Directions on this might be wrong
        if(left)
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower((float) power);

        while (opModeIsActive() /*&& (runtime.seconds() < timeoutS)*/ && (motorFrontLeft.isBusy() && motorBackRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to",  " %7d", distanceEncodeVal);
            telemetry.addData("Currently at",  " at %7d :%7d", motorFrontLeft.getCurrentPosition(), motorBackRight.getCurrentPosition());
            telemetry.update();
        }


        telemetry.addData("FRMotor Pos", motorFrontRight.getCurrentPosition());
        telemetry.addData("DistanceEncodeVal", distanceEncodeVal);
        telemetry.addData("FRMotor Target Pos", motorFrontRight.getTargetPosition());
        telemetry.update();

    }

    /**
     * Controls the teleop movement of the robot. Allows us to go forward and backward, turn, strafe,
     * and go in diagonals by moving the stick diagonally
     *
     * Issues: Works beautifully so far
     *
     * @param joystickX x value of the joystick, this is used for strafing
     * @param joystickY y value of the joystick this is used for forward/backwards movement
     * @param rotation x value of right joystick, used for turning
     */
    public void driveOmniDir(double joystickX, double joystickY, double rotation, boolean slow, double slowFactor, double baseFactor)
    {
        /**                   | Forward and|
         *                    | Backwards  | Strafing | Turning |  */
        if(!slow)
        {
            motorFrontRight.setPower((-joystickY - joystickX - rotation) * baseFactor);
            motorBackRight.setPower((-joystickY + joystickX - rotation) * baseFactor);
            motorFrontLeft.setPower((-joystickY + joystickX + rotation) * baseFactor);
            motorBackLeft.setPower((-joystickY - joystickX + rotation) * baseFactor);
        }
        else
        {
            motorFrontRight.setPower((-joystickY - joystickX - rotation) / slowFactor);
            motorBackRight.setPower((-joystickY + joystickX - rotation) / slowFactor);
            motorFrontLeft.setPower((-joystickY + joystickX + rotation) / slowFactor);
            motorBackLeft.setPower((-joystickY - joystickX + rotation) / slowFactor);
        }


         /**motorFrontRight.setPower(--1 - joystickX - rotation);
         motorBackRight.setPower(--1 + joystickX - rotation);
         motorFrontLeft.setPower(--1 + joystickX + rotation);
         motorBackLeft.setPower(--1 - joystickX + rotation);**/

         /*motorFrontRight.setPower(-joystickY - joystickX - rotation);
             motorBackRight.setPower(-joystickY + joystickX - rotation);
             motorFrontLeft.setPower(-joystickY + joystickX + rotation);
             motorBackLeft.setPower(-joystickY - joystickX + rotation);*/
    }

    /**
     * Controls the robot using tank drive, this is only used in our testing code just to see what tank drive would feel like.
     * we can move in all directions except diagonals (probably).
     *
     * <p>Issues: none/haven't used it enough to identify issues</p>
     *
     * @param joystickL values of the left joystick, used for control of left wheels
     * @param joystickR values of right joystick, used for control of right wheels
     * @param joystickX x value of joystick, used to strafe left and right
     */
    public void driveTank(double joystickL, double joystickR, double joystickX)
    {
        /**                   | Forward,    |           |
         *                    | Backwards,  |  Strafing |
                              | and Turning |           | */
        motorFrontRight.setPower(-joystickR - joystickX);
        motorBackRight.setPower(-joystickR + joystickX);
        motorFrontLeft.setPower(-joystickL + joystickX);
        motorBackLeft.setPower(-joystickL - joystickX);

         /*motorFrontRight.setPower(joystickY + joystickX - rotation);
         motorBackRight.setPower(joystickY - joystickX - rotation);
         motorFrontLeft.setPower(-joystickY + joystickX - rotation);
         motorBackLeft.setPower(-joystickY - joystickX - rotation);*/
    }

    public void driveFieldRelative(double joystickX, double joystickY, double turn, boolean slow, double slowFactor, double baseFactor)
    {
        Orientation currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        // convert to polar
        double theta = Math.atan2(joystickX, joystickY);
        double r = Math.hypot(joystickX, joystickY);
        //rotate angle
        theta = AngleUnit.normalizeRadians(theta - currentHeading.firstAngle);

        //convert back to cartesion
        double newJoystickX = r * Math.sin(theta);
        double newJoystickY = r * Math.cos(theta);

        driveOmniDir(newJoystickX,newJoystickY, turn, slow, slowFactor, baseFactor);


        /*rotate = turn;
        forward = joystickY * -1;
        strafe = joystickX;

        // Adjust Joystick X/Y inputs by yaw angle

        gyro_degrees = getIntegratedHeading();
        gyro_radians =(gyro_degrees * Math.PI/180);
        temp = forward * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
        strafe = -forward * Math.sin(gyro_radians) + strafe * Math.cos(gyro_radians);
        forward = temp;

        motorFrontRight.setPower(forward - strafe - rotate);
        motorBackRight.setPower(forward + strafe - rotate);
        motorFrontLeft.setPower(forward + strafe + rotate);
        motorBackLeft.setPower(forward - strafe + rotate); */
    }


    /**
     * Gives back the robots integrated heading, mainly used in autonomous
     *
     * <p>Issues: Need to look into some on how this works compared to just getting the angle normally</p>
     *
     * @return integrated heading in degrees [-180,180]
     */
    public double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;

    }

    /**
     * Turns the the robot to the target degrees at set power then corrects with correction power. This uses the gyro sensor which we don't use anymore.
     *
     * @param targetDegrees target degrees we want to turn
     * @param power power we want to turn towards target with [-1,1]
     * @param correctionPower power we want to correct the turn with normally lower than normal power for more precision [-1,1]
     */
    @Deprecated void turn(int targetDegrees, double power, double correctionPower)
    {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //Right is all positive
        //Left is all negative
        //Straight is left positive. Right negative.
        if(targetDegrees == 0)
        {
            //Tell robot to correct to straight forward

            if(MRgyro.getIntegratedZValue() > 0)
            {
                //If the MRgyro reads back left from zero

                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);

                while(MRgyro.getIntegratedZValue() > targetDegrees)
                {

                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                    telemetry.update();

                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }

                motorFrontLeft.setPower(correctionPower);
                motorBackLeft.setPower(correctionPower);
                motorFrontRight.setPower(-correctionPower);
                motorBackRight.setPower(-correctionPower);

                while(MRgyro.getIntegratedZValue() < targetDegrees)
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                    telemetry.update();
                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
            }
            if(MRgyro.getIntegratedZValue() < 0)
            {
                //If the MRgyro reads back right from zero

                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);

                while(MRgyro.getIntegratedZValue() < targetDegrees)
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                    telemetry.update();

                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
                motorFrontLeft.setPower(-correctionPower);
                motorBackLeft.setPower(-correctionPower);
                motorFrontRight.setPower(correctionPower);
                motorBackRight.setPower(correctionPower);

                while(MRgyro.getIntegratedZValue() > targetDegrees)
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                    telemetry.update();
                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
            }
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
        }
////////////////////////////////////////////////////////////////////////////////////////////////////
        if(targetDegrees > 0)
        {
            //TURNING LEFT
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);

            while(MRgyro.getIntegratedZValue() < targetDegrees)
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                telemetry.update();

                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }

            motorFrontLeft.setPower(correctionPower);
            motorBackLeft.setPower(correctionPower);
            motorFrontRight.setPower(-correctionPower);
            motorBackRight.setPower(-correctionPower);

            while(MRgyro.getIntegratedZValue() > targetDegrees)
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                telemetry.update();
                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
        }
        else
        {

            //TURNING RIGHT
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);

            while(MRgyro.getIntegratedZValue() > targetDegrees)
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                telemetry.update();

                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
            motorFrontLeft.setPower(-correctionPower);
            motorBackLeft.setPower(-correctionPower);
            motorFrontRight.setPower(correctionPower);
            motorBackRight.setPower(correctionPower);

            while(MRgyro.getIntegratedZValue() < targetDegrees)
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                telemetry.update();
                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    /**
     * Turns the the robot to the target degrees at set power then corrects with correction power. This uses the imu instead of gyro sensor.
     *
     * <p>Issues: Used to sometimes just not turn, I think this is fixed it hasn't happened recently.</p>
     *
     * @param targetDegrees target degrees we want to turn
     * @param power power we want to turn towards target with [-1,1]
     * @param correctionPower power we want to correct the turn with normally lower than normal power for more precision [-1,1]
     */
    public void turnIMU(int targetDegrees, double power, double correctionPower)
    {

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right is up to +180
        //Left is up to -180

        //Right is all positive
        //Left is all negative
        //Straight is left positive. Right negative.
        if(targetDegrees == 0 && !isStopRequested())
        {
            //Tell robot to correct to straight forward

            if(getIntegratedHeading() > 0 && !isStopRequested())
            {
                //If the MRgyro reads back left from zero

                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);

                while(getIntegratedHeading() > targetDegrees && !isStopRequested())
                {

                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", getIntegratedHeading());
                    telemetry.update();

                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }

                motorFrontLeft.setPower(correctionPower);
                motorBackLeft.setPower(correctionPower);
                motorFrontRight.setPower(-correctionPower);
                motorBackRight.setPower(-correctionPower);

                while(getIntegratedHeading() < targetDegrees && !isStopRequested())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", getIntegratedHeading());
                    telemetry.update();
                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
            }
            if(getIntegratedHeading() < 0 && !isStopRequested())
            {
                //If the MRgyro reads back right from zero

                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);

                while(getIntegratedHeading() < targetDegrees && !isStopRequested())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", getIntegratedHeading());
                    telemetry.update();

                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
                motorFrontLeft.setPower(-correctionPower);
                motorBackLeft.setPower(-correctionPower);
                motorFrontRight.setPower(correctionPower);
                motorBackRight.setPower(correctionPower);

                while(getIntegratedHeading() > targetDegrees && !isStopRequested())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", getIntegratedHeading());
                    telemetry.update();
                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
            }
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
        }
////////////////////////////////////////////////////////////////////////////////////////////////////
        if(targetDegrees > 0 && !isStopRequested())
        {
            //TURNING LEFT
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);

            while(getIntegratedHeading() < targetDegrees && !isStopRequested())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", getIntegratedHeading());
                telemetry.update();

                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }

            motorFrontLeft.setPower(correctionPower);
            motorBackLeft.setPower(correctionPower);
            motorFrontRight.setPower(-correctionPower);
            motorBackRight.setPower(-correctionPower);

            while(getIntegratedHeading() > targetDegrees && !isStopRequested())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", getIntegratedHeading());
                telemetry.update();
                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
        }
        else
        {

            //TURNING RIGHT
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);

            while(getIntegratedHeading() > targetDegrees && !isStopRequested())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", getIntegratedHeading());
                telemetry.update();

                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
            motorFrontLeft.setPower(-correctionPower);
            motorBackLeft.setPower(-correctionPower);
            motorFrontRight.setPower(correctionPower);
            motorBackRight.setPower(correctionPower);

            while(getIntegratedHeading() < targetDegrees && !isStopRequested())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", getIntegratedHeading());
                telemetry.update();
                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);


    }

    /**
     * Sets all drive motors to a certain power. Used to save a few lines in autonomous mainly
     *
     * @param motorPower power to set all motors to [-1,1]
     */
    public void setDrivePower(float motorPower)
    {
        motorFrontLeft.setPower(motorPower);
        motorBackLeft.setPower(motorPower);
        motorFrontRight.setPower(motorPower);
        motorBackRight.setPower(motorPower);
    }

    /**
     * Sets all drive motors to a certain mode. Used to save a few lines in autonomous mainly
     *
     * @param motorMode mode to set all motors to
     */
    public void setDriveMode(DcMotor.RunMode motorMode)
    {
        motorFrontLeft.setMode(motorMode);
        motorBackLeft.setMode(motorMode);
        motorFrontRight.setMode(motorMode);
        motorBackRight.setMode(motorMode);
    }

    /**
     * Sets all indiviual drive motors to a certain power. Used to save a few lines in autonomous mainly
     *
     * @param frontLeft power to set frontLeft motor to [-1,1]
     * @param backLeft power to set backLeft motor to [-1,1]
     * @param frontRight power to set frontRight motor to [-1,1]
     * @param backRight power to set frontRight motor to [-1,1]
     */
    public void setIndividualDrivePower(double frontLeft, double backLeft, double frontRight, double backRight)
    {
        motorFrontLeft.setPower(frontLeft);
        motorBackLeft.setPower(backLeft);
        motorFrontRight.setPower(frontRight);
        motorBackRight.setPower(backRight);
    }

    //Unfinished
    public void  driveDecay(boolean forward, int distanceEncodeVal, double power)
    {

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveTime = distanceEncodeVal;


        if(forward)
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(forward)
        {
            while (motorFrontRight.getCurrentPosition() < distanceEncodeVal - 20 && !isStopRequested())
            {

            }
        }
        else
        {

            while (motorFrontRight.getCurrentPosition() > -distanceEncodeVal + 20 && !isStopRequested())
            {

            }

        }

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }
    private double calcDecayPower(int currentPos, int targetPos)
    {
        double decay = -1/(Math.pow(targetPos,2));
        return (decay * Math.pow(targetPos,2) + 1);
    }


    public int calculateTicks(double inches)
    {
        int encoderTicks = (int) Math.round(COUNTS_PER_INCH * inches);
        return encoderTicks;
    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    /**
     * Drives us forward/backward a set distance at a set heading and base speed using a PID to control our heading.
     *
     * <p>Comments: Haven't tested driving backwards and I am wondering if a positional/distance PID controller can be
     *              incorporated to get more optimal motor speed. IE: starting out fast and slowing down as it reaches its target</p>
     *
     * @param distance distance in inches to travel
     * @param heading heading to drive at
     * @param baseDriveSpeed base power to set the motors to that will then be corrected via PID
     *
     *
     */
    public void driveHeading(double distance, double heading, double baseDriveSpeed)
    {
        PIDController pidController = new PIDController(.03,0,0,.25);

        //Find new target pos and pass to motor controller
        int distanceEncoderTicks = calculateTicks(distance);
        telemetry.addData("distanceEncoderTicks: ", distanceEncoderTicks);
        telemetry.update();

        double leftPower;
        double rightPower;

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(motorFrontLeft.getCurrentPosition() < distanceEncoderTicks - 20 && !isStopRequested())
        {
            double correction = pidController.output(heading, getRawHeading());

            leftPower = Range.clip(baseDriveSpeed - correction, -1, 1);
            rightPower = Range.clip(baseDriveSpeed + correction,-1,1);

            setIndividualDrivePower(leftPower, leftPower, rightPower, rightPower);

            telemetry.addData("leftPower: ", leftPower);
            telemetry.addData("rightPower: ", rightPower);
            telemetry.addData("correction: ", correction);
            telemetry.update();
        }
        setDrivePower(0);

    }

    //WIP
    public void strafeHeading(double distance, double heading, double baseDriveSpeed)
    {
        PIDController pidController = new PIDController(.03,0,0,.25);

        //Find new target pos and pass to motor controller
        int distanceEncoderTicks = calculateTicks(distance);

        // leftDiagonal: frontLeft and backRight
        // rightDiagonal: backLeft and frontRight

        double leftDiagonalPower = baseDriveSpeed;
        double rightDiagonalPower = -baseDriveSpeed;

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(motorFrontLeft.getCurrentPosition() < distanceEncoderTicks - 20 && !isStopRequested())
        {
            double correction = pidController.output(heading, getRawHeading());

            leftDiagonalPower = Range.clip(baseDriveSpeed - correction, -1, 1);
            rightDiagonalPower = Range.clip(baseDriveSpeed + correction,-1,1);

            setIndividualDrivePower(leftDiagonalPower, rightDiagonalPower, leftDiagonalPower, rightDiagonalPower);

            telemetry.addData("leftDiagonal: ", leftDiagonalPower);
            telemetry.addData("rightDiagonal: ", rightDiagonalPower);
            telemetry.addData("correction: ", correction);
            telemetry.update();
        }
        setDrivePower(0);
    }
}


