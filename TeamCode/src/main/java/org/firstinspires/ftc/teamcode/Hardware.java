package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

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
    public DcMotor motorCarousel; //In use
    public DcMotor motorArm;     //In use
    public DcMotor motorWinch;   //In use
    public DcMotor motorLift;

    Servo servoIntake; //In use
    Servo servoWrist; //In use

    //CRServo servoWrist;
    CRServo servoIntakeClose;
    CRServo servoIntakeFar;

    //ColorSensor colorSensor;
    //DistanceSensor distanceSensor;
    //Rev2mDistanceSensor distanceSensor;

    ModernRoboticsI2cGyro MRgyro;
    ModernRoboticsI2cRangeSensor MRRange;

    BNO055IMU imu;
    Orientation angles;
    private double previousHeading = 0;
    private double integratedHeading = 0;

    DcMotor.RunMode initialMode = null;

    int driveTime;

    //software elements mainly for field relative
    double rotate;
    double forward;
    double strafe;
    double gyro_degrees;
    double gyro_radians;
    double temp;

    /**
     * The variables and Equation used to turn the current wheels' encoders into inches
     */
    final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: Gobuilda 13.7:1 Ratio, 435 RPM
    final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP (2 rotations at input = 1 at output)
    final double     WHEEL_DIAMETER_INCHES   = 96 / 25.4;     // For figuring circumference (96mm / 25.4 = ~3.78)
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    //                                                             384.5                       /     11.8378

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

        servoIntakeClose = aMap.crservo.get("servoIntakeClose");
        servoIntakeFar = aMap.crservo.get("servoIntakeFar");

        motorLift = aMap.dcMotor.get("motorLift");
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
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
    public void drivePID(double angle, int distanceInches, double power)
    {
        int distanceEncodeVal;
        final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: Gobuilda 13.7:1 Ratio, 435 RPM
        final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 96 / 25.4;     // For figuring circumference (3.69) (96mm / 25.4 = ~3.69)
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
     * <p>Issues: Haven't used this enough to determine the issues it might work though
     * I would like to calculate the drive time instead of doing it manually</p>
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
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        if(!slow){
            motorFrontRight.setPower((-joystickY - joystickX - rotation) / baseFactor);
            motorBackRight.setPower((-joystickY + joystickX - rotation) / baseFactor);
            motorFrontLeft.setPower((-joystickY + joystickX + rotation) / baseFactor);
            motorBackLeft.setPower((-joystickY - joystickX + rotation) / baseFactor);
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

    public void driveFieldRelative(double joystickX, double joystickY, double turn)
    {
        rotate = turn;
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
        motorBackLeft.setPower(forward - strafe + rotate);
    }


    /**
     * Gives back the robots integrated heading, mainly used in autonomous
     *
     * <p>Issues: had issues in the past, but seems to be working currently</p>
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

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void driveStraight(double power, double time)
    {
        double desiredHeading = getIntegratedHeading();
        double currentHeading = getIntegratedHeading();
        double error = 0.0;
        long beginning = System.currentTimeMillis();
        long end = beginning + driveTime;

        setDrivePower((float) power);
        while(System.currentTimeMillis() != end)
        {
            currentHeading = getIntegratedHeading();
            error = ((desiredHeading - currentHeading) / 180);
            telemetry.addData("Error: ", error);
            telemetry.addData("Current Heading: ", getIntegratedHeading());
            telemetry.update();
            /*if (differential > 0)
            {
                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(differential);
                motorBackRight.setPower(differential);
            }*/

        }




    }
    public int calculateTicks(double inches)
    {
        int encoderTicks = (int) Math.round(COUNTS_PER_INCH * inches);
        return encoderTicks;
    }
}
