//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class DriveFunctions extends LinearOpMode
{
    //Define Drive Motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define Glyph Motors
    DcMotor glyphWheelLeft;
    DcMotor glyphWheelRight;
    DcMotor glyphLift;
    Servo glyphFlip;

    //Relic Motors
    Servo relicGrab;
    CRServo relicFlip;
    DcMotor relicSpool;

    //Jewel Motor
    Servo jewelArm;

    //Define Sensors and the CDI
    ColorSensor colorSensor;

    //Initialize vuforia
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    /**
     * Initialize all the hardware
     * This creates a data type DriveFunctions to store all the hardware devices
     */
    public DriveFunctions(DcMotor leftMotorFront, DcMotor rightMotorFront, DcMotor leftMotorBack, DcMotor rightMotorBack, DcMotor glyphWheelLeft, DcMotor glyphWheelRight, DcMotor glyphLift, Servo glyphFlip, Servo relicGrab, CRServo relicFlip, DcMotor relicSpool, Servo jewelArm, ColorSensor colorSensor)
    {
        //These lines enable us to store the motors, sensors and CDI without having to write them over and over again
        //Initialize DC and Servo motors
        this.leftMotorFront = leftMotorFront;
        this.leftMotorBack = leftMotorBack;
        this.rightMotorFront = rightMotorFront;
        this.rightMotorBack = rightMotorBack;
        this.glyphWheelLeft = glyphWheelLeft;
        this.glyphWheelRight = glyphWheelRight;
        this.glyphLift = glyphLift;
        this.glyphFlip = glyphFlip;
        this.relicGrab = relicGrab;
        this.relicSpool = relicSpool;
        this.relicFlip = relicFlip;
        this.jewelArm = jewelArm;

        //Initialize sensors
        this.colorSensor = colorSensor;
    }

    /**
     * Set sensor addresses, modes and DC motor directions
     */
    public void initializeMotorsAndSensors()
    {
        //Set the sensor to the mode that we want
        colorSensor.enableLed(true);

        //Reverse some motors and keep others forward
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set the motors to brake mode to prevent rolling due to chain
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        glyphLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Takes in motor powers for 4 drive motors
     */
    public void setDriveMotorPowers(float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        leftMotorFront.setPower(leftFrontPower);
        leftMotorBack.setPower(leftBackPower);
        rightMotorFront.setPower(rightFrontPower);
        rightMotorBack.setPower(rightBackPower);
    }

    /**
     * If this function is called, stop the drive motors
     */
    public void stopDriving()
    {
        //Set all drive motor powers as zero
        setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
    }


    /**
     * If this function is called, turn on the drive motors at the given powers to make it drive forward or backwards
     */
    public void driveTeleop(float drive)
    {
        //Send all the motors in the same direction
        setDriveMotorPowers(drive, drive, drive, drive);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn left
     */
    public void leftTurnTeleop(float power)
    {
        //Turn the left motors backwards and the right motors forward so that it turns left
        setDriveMotorPowers(power, power, -power, -power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn right
     */
    public void rightTurnTeleop(float power)
    {
        //Turn the right motors backwards and the left motors forward so that it turns right
        setDriveMotorPowers(-power, -power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the
     * given powers, to make it shift in the desired direction
     */
    public void shiftTeleop(float shift)
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift
        setDriveMotorPowers(-shift, shift, shift, -shift);
    }

    public void resetEncoders()
    {
        //Reset the encoders
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Use the encoders
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * Takes in powers for 4 drive motors, as well as 4 encoder distances
     * Allows us to run at the entered power, for the entered distance
     */
    public void moveDriveMotorsWithEncoders(int leftFrontDegrees, int leftBackDegrees, int rightFrontDegrees, int rightBackDegrees, float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Reset the encoders
        resetEncoders();

        //Set up the motors run to the given position
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the target position as the corresponding values entered
        leftMotorFront.setTargetPosition(leftFrontDegrees);
        leftMotorBack.setTargetPosition(leftBackDegrees);
        rightMotorFront.setTargetPosition(rightFrontDegrees);
        rightMotorBack.setTargetPosition(rightBackDegrees);

        //Turn on the motors at the corresponding powers
        setDriveMotorPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

        //Empty while loop while the motors are moving
        while ((leftMotorFront.isBusy()) && (rightMotorFront.isBusy()) && (leftMotorBack.isBusy()) && (rightMotorBack.isBusy()))
        { }

        //Stop driving
        stopDriving();

        //Use the encoders in the future
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void oneMotorEncoder(DcMotor motor, float power, int degrees)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(degrees);
        motor.setPower(power);
        while (motor.isBusy())
        { }
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void crServoTime(CRServo motor, float power, int time)
    {
        ElapsedTime count = new ElapsedTime();
        count.reset();
        motor.setPower(power);
        if (count.time() > time)
        {
            motor.setPower(0.0);
        }
    }

    public void servoTime(Servo motor, double position1, double position2, int time)
    {
        ElapsedTime counter = new ElapsedTime();
        counter.reset();
        if (counter.time() <= time)
        {
            motor.setPosition(position1);
        }
        motor.setPosition(position2);
    }

    /**
     * Drive for the given distance at the given power
     * @param degrees distance
     */
    public void driveAutonomous(float power, int degrees) throws InterruptedException {
        //Everything in the same direction creates linear driving
        moveDriveMotorsWithEncoders(-degrees, -degrees, -degrees, -degrees, -power, -power, -power, -power);

        //Braking mechanism part 1
        moveDriveMotorsWithEncoders(50, 50, 50, 50, (float) 1.0, (float) 1.0, (float) 1.0, (float) 1.0);

        //Small delay
        Thread.sleep(100);

        //Braking mechanism part 2
        setDriveMotorPowers(0,0,0,0);

        //Small delay
        Thread.sleep(100);
    }

    /**
     * Turn left for the given distance at the given power
     * @param degrees distance
     */
    public void leftTurnAutonomous(float power, int degrees) throws InterruptedException
    {
        //Left motors backwards and right motors forwards gives us a left turn
        moveDriveMotorsWithEncoders(degrees, degrees, -degrees, -degrees, power, power, -power, -power);

        //Braking mechanism part 1
        moveDriveMotorsWithEncoders(-50, -50, 50, 50, (float) -1.0, (float) -1.0, (float) 1.0, (float) 1.0);

        //Small delay
        Thread.sleep(100);

        //Braking mechanism part 2
        setDriveMotorPowers(0,0,0,0);

        //Small delay
        Thread.sleep(100);
    }

    /**
     * Turn right for the given distance at the given power
     * @param degrees distance
     */
    public void rightTurnAutonomous(float power, int degrees) throws InterruptedException
    {
        //Right motors backwards and left motors forwards gives us a right turn
        moveDriveMotorsWithEncoders(-degrees, -degrees, degrees, degrees, -power, -power, power, power);

        //Braking mechanism part 1
        moveDriveMotorsWithEncoders(50, 50, -50, -50, (float) 1.0, (float) 1.0, (float) -1.0, (float) -1.0);

        //Small delay
        Thread.sleep(100);

        //Braking mechanism part 2
        setDriveMotorPowers(0,0,0,0);

        //Small delay
        Thread.sleep(100);
    }

    /**
     * Shift left for the given distance at the given power
     * @param degrees distance
     */
    public void leftShiftAutonomous(float power, int degrees) throws InterruptedException
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift left
        moveDriveMotorsWithEncoders(degrees, -degrees, -degrees, degrees, power, -power, -power, power);

        //Braking mechanism part 1
        moveDriveMotorsWithEncoders(-50, 50, 50, -50, (float) -1.0, (float) 1.0, (float) 1.0, (float) -1.0);

        //Small delay
        Thread.sleep(100);

        //Braking mechanism part 2
        setDriveMotorPowers(0,0,0,0);

        //Small delay
        Thread.sleep(100);
    }

    /**
     * Shift right for the given distance at the given power
     * @param degrees distance
     */
    public void rightShiftAutonomous(float power, int degrees) throws InterruptedException
    {
        //This sequence of forwards, backwards, backwards, forwards makes the robot shift right
        moveDriveMotorsWithEncoders(-degrees, degrees, degrees, -degrees, -power, power, power, -power);

        //Braking mechanism part 1
        moveDriveMotorsWithEncoders(50, -50, -50, 50, (float) 1.0, (float) -1.0, (float) -1.0, (float) 1.0);

        //Small delay
        Thread.sleep(100);

        //Braking mechanism part 2
        setDriveMotorPowers(0,0,0,0);

        //Small delay
        Thread.sleep(100);
    }

    public void sweepTurnLeft(float power) throws InterruptedException
    {
        setDriveMotorPowers(-power, -power, 0, 0);
        Thread.sleep(600);
        setDriveMotorPowers(0,0,0,0);
    }

    public void sweepTurnRight(float power) throws InterruptedException
    {
        setDriveMotorPowers(0,0, -power, -power);
        Thread.sleep(400);
        setDriveMotorPowers(0,0,0,0);
    }


    /**
     * @param colorSensor take in the correct color sensor
     * @return returns true if the supplied ColorSensor either red or blue.  False otherwise
     */
    public boolean iSeeAColor(ColorSensor colorSensor)
    {
        //This is an array that stores the hue[0], saturation[1], and value[2], values
        float[] hsvValues = {0F, 0F, 0F};

        //Convert from RGB to HSV (red-green-blue to hue-saturation-value)
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        //If no value, return false
        if (hsvValues[2] == 0)
        {
            return false;
        }

        //Otherwise return true
        return true;
    }

    /**
     * Determines what color the color sensor is seeing
     * @param colorSensor take in the correct color sensor
     * @return The string "Blue" if we see the color blue, "Red" if we see the color red
     */
    public String whatColor(ColorSensor colorSensor)
    {
        //Define float for hue
        float hue;

        //This is an array that stores the hue[0], saturation[1], and value[2], values
        float[] hsvValues = {0F, 0F, 0F};

        //Convert from RGB to HSV (red-green-blue to hue-saturation-value)
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        //store the first value from the array into hue
        hue = hsvValues[0];

        //If hue is greater than 120, we are looking at blue, so return blue
        if (hue > 120)
        {
            return "Blue";
        }

        //Otherwise return red
        return "Red";
    }

    public void jewelPush(ColorSensor colorSensor, String color, String colorSeen) throws InterruptedException
    {
        //Define constants to avoid magic numbers
        float power = (float) 0.7;
        int shortDistance = 120;
        int longDistance = 170;

        //Drop the arm
        jewelArm.setPosition(0.7);

        //Wait for 1 second
        Thread.sleep(1000);

        //Delay until a color is seen
        while (!iSeeAColor(colorSensor))
        { }

        //If a color is seen, set it to the variable colorSeen
        if (iSeeAColor(colorSensor))
        {
            colorSeen = whatColor(colorSensor);
        }

        //If the color seen is our team color
        if (colorSeen.equals(color))
        {
            //Turn to the right to hit the opposite ball
            rightTurnAutonomous(power, longDistance);

            //Lift the arm
            jewelArm.setPosition(0.0);

            //Turn back to the original position so we can continue
            leftTurnAutonomous(power, longDistance);
        }

        //If the color seen is not our team color
        if (!colorSeen.equals(color))
        {
            //Turn to the left to hit the ball we see
            leftTurnAutonomous(power, shortDistance);

            //Lift the arm
            jewelArm.setPosition(0.0);

            //Turn back to the original position so we can continue
            rightTurnAutonomous(power, shortDistance);
        }
    }

    //Empty main
    @Override
    public void runOpMode() throws InterruptedException
    {

    }
} //Close class and end program

