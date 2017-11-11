//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Disabled
public class DriveFunctions extends LinearOpMode
{

    //Define Drive Motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Glyph Motors
    DcMotor glyphGrab;
    DcMotor glyphLift;

//    //Relic Motors
//    Servo relicGrab;
//    DcMotor relicLift;

    //Jewel Motor
    Servo jewelArm;

    //Define Sensors and the CDI
    ColorSensor colorSensor;


    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    /**
     * Initialize all the hardware
     * This creates a data type DriveFunctions to store all the hardware devices
     */
    public DriveFunctions(DcMotor leftMotorFront, DcMotor rightMotorFront, DcMotor leftMotorBack, DcMotor rightMotorBack, DcMotor glyphGrab, DcMotor glyphLift, Servo jewelArm, ColorSensor colorSensor)
    {
        //These lines enable us to store the motors, sensors and CDI without having to write them over and over again
        //Initialize DC and Servo motors
        this.leftMotorFront = leftMotorFront;
        this.leftMotorBack = leftMotorBack;
        this.rightMotorFront = rightMotorFront;
        this.rightMotorBack = rightMotorBack;
        this.glyphGrab = glyphGrab;
        this.glyphLift = glyphLift;
//        this.relicGrab = relicGrab;
//        this.relicLift = relicLift;
        this.jewelArm = jewelArm;

        //Initialize sensors and CDI
        this.colorSensor = colorSensor;
    }

    /**
     * Set sensor addresses, modes and DC motor directions
     */
    public void initializeMotorsAndSensors()
    {
        //Set the sensor to the mode that we want, and set their addresses
        colorSensor.enableLed(true);

        //Reverse some motors and keep others forward
        leftMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
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
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn right
     */
    public void rightTurnTeleop(float power)
    {
        //Turn the right motors backwards and the left motors forward so that it turns right
        setDriveMotorPowers(-power, -power, power, power);
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

    public static void oneMotorEncoder(int degrees, float power, DcMotor motor)
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

    /**
     * Drive for the given distance at the given power
     * @param degrees distance
     */
    public void driveAutonomous(float power, int degrees)
    {
        //Everything in the same direction creates linear driving
        moveDriveMotorsWithEncoders(-degrees, -degrees, -degrees, -degrees, -power, -power, -power, -power);
    }

    /**
     * Turn left for the given distance at the given power
     * @param degrees distance
     */
    public void leftTurnAutonomous(float power, int degrees)
    {
        //Left motors backwards and right motors forwards gives us a left turn
        moveDriveMotorsWithEncoders(-degrees, -degrees, degrees, degrees, -power, -power, power, power);
    }

    /**
     * Turn right for the given distance at the given power
     * @param degrees distance
     */
    public void rightTurnAutonomous(float power, int degrees)
    {
        //Right motors backwards and left motors forwards gives us a right turn
        moveDriveMotorsWithEncoders(degrees, degrees, -degrees, -degrees, power, power, -power, -power);
    }

    /**
     * Shift left for the given distance at the given power
     * @param degrees distance
     */
    public void leftShiftAutonomous(float power, int degrees)
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift left
        moveDriveMotorsWithEncoders(-degrees, degrees, degrees, -degrees, -power, power, power, -power);
    }

    /**
     * Shift right for the given distance at the given power
     * @param degrees distance
     */
    public void rightShiftAutonomous(float power, int degrees)
    {
        //This sequence of forwards, backwards, backwards, forwards makes the robot shift right
        moveDriveMotorsWithEncoders(degrees, -degrees, -degrees, degrees, power, -power, -power, power);
    }

    public void glyphDoor(String openOrClose) throws InterruptedException
    {
        if (openOrClose == "open")
        {
            glyphGrab.setPower(-0.5);
            Thread.sleep(700);
            glyphGrab.setPower(0.0);
            oneMotorEncoder(-200, (float) -1.0, glyphLift);
        }
        if (openOrClose == "close")
        {
            glyphGrab.setPower(0.5);
            Thread.sleep(700);
            glyphGrab.setPower(0.2);
            oneMotorEncoder(200, (float) 1.0, glyphLift);
        }
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

//    public String vuforia(){
//        /*
//         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
//         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
//         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        //License key from vuforia website
//        parameters.vuforiaLicenseKey = "Adp/KFX/////AAAAGYMHgTasR0y/o1XMGBLR4bwahfNzuw2DQMMYq7vh4UvYHleflzPtt5rN2kFp7NCyO6Ikkqhj/20qTYc9ex+340/hvC49r4mphdmd6lI/Ip64CbMTB8Vo53jBHlGMkGr0xq/+C0SKL1hRXj5EkXtSe6q9F9T/nAIcg9Jr+OfAcifXPH9UJYG8WmbLlvpqN+QuVA5KQ6ve1USpxYhcimV9xWCBrq5hFk1hGLbeveHrKDG3wYRdwBeYv3Yo5qYTsotfB4CgJT9CX/fDR/0JUL7tE29d1v1eEF/VXCgQP4EPUoDNBtNE6jpKJhtQ8HJ2KjmJnW55f9OqNc6SsULV3bkQ52PY+lPLt1y4muyMrixCT7Lu";
//
//        //Which camera?
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();
//        waitForStart();
//
//        relicTrackables.activate();
//
//
//
//        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//            telemetry.addData("VuMark", "%s visible", vuMark);
//            return vuMark + "";
//        }
//        else {
//            telemetry.addData("VuMark", "not visible");
//            return "";
//        }
//        telemetry.update();
//
//    }



    public void jewelPush(ColorSensor colorSensor, String color, String colorSeen) throws InterruptedException
    {
        float turnPower = (float) 0.3;
        int turnDistance = 300;

        jewelArm.setPosition(1.0);
        Thread.sleep(1000);
        if (iSeeAColor(colorSensor))
        {
            colorSeen = whatColor(colorSensor);
        }
        if (colorSeen == color)
        {
            rightTurnAutonomous(turnPower, turnDistance);
            jewelArm.setPosition(0.5);
            Thread.sleep(1000);
            leftTurnAutonomous(turnPower, turnDistance);
        }
        else
        {
            leftTurnAutonomous(turnPower, turnDistance);
            jewelArm.setPosition(0.5);
            Thread.sleep(1000);
            rightTurnAutonomous(turnPower, turnDistance);
        }

    }

    //Empty main
    @Override
    public void runOpMode() throws InterruptedException
    {

    }
} //Close class and end program

