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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

    //Glyph Motors
    DcMotor glyphGrab;
    DcMotor glyphLift;

    //Relic Motors
    Servo relicGrab;
    DcMotor relicSpool;
    Servo relicFlip;

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
    public DriveFunctions(DcMotor leftMotorFront, DcMotor rightMotorFront, DcMotor leftMotorBack, DcMotor rightMotorBack, DcMotor glyphGrab, DcMotor glyphLift, Servo relicGrab, Servo relicFlip, DcMotor relicSpool, Servo jewelArm, ColorSensor colorSensor)
    {
        //These lines enable us to store the motors, sensors and CDI without having to write them over and over again
        //Initialize DC and Servo motors
        this.leftMotorFront = leftMotorFront;
        this.leftMotorBack = leftMotorBack;
        this.rightMotorFront = rightMotorFront;
        this.rightMotorBack = rightMotorBack;
        this.glyphGrab = glyphGrab;
        this.glyphLift = glyphLift;
        this.relicGrab = relicGrab;
        this.relicSpool = relicSpool;
        this.relicFlip = relicFlip;
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
        moveDriveMotorsWithEncoders(50, 50, 50, 50, (float) 1.0, (float) 1.0, (float) 1.0, (float) 1.0);
    }

    /**
     * Turn left for the given distance at the given power
     * @param degrees distance
     */
    public void leftTurnAutonomous(float power, int degrees) throws InterruptedException
    {
        //Left motors backwards and right motors forwards gives us a left turn
        moveDriveMotorsWithEncoders(degrees, degrees, -degrees, -degrees, power, power, -power, -power);
        moveDriveMotorsWithEncoders(-50, -50, 50, 50, (float) -1.0, (float) -1.0, (float) 1.0, (float) 1.0);
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
        moveDriveMotorsWithEncoders(50, 50, -50, -50, (float) 1.0, (float) 1.0, (float) -1.0, (float) -1.0);
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
        moveDriveMotorsWithEncoders(-50, 50, 50, -50, (float) -1.0, (float) 1.0, (float) 1.0, (float) -1.0);
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
        moveDriveMotorsWithEncoders(50, -50, -50, 50, (float) 1.0, (float) -1.0, (float) -1.0, (float) 1.0);
        Thread.sleep(100);
    }

    public void glyphDoor(String openOrClose) throws InterruptedException
    {
        if (openOrClose == "open")
        {
            glyphGrab.setPower(-0.5);
            Thread.sleep(700);
            glyphGrab.setPower(0.0);
            oneMotorEncoder(-800, (float) -1.0, glyphLift);
        }
        if (openOrClose == "close")
        {
            glyphGrab.setPower(0.5);
            Thread.sleep(700);
            glyphGrab.setPower(0.2);
            oneMotorEncoder(800, (float) 1.0, glyphLift);
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

    public void jewelPush(ColorSensor colorSensor, String color, String colorSeen) throws InterruptedException
    {
        float power = (float) 0.3;
        int shortDistance = 70;
        int longDistance = 140;

        jewelArm.setPosition(1.0);
        Thread.sleep(1000);
        while (!iSeeAColor(colorSensor))
        { }

        if (iSeeAColor(colorSensor))
        {
            colorSeen = whatColor(colorSensor);
        }
        if (colorSeen.equals(color))
        {
            rightTurnAutonomous(power, longDistance);
            jewelArm.setPosition(0.1);
            Thread.sleep(1000);
            leftTurnAutonomous(power, longDistance);
        }
        if (!colorSeen.equals(color))
        {
            leftTurnAutonomous(power, shortDistance);
            jewelArm.setPosition(0.1);
            Thread.sleep(1000);
            rightTurnAutonomous(power, shortDistance);
        }

        driveAutonomous(power, 1400);
    }

//    public String vuforiaRead()
//    {
//
//        final String TAG = "Vuforia Navigation Sample";
//
//        OpenGLMatrix lastLocation = null;
//
//        /**
//         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//         * localization engine.
//         */
//        VuforiaLocalizer vuforia;
//
//            /*
//             * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
//             * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
//             */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        // OR...  Do Not Activate the Camera Monitor View, to save power
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//            /*
//             * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//             * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//             * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//             * web site at https://developer.vuforia.com/license-manager.
//             *
//             * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//             * random data. As an example, here is a example of a fragment of a valid key:
//             *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//             * Once you've obtained a license key, copy the string from the Vuforia web site
//             * and paste it in to your code onthe next line, between the double quotes.
//             */
//        parameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";
//
//            /*
//             * We also indicate which camera on the RC that we wish to use.
//             * Here we chose the back (HiRes) camera (for greater range), but
//             * for a competition robot, the front camera might be more convenient.
//             */
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        /**
//         * Load the data sets that for the trackable objects we wish to track. These particular data
//         * sets are stored in the 'assets' part of our application (you'll see them in the Android
//         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
//         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
//         * example "StonesAndChips", datasets can be found in in this project in the
//         * documentation directory.
//         */
//        VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("StonesAndChips");
//        VuforiaTrackable redTarget = stonesAndChips.get(0);
//        redTarget.setName("RedTarget");  // Stones
//
//        VuforiaTrackable blueTarget = stonesAndChips.get(1);
//        blueTarget.setName("BlueTarget");  // Chips
//
//        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(stonesAndChips);
//
//        /**
//         * We use units of mm here because that's the recommended units of measurement for the
//         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
//         *      <ImageTarget name="stones" size="247 173"/>
//         * You don't *have to* use mm here, but the units here and the units used in the XML
//         * target configuration files *must* correspond for the math to work out correctly.
//         */
//        float mmPerInch = 25.4f;
//        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
//        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
//
//        /**
//         * In order for localization to work, we need to tell the system where each target we
//         * wish to use for navigation resides on the field, and we need to specify where on the robot
//         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
//         * Transformation matrices are a central, important concept in the math here involved in localization.
//         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
//         * for detailed information. Commonly, you'll encounter transformation matrices as instances
//         * of the {@link OpenGLMatrix} class.
//         *
//         * For the most part, you don't need to understand the details of the math of how transformation
//         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
//         * <ol>
//         *
//         *     <li>You can put two transformations together to produce a third that combines the effect of
//         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
//         *     then the combined transformation matrix RT which does the rotation first and then the translation
//         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
//         *     <em>reverse</em> of the chronological order in which they applied.</li>
//         *
//         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
//         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
//         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
//         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
//         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
//         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
//         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
//         *
//         *     <li>If you want to break open the black box of a transformation matrix to understand
//         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
//         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
//         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
//         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
//         *
//         * </ol>
//         *
//         * This example places the "stones" image on the perimeter wall to the Left
//         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
//         *
//         * This example places the "chips" image on the perimeter wall to the Right
//         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
//         *
//         * See the doc folder of this project for a description of the field Axis conventions.
//         *
//         * Initially the target is conceptually lying at the origin of the field's coordinate system
//         * (the center of the field), facing up.
//         *
//         * In this configuration, the target's coordinate system aligns with that of the field.
//         *
//         * In a real situation we'd also account for the vertical (Z) offset of the target,
//         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
//         *
//         * To place the Stones Target on the Red Audience wall:
//         * - First we rotate it 90 around the field's X axis to flip it upright
//         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
//         * - Finally, we translate it back along the X axis towards the red audience wall.
//         */
//        OpenGLMatrix redTargetLocationOnField = OpenGLMatrix
//                    /* Then we translate the target off to the RED WALL. Our translation here
//                    is a negative translation in X.*/
//                .translation(- mmFTCFieldWidth / 2, 0, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 90, 0));
//        redTarget.setLocation(redTargetLocationOnField);
//        RobotLog.ii(TAG, "Red Target=%s", format(redTargetLocationOnField));
//
//           /*
//            * To place the Stones Target on the Blue Audience wall:
//            * - First we rotate it 90 around the field's X axis to flip it upright
//            * - Finally, we translate it along the Y axis towards the blue audience wall.
//            */
//        OpenGLMatrix blueTargetLocationOnField = OpenGLMatrix
//                    /* Then we translate the target off to the Blue Audience wall.
//                    Our translation here is a positive translation in Y.*/
//                .translation(0, mmFTCFieldWidth / 2, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        blueTarget.setLocation(blueTargetLocationOnField);
//        RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));
//
//        /**
//         * Create a transformation matrix describing where the phone is on the robot. Here, we
//         * put the phone on the right hand side of the robot with the screen facing in (see our
//         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
//         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
//         *
//         * When determining whether a rotation is positive or negative, consider yourself as looking
//         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
//         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
//         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
//         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
//         */
//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(mmBotWidth / 2, 0, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.YZY,
//                        AngleUnit.DEGREES, - 90, 0, 0));
//        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));
//
//        /**
//         * Let the trackable listeners we care about know where the phone is. We know that each
//         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
//         * we have not ourselves installed a listener of a different type.
//         */
//        ((VuforiaTrackableDefaultListener) redTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener) blueTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//
//        /**
//         * A brief tutorial: here's how all the math is going to work:
//         *
//         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
//         * P = tracker.getPose()     maps   image target coords -> phone coords
//         * L = redTargetLocationOnField maps   image target coords -> field coords
//         *
//         * So
//         *
//         * C.inverted()              maps   robot coords -> phone coords
//         * P.inverted()              maps   phone coords -> imageTarget coords
//         *
//         * Putting that all together,
//         *
//         * L x P.inverted() x C.inverted() maps robot coords to field coords.
//         *
//         * @see VuforiaTrackableDefaultListener#getRobotLocation()
//         */
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start tracking");
//        telemetry.update();
//        waitForStart();
//
//        /** Start tracking the data sets we care about. */
//        stonesAndChips.activate();
//
//        while (opModeIsActive()) {
//
//            for (VuforiaTrackable trackable : allTrackables) {
//                /**
//                 * getUpdatedRobotLocation() will return null if no new information is available since
//                 * the last time that call was made, or if the trackable is not currently visible.
//                 * getRobotLocation() will return null if the trackable is not currently visible.
//                 */
//                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
//
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//            }
//            /**
//             * Provide feedback as to where the robot was last located (if we know).
//             */
//            if (lastLocation != null) {
//                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
//                telemetry.addData("Pos", format(lastLocation));
//            } else {
//                telemetry.addData("Pos", "Unknown");
//            }
//            telemetry.update();
//        }
//
//
//    /**
//     * A simple utility that extracts positioning information from a transformation matrix
//     * and formats it in a form palatable to a human being.
//     */
//    String format (OpenGLMatrix transformationMatrix)
//    {
//        return transformationMatrix.formatAsTransform();
//    }
//}
    //Empty main
    @Override
    public void runOpMode() throws InterruptedException
    {

    }
} //Close class and end program

