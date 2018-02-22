//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.locks.Lock;

@TeleOp(name="TeleOp") //Name the class
public class Teleop extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor glyphWheelLeft;
    DcMotor glyphWheelRight;
    DcMotor glyphLift;
    Servo glyphFlip;

    //Define relic motors
    Servo relicGrab;
    CRServo relicFlip;
    DcMotor relicSpool;

    //Define the jewel motor
    Servo jewelArm;

    //Define the color sensor
    ColorSensor colorSensor;

    //Define floats to be used as joystick inputs and trigger inputs
    float drivePower;
    float shiftPower;
    float leftTurnPower;
    float rightTurnPower;
    float liftPower;

    //Define an int to be used as a toggle
    int intakeToggle = 0;

    //Define an int to use as gamepad2 initialization
    int gamepad2Init = 0;

    //Define an elapsed time variable
    private ElapsedTime runtime = new ElapsedTime();

    //Define booleans to make relic movements and shut off the intake wheels when gamepad2 is initialized
    boolean bMoved = false;
    boolean intakeWheelsOn = true;

//***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get references to the DC Motors from the hardware map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        glyphWheelLeft = hardwareMap.dcMotor.get("glyphWheelLeft");
        glyphWheelRight = hardwareMap.dcMotor.get("glyphWheelRight");
        glyphLift = hardwareMap.dcMotor.get("glyphLift");
        relicSpool = hardwareMap.dcMotor.get("relicSpool");

        //Get references to the Servo Motors from the hardware map
        glyphFlip = hardwareMap.servo.get("glyphFlip");
        relicGrab = hardwareMap.servo.get("relicGrab");
        relicFlip = hardwareMap.crservo.get("relicFlip");
        jewelArm = hardwareMap.servo.get("jewelArm");

        //Get references to the Color Sensor from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphWheelLeft, glyphWheelRight, glyphLift, glyphFlip, relicGrab, relicFlip, relicSpool, jewelArm, colorSensor);

        //Set the sensor to active mode and set the directions of the motors
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

        //Reset the flipper
        glyphFlip.setPosition(0.95);

        //Reset the runtime after the start button is clicked
        runtime.reset();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {

    //ARM CONTROLS
            //Lift the arm for the whole teleop phase so that it doesn't fall out of the robot
            jewelArm.setPosition(0.0);

    //DRIVE MOTOR CONTROLS
            //Set float variables as the inputs from the joysticks and the triggers
            drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
            shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
            leftTurnPower = (gamepad1.left_trigger + gamepad2.left_trigger);
            rightTurnPower = (gamepad1.right_trigger + gamepad2.right_trigger);
            liftPower = -gamepad1.right_stick_y;

            //Gamepad2 init
            if ((Math.abs(gamepad2.left_stick_y) > 0.1) && (gamepad2Init == 0))
            {
                bMoved = true;
            }

            //If gamepad2 is used, flip down the relic grabber and open the claws
            if (bMoved)
            {
                functions.crServoTime(relicFlip, (float) 1.0, 1500);
                relicGrab.setPosition(0.32);
                functions.intake((float) 0.0);
                intakeWheelsOn = false;
                bMoved = false;
                gamepad2Init++;

            }

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                functions.driveTeleop(drivePower);
            }

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                functions.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed on gamepad1, turn left at that power (fast)
            if (leftTurnPower > 0)
            {
                functions.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed on gamepad1, turn right at that power (fast)
            if (rightTurnPower > 0)
            {
                functions.rightTurnTeleop(rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                functions.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
            }

            //If the dpad is pushed down, flip the glyphs into the cryptobox
            //Then reset the flipper
            if (gamepad1.dpad_down)
            {
                glyphFlip.setPosition(0.3);
                Thread.sleep(1200);
                glyphFlip.setPosition(0.95);
            }

            //If the dpad is pushed up, lift the flipper slightly to make it parallel to the groun
            //Then lift the glyphs and flip them into the cryptobox
            //Also reset the flipper and lower the glyphter
            if (gamepad1.dpad_up)
            {
                glyphFlip.setPosition(0.9);
                functions.oneMotorEncoder(glyphLift, (float) -0.7, -1560);
                glyphFlip.setPosition(0.3);
                Thread.sleep(1200);
                glyphFlip.setPosition(0.95);
                functions.oneMotorEncoder(glyphLift, (float) 0.4, 1515);
            }

            //If gamepad1 right bumper is pressed, set the intakeToggle to
            if (gamepad1.right_bumper)
            {
                intakeToggle = 0;
            }

            //If gamepad1 left bumper is pressed, increase the increment operator for intake toggling
            if (gamepad1.left_bumper)
            {
                intakeToggle = 1;
            }

            //If the intake wheels are supposed to be on, set up the toggling function that allows us to intake and eject whenever we want
            if (intakeWheelsOn)
            {
                if (intakeToggle == 0)
                {
                    glyphWheelLeft.setPower(-1.0);
                    glyphWheelRight.setPower(1.0);
                }
                if (intakeToggle == 1)
                {
                    glyphWheelLeft.setPower(1.0);
                    glyphWheelRight.setPower(-1.0);
                }
            }

        //RELIC CONTROLS
            //If the gamepad2 right joystick is pushed up, spool the relic system out
            if (gamepad2.right_stick_y <= -0.05)
            {
                relicSpool.setPower(1.0);
                Thread.sleep(100);
                relicSpool.setPower(0.0);
            }

            //If the gamepad2 right joystick is pushed down, spool the relic system back in
            if (gamepad2.right_stick_y >= 0.05)
            {
                relicSpool.setPower(-0.5);
                Thread.sleep(100);
                relicSpool.setPower(0.0);
            }

            //If the x button is pressed, open the claws
            if (gamepad2.x)
            {
                relicGrab.setPosition(0.32);
            }

            //If the b button is pressed, close the
            if (gamepad2.b)
            {
                relicGrab.setPosition(1.00);
            }

            //If the y button is pressed, flip the relic flipper down
            //Also open the claws after a certain time to drop the relic
            if (gamepad2.y)
            {
                functions.crServoTime(relicFlip, (float) 1.0, 1500);
                Thread.sleep(1800);
                relicGrab.setPosition(0.32);
            }

            //If the a button is pressed, flip the relic flipper up
            if (gamepad2.a)
            {
                //Up while holding relic, since it requires more time
                functions.crServoTime(relicFlip, (float) -1.0, 4000);
            }

            //If the gamepad2 left bumper is pressed, spool out for a full zone 3 extension
            //Then flip the relic down
            //Then open the relic claws to drop the relic
            if (gamepad2.left_bumper)
            {
                functions.oneMotorEncoder(relicSpool, (float) 1.0, 6000);
                functions.crServoTime(relicFlip, (float) 1.0, 1500);
                Thread.sleep(1800);
                relicGrab.setPosition(0.32);
            }

            //If the gamepad2 left bumper is pressed, flip the relic flipper up
            //Then spool the relic system all the way in
            if (gamepad2.right_bumper)
            {
                functions.crServoTime(relicFlip, (float) -1.0, 2000);
                functions.oneMotorEncoder(relicSpool, (float) -1.0, -6000);
            }

            //If the gamepad2 dpad is pushed down, automatically climb onto the platform
            //InstaClimb
            if (gamepad2.dpad_down)
            {
                functions.driveAutonomous((float) -0.9, -1200);
            }

            //Count time
            //Update the data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
