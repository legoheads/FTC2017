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

    //Define ints to be used as toggles
    int intakeToggle = 0;

    int gamepad2Init = 0;
    //Define an elapsed time variable
    private ElapsedTime runtime = new ElapsedTime();

    boolean bMoved = false;
    boolean intakeWheelsOn = true;

//***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get references to the DC motors from the hardware map
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

        //Get references to the sensor from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphWheelLeft, glyphWheelRight, glyphLift, glyphFlip, relicGrab, relicFlip, relicSpool, jewelArm, colorSensor);

        //Set the sensor to active mode and set the directions of the motors
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

        glyphFlip.setPosition(0.95);

        //Reset the runtime after the start button is clicked
        runtime.reset();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {

            telemetry.addData("ENC", glyphLift.getCurrentPosition());
    //ARM CONTROLS
            //Lift the arm for the whole teleop phase so that it doesn't fall out of the robot
            jewelArm.setPosition(0.0);

    //DRIVE MOTOR CONTROLS
            //Set float variables as the inputs from the joysticks and the triggers
            drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
            shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
            leftTurnPower = (gamepad1.left_trigger + gamepad2.left_trigger) / 2;
            rightTurnPower = (gamepad1.right_trigger + gamepad2.right_trigger) / 2;
            liftPower = -gamepad1.right_stick_y;


            if ((Math.abs(gamepad2.left_stick_y) > 0.1) && (gamepad2Init == 0))
            {
                bMoved = true;
            }

            if (bMoved)
            {
                functions.crServoTime(relicFlip, (float) 1.0, 1500);
                relicGrab.setPosition(0.32);
                functions.intake((float) 0.0);
                intakeWheelsOn = false;
                bMoved = false;
                gamepad2Init++;

            }

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                functions.shiftTeleop(shiftPower);
            }

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                functions.driveTeleop(drivePower);
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

            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                functions.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
            }

            if (gamepad1.right_stick_y == 0.0)
            {
                relicSpool.setPower(0.0);
            }

            //If the right joystick is pushed significantly, operate the lifter at the given power
            if (gamepad1.dpad_up)
            {
                glyphFlip.setPosition(0.9);
                functions.oneMotorEncoder(glyphLift, (float) -0.7, -1800);
                glyphFlip.setPosition(0.3);
                sleep(1200);
                glyphFlip.setPosition(0.95);
                functions.oneMotorEncoder(glyphLift, (float) 0.5, 1800);
            }

            //If the right joystick is not pushed significantly, keep it stationary
            if (gamepad1.dpad_down)
            {
                glyphFlip.setPosition(0.3);
                sleep(1200);
                glyphFlip.setPosition(0.95);
            }

            if (gamepad1.left_bumper)
            {
                intakeToggle++;
            }

            if (intakeWheelsOn)
            {
                if (intakeToggle % 2 == 0)
                {
                    glyphWheelLeft.setPower(- 1.0);
                    glyphWheelRight.setPower(1.0);
                }
                if (intakeToggle % 2 == 1)
                {
                    glyphWheelLeft.setPower(1.0);
                    glyphWheelRight.setPower(-1.0);
                }
            }

        //RELIC CONTROLS
            //If the dpad is pushed to the right, unwind the spool
            if (gamepad2.right_stick_y <= -0.05)
            {
                relicSpool.setPower(1.0);
                Thread.sleep(100);
                relicSpool.setPower(0.0);
            }

            //If the dpad is pushed to the left, rewind the spool
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

            //Open the claws to grab the relic
            if (gamepad2.b)
            {
                relicGrab.setPosition(1.00);
                intakeToggle=1;
            }

            //If the y button is pressed, operate the relic flipper
            if (gamepad2.y)
            {
                functions.crServoTime(relicFlip, (float) 1.0, 1500);
                Thread.sleep(1800);
                relicGrab.setPosition(0.32);
            }

            //If the a button is pressed, lift the relic over the wall
            if (gamepad2.a)
            {
                //Up while holding relic, since it requires more time
                functions.crServoTime(relicFlip, (float) -1.0, 4000);
            }

            if (gamepad2.left_bumper)
            {
                functions.oneMotorEncoder(relicSpool, (float) 1.0, 6000);
                functions.crServoTime(relicFlip, (float) 1.0, 1500);
                Thread.sleep(1800);
                relicGrab.setPosition(0.32);
            }
            if (gamepad2.right_bumper)
            {
                functions.oneMotorEncoder(relicSpool, (float) -1.0, -6000);
            }

            //InstaClimb ©
            if (gamepad2.dpad_down)
            {
                functions.driveAutonomous((float) -0.9, -1000);
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
