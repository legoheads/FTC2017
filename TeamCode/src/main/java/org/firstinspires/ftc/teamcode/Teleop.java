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
    float fastDrivePower;
    float fastShiftPower;
    float fastLeftTurnPower;
    float fastRightTurnPower;
    float slowDrivePower;
    float slowShiftPower;
    float slowLeftTurnPower;
    float slowRightTurnPower;
    float liftPower;

    //Define ints to be used as toggles
    int relicFlipToggle = 0;
    int intakeToggle = 0;

    //Define an elapsed time variable
    private ElapsedTime runtime = new ElapsedTime();

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

        glyphFlip.setPosition(1.0);
        //Reset the runtime after the start button is clicked

        relicSpool.setPower(1.0);
        Thread.sleep(300);
        relicSpool.setPower(0.0);
        relicFlip.setPower(-1.0);
        Thread.sleep(700);
        relicFlip.setPower(0.0);

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
            fastDrivePower = gamepad1.left_stick_y + gamepad2.left_stick_y;
            fastShiftPower = gamepad1.left_stick_x + gamepad2.left_stick_x;
            fastLeftTurnPower = (gamepad1.left_trigger + gamepad2.left_trigger)/2;
            fastRightTurnPower = (gamepad1.right_trigger + gamepad2.right_trigger)/2;
            slowDrivePower = gamepad1.left_stick_y / 3;
            slowShiftPower = gamepad1.left_stick_x / 3;
            //slowLeftTurnPower = gamepad2.left_trigger / 3;
            //slowRightTurnPower = gamepad2.right_trigger / 3;
            liftPower = -gamepad1.right_stick_y;

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(fastShiftPower) > Math.abs(fastDrivePower))
            {
                functions.shiftTeleop(fastShiftPower);
            }

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(fastDrivePower) > Math.abs(fastShiftPower))
            {
                functions.driveTeleop(fastDrivePower);
            }

            //Shift if pushed more on X than Y on gamepad2 (slow)
            if (Math.abs(slowShiftPower) > Math.abs(slowDrivePower))
            {
                functions.shiftTeleop(slowShiftPower);
            }

            //Drive if pushed more on Y than X on gamepad2 (slow)
            if (Math.abs(slowDrivePower) > Math.abs(slowShiftPower))
            {
                functions.driveTeleop(slowDrivePower);
            }

            //If the left trigger is pushed on gamepad1, turn left at that power (fast)
            if (fastLeftTurnPower > 0)
            {
                functions.leftTurnTeleop(fastLeftTurnPower);
            }

            //If the right trigger is pushed on gamepad1, turn right at that power (fast)
            if (fastRightTurnPower > 0)
            {
                functions.rightTurnTeleop(fastRightTurnPower);
            }

            //If the left trigger is pushed on gamepad2, turn left at that power (slow)
            if (slowLeftTurnPower > 0)
            {
                functions.leftTurnTeleop(slowLeftTurnPower);
            }

            //If the right trigger is pushed on gamepad2, turn right at that power (slow)
            if (slowRightTurnPower > 0)
            {
                functions.rightTurnTeleop(slowRightTurnPower);
            }

            if (gamepad1.right_stick_y == 0.0)
            {
                relicSpool.setPower(0.0);
            }

            if (Math.abs(fastDrivePower) + Math.abs(fastShiftPower) + Math.abs(fastLeftTurnPower) + Math.abs(fastRightTurnPower) + Math.abs(slowDrivePower) + Math.abs(slowShiftPower) + Math.abs(slowLeftTurnPower) + Math.abs(slowRightTurnPower) < 0.15)
            {
                functions.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
            }

            //If the right joystick is pushed significantly, operate the lifter at the given power
            if (gamepad1.dpad_up)
            {
                glyphFlip.setPosition(0.9);
                functions.oneMotorEncoder(glyphLift, (float) -0.7, -2000);
                glyphFlip.setPosition(0.2);
                Thread.sleep(1000);
                glyphFlip.setPosition(1.0);
                functions.oneMotorEncoder(glyphLift, (float) 0.7, 1700);
            }

            //If the right joystick is not pushed significantly, keep it stationary
            if (gamepad1.dpad_down)
            {
                glyphFlip.setPosition(0.2);
                Thread.sleep(1000);
                glyphFlip.setPosition(1.0);
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper)
            {
                intakeToggle++;

            }

            if (intakeToggle % 2 == 0)
            {
                glyphWheelLeft.setPower(-1.0);
                glyphWheelRight.setPower(1.0);
            }
            if (intakeToggle % 2 == 1)
            {
                glyphWheelLeft.setPower(0.0);
                glyphWheelRight.setPower(0.0);
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

            //If the x button is pressed, grab/drop the relic
            if (gamepad2.x)
            {
                relicGrab.setPosition(0.32);
            }

            //Open the claws to drop the relic
            if (gamepad2.b)
            {
                relicGrab.setPosition(1.00);
                intakeToggle=1;

            }

            //If the y button is pressed, operate the relic flipper
            if (gamepad2.y)
            {
                //Increase the increment operator
                relicFlipToggle++;

                //Down movement
                if (relicFlipToggle % 2 == 1)
                {
                    relicFlip.setPower(1.0);
                    Thread.sleep(1200);
                    relicFlip.setPower(0.0);
                }

                //Up movement
                if (relicFlipToggle % 2 == 0)
                {
                    relicFlip.setPower(-1.0);
                    Thread.sleep(1000);
                    relicFlip.setPower(0.0);
                }
            }

            //If the a button is pressed, lift the relic over the wall
            if (gamepad2.a)
            {
                //Up while holding relic, since it requires more time
                relicFlip.setPower(-1.0);
                Thread.sleep(4000);
                relicFlip.setPower(0.0);
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
