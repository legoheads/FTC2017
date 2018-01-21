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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
    float drivePowerFast;
    float shiftPowerFast;
    float drivePowerSlow;
    float shiftPowerSlow;
    float fastLeftTurnPower;
    float fastRightTurnPower;
    float slowLeftTurnPower;
    float slowRightTurnPower;
    float liftPower;

    //Define ints to be used as toggles
    int glyphGrabToggle = 0;
    int glyphFlipToggle = 0;
    int relicGrabToggle = 0;
    int relicFlipToggle = 0;

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
            jewelArm.setPosition(-0.3);

    //DRIVE MOTOR CONTROLS
            //Set float variables as the inputs from the joysticks and the triggers
            drivePowerFast = gamepad1.left_stick_y * (float) 0.8;
            shiftPowerFast = gamepad1.left_stick_x;
            drivePowerSlow = gamepad2.left_stick_y / 3;
            shiftPowerSlow = gamepad2.left_stick_x / 3;
            fastLeftTurnPower = gamepad1.left_trigger;
            fastRightTurnPower = gamepad1.right_trigger;
            slowLeftTurnPower = gamepad2.left_trigger / 3;
            slowRightTurnPower = gamepad2.right_trigger / 3;
            liftPower = -gamepad1.right_stick_y;

            //Do nothing if joysticks are untouched
            if (drivePowerFast == 0 && shiftPowerFast == 0 && drivePowerSlow == 0 && shiftPowerSlow == 0)
            {
                functions.setDriveMotorPowers(0, 0, 0, 0);
            }

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(shiftPowerFast) > Math.abs(drivePowerFast))
            {
                functions.shiftTeleop(shiftPowerFast);
            }

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(drivePowerFast) > Math.abs(shiftPowerFast))
            {
                functions.driveTeleop(drivePowerFast);
            }

            //Shift if pushed more on X than Y on gamepad2 (slow)
            if (Math.abs(shiftPowerSlow) > Math.abs(drivePowerSlow))
            {
                functions.shiftTeleop(shiftPowerSlow);
            }

            //Drive if pushed more on Y than X on gamepad2 (slow)
            if (Math.abs(drivePowerSlow) > Math.abs(shiftPowerSlow))
            {
                functions.driveTeleop(drivePowerSlow);
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

    //GLYPH CONTROLS
//            //Intake system on/off on gamepad1 left bumper
//            if (gamepad1.left_bumper)
//            {
//                //Increase the increment operator
//                glyphGrabToggle++;
//
//                //If the left bumper is pressed an odd number of times, turn on the intake system wheels
//                if (glyphGrabToggle % 2 == 1)
//                {
//                    functions.intake((float) 1.0);
//                }
//
//                //If the left bumper is pressed an even number of times, turn off the intake system wheels
//                if (glyphGrabToggle % 2 == 0)
//                {
//                    functions.intake((float) 0.0);
//                }
//            }
//            functions.intake((float) 1.0);
            glyphWheelLeft.setPower(-1.0);
            glyphWheelRight.setPower(1.0);

            //If the right joystick is pushed significantly, operate the lifter at the given power
            if (Math.abs(liftPower) >= 0.05)
            {
                glyphFlip.setPosition(0.3);
                functions.glyphLift(liftPower);
            }

            //If the right joystick is not pushed significantly, keep it stationary
            if (Math.abs(liftPower) < 0.05)
            {
                functions.glyphLift((float) 0.0);
            }

            //Glyph flipper forwards/backwards
            if (gamepad2.x)
            {
                glyphFlip.setPosition(0.0);
            }
            if (gamepad2.b)
            {
                glyphFlip.setPosition(1.0);
            }

    //RELIC CONTROLS
            //If the x button is pressed, grab/drop the relic
            if (gamepad1.x)
            {
                relicGrab.setPosition(0.32);
            }

            //Open the claws to drop the relic
            if (gamepad1.b)
            {
                relicGrab.setPosition(1.00);
            }

            //If the y button is pressed, operate the relic flipper
            if (gamepad1.y)
            {
                //Increase the increment operator
                relicFlipToggle++;

                //First down movement, from the start position to perpendicular to the ground
                if (relicFlipToggle == 1)
                {
                    relicFlip.setPower(-1.0);
                    Thread.sleep(1800);
                    relicFlip.setPower(0.0);
                }

                //Up movement
                if (relicFlipToggle % 2 == 0)
                {
                    relicFlip.setPower(1.0);
                    Thread.sleep(700);
                    relicFlip.setPower(0.0);
                }

                //Down movement
                if ((relicFlipToggle % 2 == 1  && relicFlipToggle != 1))
                {
                    relicFlip.setPower(-1.0);
                    Thread.sleep(700);
                    relicFlip.setPower(0.0);
                }

            }

            //If the a button is pressed, lift the relic over the wall
            if (gamepad1.a)
            {
                //Up while holding relic, since it requires more time
                relicFlip.setPower(1.0);
                Thread.sleep(2300);
                relicFlip.setPower(0.0);
            }

            //If the dpad is pushed to the right, unwind the spool
            if (gamepad1.dpad_right)
            {
                relicSpool.setPower(1.0);
                Thread.sleep(700);
                relicSpool.setPower(0.0);
            }

            //If the dpad is pushed to the left, rewind the spool
            if (gamepad1.dpad_left)
            {
                relicSpool.setPower(-1.0);
                Thread.sleep(700);
                relicSpool.setPower(0.0);
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
