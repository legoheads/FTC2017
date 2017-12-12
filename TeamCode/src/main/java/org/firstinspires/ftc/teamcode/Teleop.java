//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp") //Name the class
public class Teleop extends LinearOpMode {
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor glyphGrab;
    DcMotor glyphLift;

    //Define relic motors
    Servo relicGrab;
    Servo relicFlip;
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

    //Define an elapsed time variable
    private ElapsedTime runtime = new ElapsedTime();

    //Define ints to be used as toggles
    int glyphGrabToggle = 0;
    int relicFlipToggle = 0;
    int relicDropToggle = 0;
    int relicSpoolToggle = 0;

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
        glyphGrab = hardwareMap.dcMotor.get("glyphGrab");
        glyphLift = hardwareMap.dcMotor.get("glyphLift");
        relicSpool = hardwareMap.dcMotor.get("relicSpool");

        //Get references to the Servo Motors from the hardware map
        jewelArm = hardwareMap.servo.get("jewelArm");
        relicGrab = hardwareMap.servo.get("relicGrab");
        relicFlip = hardwareMap.servo.get("relicFlip");

        //Get references to the sensor from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphGrab, glyphLift, relicGrab, relicFlip, relicSpool, jewelArm, colorSensor);

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
            //Set float variables as the inputs from the joysticks and the triggers
            drivePowerFast = gamepad1.left_stick_y * (float) 0.8;
            shiftPowerFast = gamepad1.left_stick_x;
            drivePowerSlow = gamepad2.left_stick_y / 3;
            shiftPowerSlow = gamepad2.left_stick_x / 3;
            fastLeftTurnPower = gamepad1.left_trigger / 2;
            fastRightTurnPower = gamepad1.right_trigger / 2;
            slowLeftTurnPower = gamepad2.left_trigger / 3;
            slowRightTurnPower = gamepad2.right_trigger / 3;
            liftPower = gamepad1.right_stick_y;
            jewelArm.setPosition(0.1);

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

            //Do nothing if joysticks are untouched
            if (drivePowerFast == 0 && shiftPowerFast == 0 && drivePowerSlow == 0 && shiftPowerSlow == 0)
            {
                functions.setDriveMotorPowers(0, 0, 0, 0);
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

            //Grabbing/dropping glyphs on gamepad1 right bumper
            if (gamepad1.right_bumper)
            {
                //Increase the increment operator
                glyphGrabToggle++;

                //If the right bumper is pressed, open/close the door
                if (glyphGrabToggle % 2 == 0)
                {
                    functions.glyphDoor("open");
                }
                if (glyphGrabToggle % 2 == 1)
                {
                    functions.glyphDoor("close");
                }
            }

            //If the right joystick is moved significantly, move the lifter up or down depending on how it is pushed
            //If it is not pushed significantly, don't move it
            if (Math.abs(liftPower)>=0.1)
            {
                glyphLift.setPower(liftPower);
            }
            if (Math.abs(liftPower) < 0.1)
            {
                glyphLift.setPower(0.0);
            }

            //If the x button is pressed, grab/drop the relic
            if (gamepad1.x)
            {
                relicDropToggle++;
                if (relicDropToggle %2 == 0)
                {
                    relicGrab.setPosition(0.0);

                }
                if (relicDropToggle %2 == 1)
                {
                    relicGrab.setPosition(1.0);
                }
            }

            //If the y button is pressed, flip the relic up and down to clear the wall
            if (gamepad1.y)
            {
                relicFlipToggle++;
                if (relicFlipToggle % 2 == 0)
                {
                    relicFlip.setPosition(1.0);

                }
                if (relicFlipToggle % 2 == 1)
                {
                    relicFlip.setPosition(0.3);

                }
            }

            if (gamepad1.a)
            {
                relicFlip.setPosition(0.0);
                relicGrab.setPosition(1.0);

            }

            //If the dpad is pushed to the left, unwind the spool
            //If it is pushed to the left, rewind the spool
            if (gamepad1.dpad_left) {
                relicSpoolToggle++;
                if (relicSpoolToggle % 2 == 1) {
                    relicSpool.setPower(1.0);
                }
                if (relicSpoolToggle % 2 == 0) {
                    relicSpool.setPower(0.0);
                }
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
