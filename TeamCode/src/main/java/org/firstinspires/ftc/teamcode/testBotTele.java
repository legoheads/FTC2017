//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tele Lebron") //Name the class
public class testBotTele extends LinearOpMode
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
    DcMotor relicLift;

    //Jewel Motor
    Servo jewelArm;

    //Define Sensors and the CDI
    ColorSensor colorSensor;

    //Define floats to be used as joystick and trigger inputs
    float drivePowerFast;
    float shiftPowerFast;
    float drivePowerSlow;
    float shiftPowerSlow;
    float rightTurnPower;
    float leftTurnPower;
    float liftPower;

    public void setDriveMotorPowers(float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        leftMotorFront.setPower(leftFrontPower);
        leftMotorBack.setPower(leftBackPower);
        rightMotorFront.setPower(rightFrontPower);
        rightMotorBack.setPower(rightBackPower);
    }

    private ElapsedTime runtime = new ElapsedTime();

    private int yPress = 0;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Get references to the DC motors from the hardware map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        glyphGrab = hardwareMap.dcMotor.get("glyphGrab");
        glyphLift = hardwareMap.dcMotor.get("glyphLift");
//        relicGrab = hardwareMap.servo.get("relicGrab");
//        relicLift = hardwareMap.dcMotor.get("relicLift");

        //Get references to the Servo Motors from the hardware map
        //jewelArm = hardwareMap.servo.get("jewelArm");

        //Get references to the sensors and the CDI from the hardware map
        //colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors, CDI)
        //DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphGrab, glyphLift, relicLift, relicGrab, jewelArm, colorSensor, CDI);

        //Set the sensors to the modes that we want, and set their addresses. Also set the directions of the motors
        //Reverse some motors and keep others forward
        leftMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Wait for start button to be clicked
        waitForStart();
        runtime.reset();



//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            //Set float variables as the inputs from the joysticks and the triggers
            drivePowerFast = gamepad1.left_stick_y;
            shiftPowerFast = gamepad1.left_stick_x;
            drivePowerSlow = gamepad1.right_stick_y / 3;
            shiftPowerSlow = gamepad1.right_stick_x / 3;
            leftTurnPower = gamepad1.left_trigger / 4;
            rightTurnPower = gamepad1.right_trigger / 4;
            liftPower = -gamepad2.left_stick_y;


            //Do nothing if joystick is stationary

            //Shift if pushed more on X than Y
            if (Math.abs(shiftPowerFast) > Math.abs(drivePowerFast))
            {
                setDriveMotorPowers(-shiftPowerFast, shiftPowerFast, shiftPowerFast, -shiftPowerFast);
            }

            //Drive if joystick pushed more Y than X
            if (Math.abs(drivePowerFast) > Math.abs(shiftPowerFast))
            {
                setDriveMotorPowers(drivePowerFast, drivePowerFast, drivePowerFast, drivePowerFast);
            }

            if (drivePowerFast == 0 && shiftPowerFast == 0 && drivePowerSlow == 0 && shiftPowerSlow == 0)
            {
                setDriveMotorPowers(0, 0, 0, 0);
            }

            if (Math.abs(shiftPowerSlow) > Math.abs(drivePowerSlow))
            {
                setDriveMotorPowers(-shiftPowerSlow, shiftPowerSlow, shiftPowerSlow, -shiftPowerSlow);
            }

            //Drive if joystick pushed more Y than X
            if (Math.abs(drivePowerSlow) > Math.abs(shiftPowerSlow))
            {
                setDriveMotorPowers(drivePowerSlow, drivePowerSlow, drivePowerSlow, drivePowerSlow);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0)
            {
                setDriveMotorPowers(leftTurnPower, leftTurnPower, -leftTurnPower, -leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0)
            {
                setDriveMotorPowers(-rightTurnPower, -rightTurnPower, rightTurnPower, rightTurnPower);
            }

            if (gamepad2.y)
            {
                //Increase the increment operator
                yPress++;

                //If the "y" button is pressed, grab/drop a glyph
                if (yPress % 2 == 0)
                {
                    glyphGrab.setPower(0.2);
                    Thread.sleep(700);
                    glyphGrab.setPower(0.0);
                }
                else if (yPress % 2 == 1)
                {
                    glyphGrab.setPower(- 0.2);
                    Thread.sleep(700);
                    glyphGrab.setPower(0.0);
                    if (!gamepad2.y)
                    {
                        glyphGrab.setPower(-0.2);
                    }
                }
            }

            if (Math.abs(liftPower)>=0.1)
            {
                glyphLift.setPower(liftPower);
            }
            if (Math.abs(liftPower) < 0.1)
            {
                glyphLift.setPower(0.0);
            }

//            if (gamepad2.dpad_up)
//            {
//                glyphLift.setPower(0.7);
//                Thread.sleep(1700);
//                glyphLift.setPower(0.0);
//            }
//            if (gamepad2.dpad_down)
//            {
//                glyphLift.setPower(-0.7);
//                Thread.sleep(750);
//                glyphLift.setPower(0.0);
//            }

            if ((gamepad1.b) || (gamepad2.b))
            {
                setDriveMotorPowers(0,0,0,0);
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