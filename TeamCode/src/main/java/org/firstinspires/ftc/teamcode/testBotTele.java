//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
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
    //DcMotor glyphLift;

    //Relic Motors
//    Servo relicGrab;
    DcMotor relicLift;

    //Jewel Motor
    Servo jewelArm;

    //Define Sensors and the CDI
    ColorSensor colorSensor;

    //Define floats to be used as joystick and trigger inputs
    float drive;
    float shift;
    float rightTurn;
    float leftTurn;

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
        //glyphGrab = hardwareMap.dcMotor.get("glyphGrab");
//        glyphLift = hardwareMap.dcMotor.get("glyphLift");
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
            drive = - gamepad1.left_stick_y;
            shift = gamepad1.left_stick_x / 2;
            leftTurn = gamepad1.left_trigger;
            rightTurn = gamepad1.right_trigger;


            //Do nothing if joystick is stationary

            //Shift if pushed more on X than Y
            if (Math.abs(shift) > Math.abs(drive))
            {
                setDriveMotorPowers(- shift, shift / (float) 1.5, shift, - shift / (float) 1.5);
            }

            //Drive if joystick pushed more Y than X
            if (Math.abs(drive) > Math.abs(shift))
            {
                setDriveMotorPowers(drive, drive, drive, drive);
            }

            if (drive == 0 && shift == 0)
            {
                setDriveMotorPowers(0, 0, 0, 0);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurn > 0)
            {
                setDriveMotorPowers(- leftTurn, - leftTurn, leftTurn, leftTurn);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurn > 0)
            {
                setDriveMotorPowers(rightTurn, rightTurn, - rightTurn, - rightTurn);
            }

            if (gamepad1.y)
            {
                //Increase the increment operator
                yPress++;

                //If the "y" button is pressed, grab/drop a glyph
                if (yPress % 2 == 0 && yPress >= 0)
                {
                    glyphGrab.setPower(0.2);
                    Thread.sleep(500);
                    glyphGrab.setPower(0.0);
                }
                else if (yPress % 2 == 1 && yPress >= 0)
                {
                    glyphGrab.setPower(- 0.2);
                    Thread.sleep(500);
                    glyphGrab.setPower(0.0);
                }
            }

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