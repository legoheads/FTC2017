//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name="TeleOp") //Name the class
public class Teleop extends LinearOpMode
{
    //Define DC Motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;
    DcMotor glyphGrabber;

    //Define Servo Motors
    Servo leftGlyphGrabber;
    Servo rightGlyphGrabber;

    //Define Sensors and the CDI
    ColorSensor colorSensor;
    DeviceInterfaceModule CDI;

    //Define floats to be used as joystick and trigger inputs
    float drive;
    float shift;
    float rightTurn;
    float leftTurn;

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
        glyphGrabber = hardwareMap.dcMotor.get("glyphGrabber");

        //Get references to the Servo Motors from the hardware map
        leftGlyphGrabber = hardwareMap.servo.get("leftGlyphGrabber");
        rightGlyphGrabber = hardwareMap.servo.get("rightGlyphGrabber");

        //Get references to the sensors and the CDI from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        CDI = hardwareMap.deviceInterfaceModule.get("CDI");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors, CDI)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphGrabber, leftGlyphGrabber, rightGlyphGrabber, colorSensor, CDI);

        //Set the sensors to the modes that we want, and set their addresses. Also set the directions of the motors
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
    //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            //Set float variables as the inputs from the joysticks and the triggers
            drive = -gamepad1.left_stick_y;
            shift = - gamepad1.left_stick_x;
            leftTurn = gamepad1.left_trigger;
            rightTurn = gamepad1.right_trigger;

            //Do nothing if joystick is stationary
            //Drive vs Shift on left joystick:
            if ((drive == 0) && (shift == 0) && (leftTurn == 0) && (rightTurn == 0))
            {
                functions.stopDriving();
            }

            //Shift if pushed more on X than Y
            if (Math.abs(shift) > Math.abs(drive))
            {
                functions.shiftTeleop(shift);
            }

            //Drive if joystick pushed more Y than X
            if (Math.abs(drive) > Math.abs(shift))
            {
                functions.driveTeleop(drive);
            }


            //If the left trigger is pushed, turn left at that power
            if (leftTurn > 0)
            {
                functions.leftTurnTeleop(leftTurn);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurn > 0)
            {
                functions.rightTurnTeleop(rightTurn);
            }

            //If the "a" button is pressed, grab the glyph with the DC
            if (gamepad2.a)
            {
                glyphGrabber.setPower(0.5);
                Thread.sleep((long) 0.5);
                glyphGrabber.setPower(0.0);
            }

            //If the "y" button is pressed, grab the glyph with servos
            if (gamepad2.y)
            {
                leftGlyphGrabber.setPosition(0.5);
                rightGlyphGrabber.setPosition(0.5);
            }

            //Stop driving when any "b" button is pressed
            if ((gamepad1.b) || (gamepad2.b))
            {
                functions.stopDriving();
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program