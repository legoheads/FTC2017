//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp (name="Legoheads TeleOp ") //Name the class
public class Teleop extends LinearOpMode
{
    //Define DC Motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;
    DcMotor shooterLeft;
    DcMotor shooterRight;
    DcMotor spinnerTop;
    DcMotor spinnerBottom;

    //Define an int to use as the spinner's mode, and an int to use as the shooters mode
    int spinnerCount = 1;
    int shooterCount = 1;

    //Define Sensors and the CDI
    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;
    ColorSensor colorSensorBottom;
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
        spinnerTop = hardwareMap.dcMotor.get("spinnerTop");
        spinnerBottom = hardwareMap.dcMotor.get("spinnerBottom");
        shooterLeft = hardwareMap.dcMotor.get("shooterLeft");
        shooterRight = hardwareMap.dcMotor.get("shooterRight");

        //Get references to the sensors and the CDI from the hardware map
        colorSensorBottom = hardwareMap.colorSensor.get("colorSensorBottom");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");
        CDI = hardwareMap.deviceInterfaceModule.get("CDI");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors, CDI)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, spinnerTop, spinnerBottom, shooterLeft, shooterRight, colorSensorLeft, colorSensorRight, colorSensorBottom, CDI);

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


            //If the dpad is pushed left or right, stop the spinner
            if (gamepad2.dpad_left || gamepad2.dpad_right)
            {
                spinnerCount = 0;
                functions.spinner(spinnerCount);
            }
            //If the dpad is pressed down, start the spinner forward
            if (gamepad2.dpad_down)
            {
                spinnerCount = 1;
                functions.spinner(spinnerCount);
            }
            //If the dpad is pressed up, start the spinner backward
            if (gamepad2.dpad_up)
            {
                spinnerCount = 2;
                functions.spinner(spinnerCount);
            }


            //If the "y" button is pressed, shoot the ball
            if (gamepad2.y)
            {
                shooterCount+=1;
                functions.shooterTeleOp(shooterCount, (float) 1.0);
            }


            //Stop all motors when any bumper is pressed
            if ((gamepad1.b) || (gamepad2.b))
            {
                functions.stopEverything();
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program