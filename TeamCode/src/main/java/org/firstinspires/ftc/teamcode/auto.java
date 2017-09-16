//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto") //Name the program
public class auto extends LinearOpMode
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

    //Define a string to use as the color, and set it to blue, since we are blue team
    String color = "Blue";

//    //Define an int for the time that the shooter will be on
//    int shootTime = 3000;

    //Define up drive powers to avoid magic numbers
    float drivePower = (float) 0.8;
    float stopOnLinePower = (float) 0.25;
    float shiftPower = (float) 0.6;
    float turnPower = (float) 0.6;

//***************************************************************************************************************************
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

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program