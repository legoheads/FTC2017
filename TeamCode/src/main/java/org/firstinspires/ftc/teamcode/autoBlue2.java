//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto Blue2") //Name the program
public class autoBlue2 extends LinearOpMode
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
    Servo relicSpool;
    Servo relicFlip;

    //Jewel Motor
    Servo jewelArm;

    //Define Sensors and the CDI
    ColorSensor colorSensor;

    String color = "Blue";
    String colorSeen;

    //Define up drive powers to avoid magic numbers
    float drivePower = (float) 0.8;
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
        glyphGrab = hardwareMap.dcMotor.get("glyphGrab");
        glyphLift = hardwareMap.dcMotor.get("glyphLift");
        relicGrab = hardwareMap.servo.get("relicGrab");
        relicSpool = hardwareMap.servo.get("relicSpool");
        relicFlip = hardwareMap.servo.get("relicFlip");

        //Get references to the Servo Motors from the hardware map
        jewelArm = hardwareMap.servo.get("jewelArm");

        //Get references to the sensors and the CDI from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors, CDI)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphGrab, glyphLift, relicGrab, relicSpool, relicFlip, jewelArm, colorSensor);

        Vuforia vuforia = new Vuforia();

        //Set the sensors to the modes that we want, and set their addresses. Also set the directions of the motors
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive()) {
            //Close door
            functions.glyphDoor("close");
            //Do jewels
            functions.jewelPush(colorSensor, color, colorSeen);

            //Go to jewels
            functions.driveAutonomous(drivePower, 500);

            functions.leftTurnAutonomous(turnPower, 300);

            functions.driveAutonomous(drivePower, 300);

            functions.glyphDoor("open");
//
//            //Do jewels
//            functions.jewelPush(colorSensor, color, colorSeen);
//
//            //Move to pictograph
//            functions.rightShiftAutonomous(shiftPower, 300);
//
//            vuforia.runOpMode();
//
//            //Move towards cryptobox
//            functions.leftShiftAutonomous(shiftPower, 800);
//
//            //Move away from the cryptobox
//            functions.driveAutonomous(drivePower, 500);
//
//            //Turn to face cryptobox
//
//            //Align with the cryptobox
//            functions.rightShiftAutonomous(shiftPower, 600);
//
//            //Drive into the cryptobox
//            functions.driveAutonomous(drivePower, 1300);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program