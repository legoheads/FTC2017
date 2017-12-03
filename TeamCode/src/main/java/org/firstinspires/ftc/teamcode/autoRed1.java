//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


import java.util.ArrayList;
import java.util.List;

@Autonomous(name="AutoRed1") //Name the program
public class autoRed1 extends LinearOpMode {
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

    //Define strings to use, as our team color, and the color we see with the sensor
    String color = "Red";
    String colorSeen;
    RelicRecoveryVuMark vuforiaReading;

    int vuforiaValues[] = {1400, 1750, 2100};
    int distanceToCryptobox;

    //Define powers to avoid magic numbers
    float drivePower = (float) 0.5;
    float shiftPower = (float) 0.5;
    float turnPower = (float) 0.5;

    VuforiaLocalizer vuforia;
    //***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
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


        //Define vuforia
//        Vuforia vuforia = new Vuforia();

        //Set the sensor to active mode and set the directions of the motors
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive()) {
            //Close door
//            functions.glyphDoor("close");

            distanceToCryptobox = vuforiaValues[1];

//            vuforiaReading = functions.vuforia();
//
//            if (vuforiaReading == RelicRecoveryVuMark.LEFT) {
//                distanceToCryptobox = vuforiaValues[0];
//            }
//            if (vuforiaReading == RelicRecoveryVuMark.CENTER) {
//                distanceToCryptobox = vuforiaValues[1];
//            }
//            if (vuforiaReading == RelicRecoveryVuMark.RIGHT) {
//                distanceToCryptobox = vuforiaValues[2];
//            }

            //Do jewels and get off platform
            functions.jewelPush(colorSensor, color, colorSeen);

            functions.driveAutonomous(-drivePower, -distanceToCryptobox);

//            //Turn left 90 degrees to position towards cryptobox
//            functions.leftTurnAutonomous(turnPower, 1000);
//
//            //Align on wall
//            functions.driveAutonomous(-drivePower, -700);
//
//            Thread.sleep(300);
//
//            //Drive towards cryptobox
//            functions.driveAutonomous(drivePower/2, 2300);

            Thread.sleep(300);

            //Turn to be aligned with crytobox
            functions.rightTurnAutonomous(turnPower/2, 1080);

            Thread.sleep(300);

            //Go to the cryptobox
            functions.driveAutonomous(drivePower, 600);

            //Drop the glyph in the cryptobox while ending in the safe zone
//            functions.glyphDoor("open");

            functions.leftTurnAutonomous(turnPower, 300);

            functions.driveAutonomous(-drivePower, -400);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program