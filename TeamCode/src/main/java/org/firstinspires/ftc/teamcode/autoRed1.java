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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


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

    int vuforiaValues[] = {1200, 1550, 1900};
    int distanceToCryptobox;

    //Define powers to avoid magic numbers
    float drivePower = (float) 0.5;
    float shiftPower = (float) 0.5;
    float turnPower = (float) 0.5;

    //Vuforia Initialization
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    int count = 0;
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

        //Set the sensor to active mode and set the directions of the motors
        functions.initializeMotorsAndSensors();

        //Vuforia Initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Adp/KFX/////AAAAGYMHgTasR0y/o1XMGBLR4bwahfNzuw2DQMMYq7vh4UvYHleflzPtt5rN2kFp7NCyO6Ikkqhj/20qTYc9ex+340/hvC49r4mphdmd6lI/Ip64CbMTB8Vo53jBHlGMkGr0xq/+C0SKL1hRXj5EkXtSe6q9F9T/nAIcg9Jr+OfAcifXPH9UJYG8WmbLlvpqN+QuVA5KQ6ve1USpxYhcimV9xWCBrq5hFk1hGLbeveHrKDG3wYRdwBeYv3Yo5qYTsotfB4CgJT9CX/fDR/0JUL7tE29d1v1eEF/VXCgQP4EPUoDNBtNE6jpKJhtQ8HJ2KjmJnW55f9OqNc6SsULV3bkQ52PY+lPLt1y4muyMrixCT7Lu";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        //Wait for start button to be clicked
        waitForStart();

        //Activate Trackables
        relicTrackables.activate();

//***************************************************************************************************************************
        while (opModeIsActive()) {
            //Close door
//            functions.glyphDoor("close");

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            while (vuMark == RelicRecoveryVuMark.UNKNOWN && count < 500) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                sleep(10);
                count++;
            }



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
            functions.leftTurnAutonomous(turnPower/2, 1000);

            Thread.sleep(300);

            //Go to the cryptobox
            functions.driveAutonomous(drivePower, 600);

            //Drop the glyph in the cryptobox while ending in the safe zone
//            functions.glyphDoor("open");

            functions.leftTurnAutonomous(turnPower, 300);

            functions.driveAutonomous(drivePower, 400);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program