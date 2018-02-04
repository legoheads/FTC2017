//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



@Autonomous(name="AutoBlue1") //Name the program
public class autoBlue1 extends LinearOpMode
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

    //Define strings to use, as our team color, and the color we see with the sensor
    String color = "Blue";
    String colorSeen;

    //Define the vuforia values
    int vuforiaValues[] = {1290, 1650, 2010};

    //Define an int to use as the distance to the cryptobox
    int distanceToCryptobox;

    //Define powers to avoid magic numbers
    float drivePower = (float) 0.2;
    float shiftPower = (float) 0.2;
    float turnPower = (float) 0.2;

    //Vuforia Initialization
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    int count = 0;

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

//        jewelArm.setPosition(0.0);
        //Wait for start button to be clicked
        waitForStart();

        //Activate Trackables
        relicTrackables.activate();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            //Initialize the vuforia template
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            //Look for the cryptobox key for 5 seconds, then move on. Set the key as equal to vuMark
            while (vuMark == RelicRecoveryVuMark.UNKNOWN && count < 500)
            {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                sleep(10);
                count++;
            }

            //Access the first value if the key is left
            if (vuMark == RelicRecoveryVuMark.LEFT)
            {
                distanceToCryptobox = vuforiaValues[0];
            }

            //Access the second value if the key is center
            if (vuMark == RelicRecoveryVuMark.CENTER)
            {
                distanceToCryptobox = vuforiaValues[1];
            }

            //Access the third value if the key is right
            if (vuMark == RelicRecoveryVuMark.RIGHT)
            {
                distanceToCryptobox = vuforiaValues[2];
            }

            //If vuforia is not picked up, go for center because it is most consistent
            if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                distanceToCryptobox = vuforiaValues[1];
            }

            //Lift the flipper off the ground so we can drive around
            glyphFlip.setPosition(0.5);

            //Do jewels
            functions.jewelPush(colorSensor, color, colorSeen);

            //Drive to the cryptobox
            functions.driveAutonomous(drivePower, distanceToCryptobox);

            //Small delay
            Thread.sleep(300);

            //Turn to be aligned with crytobox
            functions.rightTurnAutonomous(turnPower, 1050);

            //Small delay
            Thread.sleep(300);

            //Go to the cryptobox and put the glyph into the cryptobox
//            functions.driveAutonomous(-drivePower, -80);

            functions.driveAutonomous(-drivePower, -100);

            //Flip the glyph into the cryptobox
            glyphFlip.setPosition(0.0);

            Thread.sleep(2000);

            functions.driveAutonomous(-drivePower, -200);

            //Turn to ensure the glyph enters the cryptobox
            if (distanceToCryptobox == vuforiaValues[0])
            {
                functions.rightTurnAutonomous(turnPower, 300);
            }
            else
            {
                functions.leftTurnAutonomous(turnPower, 300);
            }

            //Push in the glyph one final time
            functions.driveAutonomous(-drivePower, -400);

            functions.driveAutonomous(drivePower, 250);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
}//Close class and end program