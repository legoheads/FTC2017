package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Vuforia Test") //Name the program
public class vuforiaTest extends LinearOpMode {
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
    String color = "Blue";
    String colorSeen;
    RelicRecoveryVuMark vuforiaReading;

    int vuforiaValues[] = {1400, 1750, 2100};
    int distanceToCryptobox;

    //Define powers to avoid magic numbers
    float drivePower = (float) 0.5;
    float shiftPower = (float) 0.5;
    float turnPower = (float) 0.5;

    int cameraMonitorViewId;


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


        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //Get references to the sensor from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphGrab, glyphLift, relicGrab, relicFlip, relicSpool, jewelArm, colorSensor);

        //vuforia vuforia = new vuforia(viewID, vuforiaLocal);

        vuforiaClass vuf = new vuforiaClass();

        //Set the sensor to active mode and set the directions of the motors
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();



//****************************************************************************************************************************************
        vuforiaReading = vuf.go(cameraMonitorViewId);
        Thread.sleep(3000);
        while (opModeIsActive())
        {
//            telemetry.addData("Vuforia Reading", vuforiaReading)
            if (vuforiaReading == RelicRecoveryVuMark.LEFT)
            {
                functions.leftTurnAutonomous((float) 0.2, 100);
                break;
            }
            if (vuforiaReading == RelicRecoveryVuMark.CENTER)
            {
                functions.driveAutonomous((float) 0.2, 100);
                break;
            }
            if (vuforiaReading == RelicRecoveryVuMark.RIGHT)
            {
                functions.rightTurnAutonomous((float) 0.2, 100);
                break;
            }
            else
            { }
            break;
        }
    }
}
