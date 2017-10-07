//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto Blue") //Name the program
public class autoBlue extends LinearOpMode {
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
    DeviceInterfaceModule CDI;

    String color = "Blue";

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
        relicLift = hardwareMap.dcMotor.get("relicLift");

        //Get references to the Servo Motors from the hardware map
        jewelArm = hardwareMap.servo.get("jewelArm");

        //Get references to the sensors and the CDI from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        CDI = hardwareMap.deviceInterfaceModule.get("CDI");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors, CDI)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphGrab, glyphLift, relicLift, relicGrab, jewelArm, colorSensor, CDI);

        //Set the sensors to the modes that we want, and set their addresses. Also set the directions of the motors
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {

            //Go to jewels
            functions.driveAutonomous(-drivePower, -400);

            //Do jewels
            functions.jewelPush(colorSensor, color);

            //Move to pictograph
            functions.rightShiftAutonomous(shiftPower, 300);

            //Use Vuforia to read the picture
            //INSERT VUFORIA HERE
            functions.vuforia();

            //Move towards cryptobox
            functions.leftShiftAutonomous(shiftPower, 800);

            //Move away from the cryptobox
            functions.driveAutonomous(drivePower, 500);

            //Turn to face cryptobox
            functions.leftTurnAutonomous(turnPower, 500);

            //Align with the cryptobox
            functions.rightShiftAutonomous(shiftPower, 600);

            //Drive into the cryptobox
            functions.driveAutonomous(drivePower, 1300);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program