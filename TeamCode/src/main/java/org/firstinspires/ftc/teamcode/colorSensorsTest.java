//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

@TeleOp(name = "Test All Color Sensors") //Name the program
public class colorSensorsTest extends LinearOpMode //CLASS START
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

    //Define Sensors and the CDI
    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;
    ColorSensor colorSensorBottom;
    DeviceInterfaceModule CDI;

    //These are arrays that will hold the hue, saturation, and value information for all three sensors.
    float hsvValuesBottom[] = {0F,0F,0F};
    float hsvValuesLeft[] = {0F,0F,0F};
    float hsvValuesRight[] = {0F,0F,0F};

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

//***************************************************************************************************************************
        //While the op mode is active, loop and read the RGB data from all three sensors.
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            //Convert RGB (red-blue-green) to HSV (hue-saturation-value) and store them into hsvValues
            //This makes hsvValues[0] = hue, hsvValues[1] = saturation and hsvValues[2] = value
            Color.RGBToHSV(colorSensorBottom.red() * 8, colorSensorBottom.green() * 8, colorSensorBottom.blue() * 8, hsvValuesBottom);
            Color.RGBToHSV(colorSensorLeft.red() * 8, colorSensorLeft.green() * 8, colorSensorLeft.blue() * 8, hsvValuesLeft);
            Color.RGBToHSV(colorSensorRight.red() * 8, colorSensorRight.green() * 8, colorSensorRight.blue() * 8, hsvValuesRight);

            //Tell us the data from the bottom sensor
            telemetry.addData("Clear Bottom", colorSensorBottom.alpha());
            telemetry.addData("Red Bottom", colorSensorBottom.red());
            telemetry.addData("Green Bottom", colorSensorBottom.green());
            telemetry.addData("Blue Bottom", colorSensorBottom.blue());
            telemetry.addData("Hue Bottom", hsvValuesBottom[0]);
            telemetry.addData("Saturation Bottom", hsvValuesBottom[1]);
            telemetry.addData("Value Bottom", hsvValuesBottom[2]);

            //Tell us the data from the left sensor
            telemetry.addData("Clear Left", colorSensorLeft.alpha());
            telemetry.addData("Red Left", colorSensorLeft.red());
            telemetry.addData("Green Left", colorSensorLeft.green());
            telemetry.addData("Blue Left", colorSensorLeft.blue());
            telemetry.addData("Hue Left", hsvValuesLeft[0]);
            telemetry.addData("Saturation Left", hsvValuesLeft[1]);
            telemetry.addData("Value Left", hsvValuesLeft[2]);

            //Tell us the data from the right sensor
            telemetry.addData("Clear Right", colorSensorRight.alpha());
            telemetry.addData("Red Right", colorSensorRight.red());
            telemetry.addData("Green Right", colorSensorRight.green());
            telemetry.addData("Blue Right", colorSensorRight.blue());
            telemetry.addData("Hue Right", hsvValuesRight[0]);
            telemetry.addData("Saturation Right", hsvValuesRight[1]);
            telemetry.addData("Value Right", hsvValuesRight[2]);

            //Update the data if/when it changes
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while(opModeIsActive())" loop
    } //Close "run Opmode" loop
} //Close class and end program
