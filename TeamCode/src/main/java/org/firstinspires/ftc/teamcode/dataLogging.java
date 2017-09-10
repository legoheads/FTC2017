//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@TeleOp(name = "Data Logging Program") //Name the program
public class dataLogging extends LinearOpMode
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
        //While the op mode is active, loop and read the RGB data.
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            if (gamepad1.b)
            {
                //If "b" is pressed, reset the encoders
                leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //Use the encoders
                leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //Show all the encoder values on the driver station
            telemetry.addData("left front", leftMotorFront.getCurrentPosition());
            telemetry.addData("left back", leftMotorBack.getCurrentPosition());
            telemetry.addData("right front", rightMotorFront.getCurrentPosition());
            telemetry.addData("right back", rightMotorBack.getCurrentPosition());

            //Update the data if/when it changes
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while(opModeIsActive())" loop
    } //Close "run Opmode" loop
} //Close class and end program