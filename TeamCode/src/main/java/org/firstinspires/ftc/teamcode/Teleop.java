//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp") //Name the class
public class Teleop extends LinearOpMode {
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

    //Define floats to be used as joystick and trigger inputs
    float drive;
    float shift;
    float rightTurn;
    float leftTurn;

    private int yPress = 0;

    private ElapsedTime runtime = new ElapsedTime();

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        runtime.reset();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
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

            if (gamepad1.y) {
                yPress++;
            }

            //If the "y" button is pressed, grab/drop a glyph
            if (yPress %2 == 0 && yPress>=0) {
                functions.glyphDoor("close");
            }
            else if (yPress %2 == 1 && yPress>=0) {
                functions.glyphDoor("open");
            }

            //Stop driving when any "b" button is pressed
            if ((gamepad1.b) || (gamepad2.b)) {
                functions.stopDriving();
            }

            //Count time
            //Update the data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program