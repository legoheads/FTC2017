//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    Servo relicSpool;
    Servo relicFlip;

    //Jewel Motor
    Servo jewelArm;

    //Define Sensors and the CDI
    ColorSensor colorSensor;

    //Define floats to be used as joystick inputs, trigger inputs, and constants
    float drivePowerFast;
    float shiftPowerFast;
    float drivePowerSlow;
    float shiftPowerSlow;
    float fastLeftTurnPower;
    float fastRightTurnPower;
    float slowLeftTurnPower;
    float slowRightTurnPower;
    float liftPower;

    private ElapsedTime runtime = new ElapsedTime();

    private int rightBumperPress = 0;

    int relicFlipToggle = -1;
    int relicDropToggle = -1;

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
        relicSpool = hardwareMap.servo.get("relicSpool");
        relicFlip = hardwareMap.servo.get("relicFlip");

        //Get references to the Servo Motors from the hardware map
        jewelArm = hardwareMap.servo.get("jewelArm");
        //Get references to the sensors and the CDI from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors, CDI)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphGrab, glyphLift, relicGrab, relicSpool, relicFlip, jewelArm, colorSensor);


        //Set the sensors to the modes that we want, and set their addresses. Also set the directions of the motors
        //Reverse some motors and keep others forward
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
            drivePowerFast = gamepad1.left_stick_y * (float) 0.8;
            shiftPowerFast = gamepad1.left_stick_x;
            fastLeftTurnPower = (float) (gamepad1.left_trigger / 2);
            fastRightTurnPower = (float) (gamepad1.right_trigger / 2);
            jewelArm.setPosition(0.1);
            liftPower = gamepad1.right_stick_y;



            //Do nothing if joystick is stationary

            //Shift if pushed more on X than Y
            if (Math.abs(shiftPowerFast) > Math.abs(drivePowerFast)) {
                functions.shiftTeleop(shiftPowerFast);
            }

            //Drive if joystick pushed more Y than X
            if (Math.abs(drivePowerFast) > Math.abs(shiftPowerFast)) {
                functions.driveTeleop(drivePowerFast);
            }

            if (drivePowerFast == 0 && shiftPowerFast == 0 && drivePowerSlow == 0 && shiftPowerSlow == 0) {
                functions.setDriveMotorPowers(0, 0, 0, 0);
            }

            if (Math.abs(shiftPowerSlow) > Math.abs(drivePowerSlow))
            {
                functions.shiftTeleop(shiftPowerSlow);
            }

            //Drive if joystick pushed more Y than X
            if (Math.abs(drivePowerSlow) > Math.abs(shiftPowerSlow))
            {
                functions.driveTeleop(drivePowerSlow);
            }

            //If the left trigger is pushed, turn left at that power
            if (fastLeftTurnPower > 0)
            {
                functions.leftTurnTeleop(fastLeftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (fastRightTurnPower > 0)
            {
                functions.rightTurnTeleop(fastRightTurnPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (slowLeftTurnPower > 0)
            {
                functions.leftTurnTeleop(slowLeftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (slowRightTurnPower > 0)
            {
                functions.rightTurnTeleop(slowRightTurnPower);
            }


            if ((gamepad1.right_bumper)) {
                //Increase the increment operator
                rightBumperPress++;

                //If the "y" button is pressed, grab/drop a glyph
                if (rightBumperPress % 2 == 0) {
                    functions.glyphDoor("open");
                }
                if (rightBumperPress % 2 == 1) {
                    functions.glyphDoor("close");
                }
            }

            if (Math.abs(liftPower)>=0.1) {
                glyphLift.setPower(liftPower);
            }
            if (Math.abs(liftPower) < 0.1) {
                glyphLift.setPower(0.0);
            }

            if ((gamepad1.b) || (gamepad2.b)) {
                functions.stop();
            }

            if (gamepad1.dpad_up) {
                glyphLift.setPower(0.0);
                functions.oneMotorEncoder(650, (float) 1.0, glyphLift);
            }
            if (gamepad1.dpad_down) {
                glyphLift.setPower(0.0);
                functions.oneMotorEncoder(-650, (float) -1.0, glyphLift);
            }
            if (gamepad2.x){
                relicDropToggle++;
            }
            if (relicDropToggle %2 == 0){
                relicGrab.setPosition(0);
            }
            if (relicDropToggle %2 == 1){
                relicGrab.setPosition(1.0);
            }

            if (gamepad2.y){
                relicFlipToggle++;
            }
            if (relicFlipToggle % 2 == 0){
                relicFlip.setPosition(1.0);
            }
            if (relicFlipToggle % 2 == 1){
                relicFlip.setPosition(0);
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