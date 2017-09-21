//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleFLOP") //Name the class
public class TELE2 extends LinearOpMode
{
    //Define Servo Motors
    DcMotor leftGlyphGrabber;

    private ElapsedTime runtime = new ElapsedTime();

    int yPressed = 0;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftGlyphGrabber = hardwareMap.dcMotor.get("leftGlyphGrabber");

        //Wait for start button to be clicked
        waitForStart();
        runtime.reset();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            //Set float variables as the inputs from the joysticks and the triggers

            if(gamepad1.y) {
                yPressed++;
            }

            if (yPressed %2 ==1) {
                leftGlyphGrabber.setPower(0.2);
                Thread.sleep(800);
                leftGlyphGrabber.setPower(0.05);
            }
            else if (yPressed %2 ==0 && yPressed != 0){
                leftGlyphGrabber.setPower(-0.2);
                Thread.sleep(500);
                leftGlyphGrabber.setPower(0.0);
            }

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program