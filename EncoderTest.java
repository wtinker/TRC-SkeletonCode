package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Encoder Test", group = "SkeletonCode") //Establishes name and group
//@Disabled //Stops this from appearing, just comment it out to include
public class EncoderTest extends OpMode{

    DcMotor M1;
    DcMotor M2;

    int pos1, pos2;

    int change = 50;

    double yStartTime = 0;
    double aStartTime = 0;
    double upStartTime = 0;
    double downStartTime = 0;
    //Values for button cooldowns

    public void init() {

        M1 = hardwareMap.dcMotor.get("Motor 1");
        M2 = hardwareMap.dcMotor.get("Motor 2");

        M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void loop() {

        if(gamepad1.y && yCooldown()) {
            pos1 = M1.getCurrentPosition();
            pos1 = (int) (pos1 + change);
            M1.setTargetPosition(pos1);
            M1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(gamepad1.a && aCooldown()) {
            pos1 = M1.getCurrentPosition();
            pos1 = (int) (pos1 - change);
            M1.setTargetPosition(pos1);
            M1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(gamepad1.dpad_up && upCooldown()) {
            pos2 = M2.getCurrentPosition();
            pos2 = (int) (pos2 + change);
            M2.setTargetPosition(pos2);
            M2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(gamepad1.dpad_down && downCooldown()) {
            pos2 = M2.getCurrentPosition();
            pos2 = (int) (pos2 - change);
            M2.setTargetPosition(pos2);
            M2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Motor 1:", pos1);
        telemetry.addData("Motor 2:", pos2);
        telemetry.update();
    }

    //Input Cooldowns
    public boolean yCooldown() {
        if(getRuntime() - yStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            yStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean aCooldown() {
        if(getRuntime() - aStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            aStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean upCooldown() {
        if(getRuntime() - upStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            upStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean downCooldown() {
        if(getRuntime() - downStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            downStartTime = getRuntime();
            return true;
        }
        return false;
    }
}
