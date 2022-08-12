package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "SkeletonCode")
//@Disabled
public class ServoTest extends OpMode{

    Servo S1;
    Servo S2;

    double pos1 = 0.01;
    double pos2 = 0.01;

    double change = 0.05;

    double yStartTime = 0;
    double aStartTime = 0;
    double upStartTime = 0;
    double downStartTime = 0;
    //Values for button cooldowns

    public void init() {

        S1 = hardwareMap.servo.get("Servo 1");
        S2 = hardwareMap.servo.get("Servo 2");

    }

    public void loop() {

        if(gamepad1.y && yCooldown()) {
            pos1 = S1.getPosition();
            pos1 = (pos1 + change);
            S1.setPosition(pos1);
        }

        if(gamepad1.a && aCooldown()) {
            pos1 = S1.getPosition();
            pos1 = (pos1 - change);
            S1.setPosition(pos1);
        }

        if(gamepad1.dpad_up && upCooldown()) {
            pos2 = S2.getPosition();
            pos2 = (pos2 + change);
            S2.setPosition(pos2);
        }

        if(gamepad1.dpad_down && downCooldown()) {
            pos2 = S2.getPosition();
            pos2 = (pos2 - change);
            S2.setPosition(pos2);
        }
        telemetry.addData("Servo 1:", pos1);
        telemetry.addData("Servo 2:", pos2);
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
