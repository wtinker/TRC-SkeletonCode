package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//All of the imports we need

@TeleOp(name = "TeleopSkeletonCooldown", group = "SkeletonCode") //Establishes name and group
@Disabled //Stops this code from appearing, just comment it out to include
public class TeleopSkeletonCooldown extends LinearOpMode {

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    //Declares our drivetrain motors, make sure to add any extra motors used

    double power = 0.5; //Power Coefficient determines power to the wheel motors of robot
    boolean turboMode = false; //Turbomode puts the robot at 100% power in the wheel motors

    double leftTriggerStartTime = 0;
    double leftBumperStartTime = 0;
    double rightBumperStartTime = 0;
    double rightTriggerStartTime = 0;
    double bStartTime = 0;
    double yStartTime = 0;
    double aStartTime = 0;
    double xStartTime = 0;
    double upStartTime = 0;
    double downStartTime = 0;
    double leftStartTime = 0;
    double rightStartTime = 0;
    //Values for button cooldowns

    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        //Declares Hardwaremap stuff, make sure to add for extra motors

        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //Correcting the direction of drivetrain motors, if auto doesnt move right check this first

        waitForStart();

        while (opModeIsActive()) {
            //Put any Telemetry you want in here
            defaultMode();
        }
    }

    public void defaultMode() {
        //Switches full speed on and off
        if (gamepad1.right_bumper && rightBumperCooldown()) {
            turboMode = !turboMode;
        }

        //Tank Drive Movement
        if (turboMode) {
            FrontLeft.setPower(gamepad1.left_stick_y);
            FrontRight.setPower(gamepad1.right_stick_y);
            BackLeft.setPower(gamepad1.left_stick_y);
            BackRight.setPower(gamepad1.right_stick_y);
        } else {
            FrontLeft.setPower(gamepad1.left_stick_y * power);
            FrontRight.setPower(gamepad1.right_stick_y * power);
            BackLeft.setPower(gamepad1.left_stick_y * power);
            BackRight.setPower(gamepad1.right_stick_y * power);
        }

        //Other Controller Inputs
        if (gamepad1.left_bumper && leftBumperCooldown()) {

        }

        if (gamepad1.left_trigger > 0.5 && leftTriggerCooldown()) {

        }

        if (gamepad1.right_trigger > 0.5 && rightTriggerCooldown()) {

        }

        if (gamepad1.b && bCooldown()) {

        }

        if (gamepad1.y && yCooldown()) {

        }

        if (gamepad1.a && aCooldown()) {

        }

        if(gamepad1.x && xCooldown()) {

        }

        if(gamepad1.dpad_up && upCooldown()) {

        }

        if(gamepad1.dpad_down && downCooldown()) {

        }

        if(gamepad1.dpad_left && leftCooldown()) {

        }

        if(gamepad1.dpad_right && rightCooldown()) {

        }

    }

    //Input Cooldowns
    public boolean leftTriggerCooldown() {
        if(getRuntime() - leftTriggerStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            leftTriggerStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean leftBumperCooldown() {
        if(getRuntime() - leftBumperStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            leftBumperStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightTriggerCooldown() {
        if(getRuntime() - rightTriggerStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            rightTriggerStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightBumperCooldown() {
        if(getRuntime() - rightBumperStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            rightBumperStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean xCooldown() {
        if(getRuntime() - xStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            xStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean bCooldown() {
        if(getRuntime() - bStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            bStartTime = getRuntime();
            return true;
        }
        return false;
    }

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

    public boolean leftCooldown() {
        if(getRuntime() - leftStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            leftStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightCooldown() {
        if(getRuntime() - rightStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            rightStartTime = getRuntime();
            return true;
        }
        return false;
    }

}
