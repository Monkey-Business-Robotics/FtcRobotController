package org.firstinspires.ftc.tc25734;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MechanumDriveClass {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final DcMotor fl_motor;
    private final DcMotor bl_motor;
    private final DcMotor fr_motor;
    private final DcMotor br_motor;

    public MechanumDriveClass(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Initialize motors
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");

        // Set motor directions
        fl_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        bl_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        fr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMode(int mode) {
        // Stop & reset all encoders
        fl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (mode == 0) {
            fl_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bl_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fr_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            br_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (mode == 1) {
            fl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void runByPower(double x, double y, double r) {
        // Reverse y for joystick direction
        //y = -y;
        x = -x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), .75);
        double frontLeftPower = (y + x + r) / denominator;
        double backLeftPower = (y - x + r) / denominator;
        double frontRightPower = (y - x - r) / denominator;
        double backRightPower = (y + x - r) / denominator;
        
        // Scale down power by 0.75 (reduce power by 25%)
        double powerscale = 0.75;
        frontLeftPower *= powerscale;
        backLeftPower *= powerscale;
        frontRightPower *= powerscale;
        backRightPower *= powerscale;
        
        // Set motor powers
        fl_motor.setPower(frontLeftPower);
        bl_motor.setPower(backLeftPower);
        fr_motor.setPower(frontRightPower);
        br_motor.setPower(backRightPower);

        // Add telemetry data
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
    }
}