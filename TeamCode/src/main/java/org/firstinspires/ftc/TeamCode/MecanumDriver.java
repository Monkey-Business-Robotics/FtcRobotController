package org.firstinspires.ftc.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDriver {

    private final Telemetry telemetry;
    private final LinearOpMode opmode;

    private final DcMotor fl_motor;
    private final DcMotor bl_motor;
    private final DcMotor fr_motor;
    private final DcMotor br_motor;

    private static final int WHEEL_BASE_MM = 520;
    private static final int TICKS_PER_REV = 28 * 20; // Motor * Ratio
    private static final double WHEEL_DIAMETER = 75;  // Wheel Diameter
    private static final double TICKS_PER_MM = (TICKS_PER_REV) / (Math.PI * WHEEL_DIAMETER);

    public MecanumDriver(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode) {
        this.telemetry = telemetry;
        this.opmode = opmode;

        // Initialize motors
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");

        // Set motor directions
        fl_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        bl_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        fr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            stop_motors();
            fl_motor.setTargetPosition(0);
            bl_motor.setTargetPosition(0);
            fr_motor.setTargetPosition(0);
            br_motor.setTargetPosition(0);

            fl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void moveY(double d, double p) {
        // d = distance, p = power/speed
        d = -d;
        int fl_target = fl_motor.getCurrentPosition() + (int) (d * TICKS_PER_MM);
        int bl_target = bl_motor.getCurrentPosition() + (int) (d * TICKS_PER_MM);
        int fr_target = fr_motor.getCurrentPosition() + (int) (d * TICKS_PER_MM);
        int br_target = br_motor.getCurrentPosition() + (int) (d * TICKS_PER_MM);

        fl_motor.setTargetPosition(fl_target);
        bl_motor.setTargetPosition(bl_target);
        fr_motor.setTargetPosition(fr_target);
        br_motor.setTargetPosition(br_target);
        fl_motor.setPower(p);
        bl_motor.setPower(p);
        fr_motor.setPower(p);
        br_motor.setPower(p);

        while (opmode.opModeIsActive() && (fl_motor.isBusy() || fr_motor.isBusy() || bl_motor.isBusy() || br_motor.isBusy())) {
            telemetry.addData("FL: ", fl_motor.getCurrentPosition());
            telemetry.addData("FR: ", fr_motor.getCurrentPosition());
            telemetry.addData("BL: ", bl_motor.getCurrentPosition());
            telemetry.addData("BR: ", br_motor.getCurrentPosition());
            telemetry.update();
        }

        stop_motors();
    }

    public void moveX(double d, double p) {
        d = -d;
        int fl_target = fl_motor.getCurrentPosition() + (int) (d * TICKS_PER_MM);
        int bl_target = bl_motor.getCurrentPosition() - (int) (d * TICKS_PER_MM);
        int fr_target = fr_motor.getCurrentPosition() - (int) (d * TICKS_PER_MM);
        int br_target = br_motor.getCurrentPosition() + (int) (d * TICKS_PER_MM);

        fl_motor.setTargetPosition(fl_target);
        bl_motor.setTargetPosition(bl_target);
        fr_motor.setTargetPosition(fr_target);
        br_motor.setTargetPosition(br_target);
        fl_motor.setPower(p);
        bl_motor.setPower(p);
        fr_motor.setPower(p);
        br_motor.setPower(p);

        while (opmode.opModeIsActive() && (fl_motor.isBusy() || fr_motor.isBusy() || bl_motor.isBusy() || br_motor.isBusy())) {
            telemetry.addData("FL: ", fl_motor.getCurrentPosition());
            telemetry.addData("FR: ", fr_motor.getCurrentPosition());
            telemetry.addData("BL: ", bl_motor.getCurrentPosition());
            telemetry.addData("BR: ", br_motor.getCurrentPosition());
            telemetry.update();
        }

        stop_motors();
    }

    public void rotate(double a, double p) {
        double d = (a / 360) * WHEEL_BASE_MM;
        d = -d;

        int fl_target = fl_motor.getCurrentPosition() + (int) (d * TICKS_PER_MM);
        int bl_target = bl_motor.getCurrentPosition() - (int) (d * TICKS_PER_MM);
        int fr_target = fr_motor.getCurrentPosition() + (int) (d * TICKS_PER_MM);
        int br_target = br_motor.getCurrentPosition() - (int) (d * TICKS_PER_MM);

        fl_motor.setTargetPosition(fl_target);
        bl_motor.setTargetPosition(bl_target);
        fr_motor.setTargetPosition(fr_target);
        br_motor.setTargetPosition(br_target);
        fl_motor.setPower(p);
        bl_motor.setPower(p);
        fr_motor.setPower(p);
        br_motor.setPower(p);

        while (opmode.opModeIsActive() && (fl_motor.isBusy() || fr_motor.isBusy() || bl_motor.isBusy() || br_motor.isBusy())) {
            telemetry.addData("FL: ", fl_motor.getCurrentPosition());
            telemetry.addData("FR: ", fr_motor.getCurrentPosition());
            telemetry.addData("BL: ", bl_motor.getCurrentPosition());
            telemetry.addData("BR: ", br_motor.getCurrentPosition());
            telemetry.update();
        }

        stop_motors();
    }

    public void stop_motors() {
        fl_motor.setPower(0);
        fr_motor.setPower(0);
        bl_motor.setPower(0);
        br_motor.setPower(0);
    }

    public void runByPower(double x, double y, double r, double powerScale) {
        // Reverse y for joystick direction
        //y = -y;
        x = -x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        double frontLeftPower = (y + x + r) / denominator;
        double backLeftPower = (y - x + r) / denominator;
        double frontRightPower = (y - x - r) / denominator;
        double backRightPower = (y + x - r) / denominator;

        // Scale down power and up power

        frontLeftPower *= powerScale;
        backLeftPower *= powerScale;
        frontRightPower *= powerScale;
        backRightPower *= powerScale;

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