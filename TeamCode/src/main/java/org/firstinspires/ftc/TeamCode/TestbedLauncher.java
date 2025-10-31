package org.firstinspires.ftc.teamcode; // make sure this matches your folder path

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Testbed Launcher", group = "Linear Opmode")
@Disabled
public class TestbedLauncher extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor"); // matches your config name
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // explicit & safe

        telemetry.addLine("Initialized - ready for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Run at full power when match starts
        motor.setPower(1);
        telemetry.addLine("Motor running");
        telemetry.update();

        // Keep reporting while opmode is active
        while (opModeIsActive()) {
            telemetry.addData("Motor Power", "%.2f", motor.getPower());
            telemetry.update();
            idle(); // lets other threads run
        }

        // Safety: stop the motor when opmode ends
        motor.setPower(0.0);
    }
}
