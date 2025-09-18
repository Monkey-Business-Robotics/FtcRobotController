package org.firstinspires.ftc.tc25734;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Slider Fix", group="Linear Opmode")
public class SliderFix extends LinearOpMode {
    private DcMotor slider;
    private DcMotor wrist_a;



    @Override
    public void runOpMode() {
        slider = hardwareMap.get(DcMotor.class, "slider");
        wrist_a = hardwareMap.get(DcMotor.class, "wrist_a");

        slider.setPower(0);
        wrist_a.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        slider.setPower(-1.0);
        wrist_a.setPower(1.0);

        while (opModeIsActive()) {





            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}