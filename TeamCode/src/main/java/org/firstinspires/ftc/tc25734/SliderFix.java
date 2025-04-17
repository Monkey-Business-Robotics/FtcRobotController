package org.firstinspires.ftc.tc25734;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Slider Fix", group="Linear Opmode")
public class SliderFix extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor slider = hardwareMap.get(DcMotor.class, "slider");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        slider.setTargetPosition(0);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotor.Direction.FORWARD);

        int sliderPosition;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double power = -gamepad2.left_stick_y;
            sliderPosition = slider.getCurrentPosition();
            telemetry.addData("slider position", sliderPosition);
            slider.setPower(power);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
