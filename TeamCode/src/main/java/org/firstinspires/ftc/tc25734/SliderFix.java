package org.firstinspires.ftc.tc25734;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.tc25734.MechanumDriveClass;
import org.firstinspires.ftc.tc25734.ClawSystemClass;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Slider Fix", group="Linear Opmode")
public class SliderFix extends LinearOpMode {

    private DcMotor slider;
    
    private static final int SLIDER_HIGH_BAR_POSITION = 720;
    //private static final int SLIDER_MIN = 0; // 1000~=2in
    private static final int SLIDER_MAX = 19000;

    @Override
    public void runOpMode() {
        slider = hardwareMap.get(DcMotor.class, "slider");

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
        
        // ClawSystemClass clawDriver = new ClawSystemClass(hardwareMap, telemetry);
        MechanumDriveClass mechanumDrive = new MechanumDriveClass(hardwareMap, telemetry);
        mechanumDrive.setMode(0);

        int sliderPosition = 0;
        Boolean clawOpen = false;
        Boolean pivotDown = false;
        Boolean button_a_released = true;
        Boolean button_b_released = true;
        Boolean button_x_released = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            mechanumDrive.runByPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            // clawDriver.moveWrist(gamepad2.right_stick_y * 0.75);
            
            double power = -gamepad2.left_stick_y;
            sliderPosition = slider.getCurrentPosition();
           // if ((sliderPosition <= SLIDER_MIN) && power < 0) {
        //        power = 0;
          //  }
            if ((sliderPosition >= SLIDER_MAX) && power > 0) {
                power = 0;
            }
            telemetry.addData("slider position", sliderPosition);
            slider.setPower(power);
            
            // if (gamepad2.a && button_a_released)  {
            //     clawOpen = !clawOpen;
            //     button_a_released = false;
            // } else if (!gamepad2.a) {
            //     button_a_released = true;
            // }
            // clawDriver.setClawOpen(clawOpen);
            
            // if (gamepad2.x && button_x_released) {
            //     sliderTarget = SLIDER_HIGH_BAR_POSITION;
            //     button_x_released = false;
            // } else if (!gamepad2.x) {
            //     button_x_released = true;
            // }
            
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
