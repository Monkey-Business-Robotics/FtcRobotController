package org.firstinspires.ftc.tc25734;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ClawSystemClass {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final Servo clawServoA;
    private final Servo clawServoB;
   // private final DcMotor wrist;
    
    private final static double CLAW_CLOSED_POSITION_A = 0.5;
    private final static double CLAW_OPEN_POSITION_A = 1.0;
    private final static double CLAW_CLOSED_POSITION_B = 0.8;
    private final static double CLAW_OPEN_POSITION_B = 0.5;
    //private final static double WRIST_MIN = 0;
    //private final static double WRIST_MAX = 90;
    //private final static double WRIST_MID = 45;
    
    private double clawPositionA = 0.5;
    private double clawPositionB = 0;
    //private double wristPosition = 0;
    //private PIDFController wristPIDF = new PIDFController(0,0,0,0);

    public ClawSystemClass(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        
        clawServoA = hardwareMap.get(Servo.class, "claw_a");
        clawServoB = hardwareMap.get(Servo.class, "claw_b");
        //wrist = hardwareMap.get(DcMotor.class, "wrist_a");
        
        //wrist.setPower(0);
        //wrist.setTargetPosition(0);
        //wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }
    
    public void setClawOpen(Boolean open) {
        if (open) {
            clawPositionA = CLAW_OPEN_POSITION_A;
            clawPositionB = CLAW_OPEN_POSITION_B;
        } else {
            clawPositionA = CLAW_CLOSED_POSITION_A;
            clawPositionB = CLAW_CLOSED_POSITION_B;
        }
        
        telemetry.addData("Claw A:", clawPositionA);
        telemetry.addData("Claw B:", clawPositionB);
        clawServoA.setPosition(clawPositionA);
        clawServoB.setPosition(clawPositionB);
    }

    //public void moveWrist(double power) {
        // if ((wristPosition <= WRIST_MIN) && power < 0) {
        //     power = 0;
        // }
       // if ((wristPosition >= WRIST_MAX) && power > 0) {
        //    power = 0;
       // }
       // telemetry.addData("wrist position", wristPosition);
       // wrist.setPower(power);
    
 }   

    

