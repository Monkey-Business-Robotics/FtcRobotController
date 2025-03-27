// IDENTIFIERS_USED=leftDriveAsDcMotor,rightDriveAsDcMotor

var rightTarget, LeftTarget, Speed, RightPos, LeftPos;

/**
 * Describe this function...
 */
function Drive(rightTarget, LeftTarget, Speed) {
  RightPos = (typeof RightPos == 'number' ? RightPos : 0) + 420 * (rightTarget / (3.5 * Math.PI));
  LeftPos = (typeof LeftPos == 'number' ? LeftPos : 0) + 420 * (LeftTarget / (3.5 * Math.PI));
  rightDriveAsDcMotor.setDualTargetPosition(RightPos, leftDriveAsDcMotor, LeftPos);
  rightDriveAsDcMotor.setDualMode("RUN_TO_POSITION", leftDriveAsDcMotor, "RUN_TO_POSITION");
  rightDriveAsDcMotor.setDualPower(Speed, leftDriveAsDcMotor, Speed);
  while (linearOpMode.opModeIsActive() && leftDriveAsDcMotor.isBusy() && rightDriveAsDcMotor.isBusy()) {
    linearOpMode.idle();
  }
}

/**
 * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
 * Comment Blocks show where to place Initialization code (runs once, after touching the
 * DS INIT button, and before touching the DS
 * Start arrow), Run code (runs once, after
 * touching Start), and Loop code (runs repeatedly
 * while the OpMode is active, namely not
 * Stopped).
 */
function runOpMode() {
  rightDriveAsDcMotor.setDualMode("STOP_AND_RESET_ENCODER", leftDriveAsDcMotor, "STOP_AND_RESET_ENCODER");
  leftDriveAsDcMotor.setDirection("REVERSE");
  RightPos = 0;
  LeftPos = 0;
  linearOpMode.waitForStart();
  Drive(-39, -39, 0.5);
}
