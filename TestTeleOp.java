package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by yasmeen on 12/3/15.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class TestTeleOp extends OpMode {

    final static double TAPE_MIN_RANGE  = 0.20;
    final static double TAPE_MAX_RANGE  = 0.90;

    final static double CLAMP_MIN_RANGE = 0.30;
    final static double CLAMP_MAX_RANGE = 0.80;

    // position of the tape measure servo.
    double tapePosition;

    // amount to change the arm servo position.
    double tapeDelta = 0.1;

    // position of the left hook servo
    double leftClampPosition;

    // amount to change the left hook servo position by
    double leftClampDelta = 0.1;

    // position of the right hook servo
    double rightClampPosition;

    // amount to change the right hook servo position by
    double rightClampDelta = 0.1;

    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorClimbRight;
    DcMotor motorClimbLeft;
    Servo servoTape;
    Servo servoLeftClamp;
    Servo servoRightClamp;

    //DcMotor lift1;
    //DcMotor lift2;

    /**
     * Constructor
     */
    public TestTeleOp() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
		/*
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot.
		 *
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
        motorRightFront = hardwareMap.dcMotor.get("frontRight");
        motorRightBack = hardwareMap.dcMotor.get("backRight");
        motorLeftFront = hardwareMap.dcMotor.get("frontLeft");
        motorLeftBack = hardwareMap.dcMotor.get("backLeft");
        motorClimbRight = hardwareMap.dcMotor.get("climbRight");
        motorClimbLeft = hardwareMap.dcMotor.get("climbRight");

        // motorLeft.setDirection(DcMotor.Direction.REVERSE);

        servoTape = hardwareMap.servo.get("tm");
        servoRightClamp = hardwareMap.servo.get("rightClamp");
        servoLeftClamp = hardwareMap.servo.get("leftClamp");

        // assign the starting position of the wrist and claw
        // tapePosition = 0.2;
        // rightClampPosition = 0.2;
        // leftClampPosition = 0.2;

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

		/*
		 * Gamepad 1 = Driver
		 *  - clamps(bumper & trigger), driving
		 *
		 * Gamepad 2 = Operator
		 *  - tm, climb motors
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // write the values to the motors
        motorRight.setPower(right);
        motorLeft.setPower(left);

        // update the position of the left bumper.
        if (gamepad1.left_bumper) {
            // if the left_bumper button is pushed on gamepad1, increment the position of
            // the left clamp.
            //armPosition += armDelta;
            leftClamp += leftClampDelta;
        }

        if (gamepad1.left_trigger) {
            // if the left_trigger button is pushed on gamepad1, decrease the position of
            // the left clamp.
            //armPosition -= armDelta;
            leftClamp -= leftClampDelta;
        }

        // update the position of the right clamp
        if (gamepad1.right_bumper) {
            //clawPosition += clawDelta;
            rightClamp += rightClampDelta;
        }

        // update the position of the right clamp
        if (gamepad1.right_bumper) {
            //clawPosition += clawDelta;
            rightClamp -= rightClampDelta;
        }

/*
        if (gamepad1.left_trigger > 0.25) {
            clawPosition -= clawDelta;
        }

        if (gamepad1.b) {
            clawPosition -= clawDelta;
        }

        // update the position of the claw
        if (gamepad1.x) {
            clawPosition += clawDelta;
        }

        if (gamepad1.b) {
            clawPosition -= clawDelta;
        }
*/
        // clip the position values so that they never exceed their allowed range.
        //armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
        leftClamp = Range.clip(leftClampPosition, CLAMP_MIN_RANGE, CLAMP_MAX_RANGE);
        rightClamp = Range.clip(rightClampPosition, CLAMP_MIN_RANGE, CLAMP_MAX_RANGE);

        // write position values to the wrist and claw servo
        //arm.setPosition(armPosition);
        //claw.setPosition(clawPosition);
        leftClamp.setPosition(leftClampPosition);
        rightClamp.setPosition(rightClampPosition);
		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

        telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("leftClamp", "left clamp:  " + String.format("%.2f", leftClampPosition));
        telemetry.addData("rightClamp", "right clamp:  " + String.format("%.2f", rightClampPosition));
        //telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        //telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
