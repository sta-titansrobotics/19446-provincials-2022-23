package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//blah blah blah

@TeleOp
public class oldPowerplay extends LinearOpMode {

    DcMotor leftLift, rightLift;

    boolean pRightBumper = false;
    boolean pLeftBumper = false;

    boolean pGA2Y = false;
    boolean pGA2A = false;

    boolean scissorToggle = false;
    boolean sliderToggle = false;

    public enum LimitState {
        LIMIT_ON,
        LIMIT_OFF,
        LIMIT_RESET,
    }

    LimitState limitState = LimitState.LIMIT_ON;

    @Override
    public void runOpMode() {


        // Movement Motors
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Other
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        // Reverse right lift motor
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // lift motors
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");;

        double scissorPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        ElapsedTime rpTimer = new ElapsedTime();

        waitForStart();

        // set initial positions
        scissorPos = 0.5;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // forward, back
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            // strafing
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing

            // turning
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            // lift

            // joystick up returns negative values, joystick down returns positive values

            double lift_power = -gamepad2.left_stick_y; // flip it
            if (gamepad2.left_stick_y < 0) {

                leftLift.setPower(lift_power * 0.85);
                rightLift.setPower(lift_power* 0.85);


            } else if (gamepad2.left_stick_y > 0) {

                if (leftLift.getCurrentPosition() > 0) {
                    leftLift.setPower(lift_power * 0.30);
                }

                if (rightLift.getCurrentPosition() > 0) {
                    rightLift.setPower(lift_power * 0.30);
                }

            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }

            // scissor intake
            boolean ga2Y = gamepad2.y;
            if (ga2Y && !pGA2Y) {

                if (scissorPos > 0) {
                    scissorPos += 0.1;
                }

            }
            pGA2Y = ga2Y;

            boolean ga2A = gamepad2.a;
            if (ga2A && !pGA2A) {
                if (scissorPos < 1) {
                    scissorPos -= 0.1;
                }

            }
            pGA2A = ga2A;


            switch (limitState) {
                case LIMIT_ON:
                    // limiters
                    if (leftLift.getCurrentPosition() < 0) {
                        leftLift.setPower(0.5);
                        // moveLiftSingle(leftLift, 0.5, 0);
                    }

                    if (rightLift.getCurrentPosition() < 0) {
                        rightLift.setPower(0.5);
                        // moveLiftSingle(rightLift, 0.5, 0);
                    }
                    if (gamepad2.left_bumper) {
                        limitState = LimitState.LIMIT_OFF;
                    }
                    break;
                case LIMIT_OFF:
                    if (gamepad2.left_bumper) {
                        limitState = LimitState.LIMIT_RESET;
                    }
                    break;
                case LIMIT_RESET:

                    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    limitState = LimitState.LIMIT_ON;

                    break;

                default:
                    limitState = LimitState.LIMIT_ON;
                    
            }

            // set positions to servos
            servoScissor.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));

            // add telemetry data
            telemetry.addData("Left Lift Power: ", leftLift.getPower());
            telemetry.addData("Right Lift Power: ", rightLift.getPower());
            telemetry.addData("Left Lift Encoder: ", leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder: ", rightLift.getCurrentPosition());
            telemetry.addData("Scissor Intake Position: ", servoScissor.getPosition());

            telemetry.update();

        }

    }



}