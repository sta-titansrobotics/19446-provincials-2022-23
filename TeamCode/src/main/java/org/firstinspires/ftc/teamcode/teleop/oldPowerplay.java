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
    boolean pGA2X = false;
    boolean pGA2A = false;
    boolean pGA2B = false;

    boolean scissorToggle = false;
    boolean sliderToggle = false;

    public enum RpState {
        rpSTART,
        rpScissor,
        rpUP,
    }

    RpState rpState = RpState.rpSTART;

    @Override
    public void runOpMode() {


        // Movement Motors
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

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

        Servo servoScissor = hardwareMap.get(Servo.class, "servoScissor");
        Servo verticalServo = hardwareMap.get(Servo.class, "servoScissorLift");

        TouchSensor liftSensorRight = hardwareMap.get(TouchSensor.class, "liftSensorRight");
        TouchSensor liftSensorLeft = hardwareMap.get(TouchSensor.class, "liftSensorLeft");

        double verticalServoPos, scissorPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        ElapsedTime rpTimer = new ElapsedTime();

        waitForStart();

        // set initial positions
        verticalServoPos = 0.5;
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
            if (gamepad2.left_stick_y < 0) {

                leftLift.setPower(-gamepad2.left_stick_y * 0.85);
                rightLift.setPower(-gamepad2.left_stick_y * 0.85);

                if (verticalServoPos > MIN_POSITION) {
                    verticalServoPos += 0.01;
                }

            } else if (gamepad2.left_stick_y > 0) {

                if (leftLift.getCurrentPosition() > 0) {
                    leftLift.setPower(-gamepad2.left_stick_y * 0.30);
                }

                if (rightLift.getCurrentPosition() > 0) {
                    rightLift.setPower(-gamepad2.left_stick_y * 0.30);
                }


                if (verticalServoPos < MAX_POSITION) {
                    verticalServoPos -= 0.01;
                }
            } else {

                if (leftLift.getCurrentPosition() > 750 && !gamepad2.x && !leftLift.isBusy()) {

                    leftLift.setPower(0.1);
                    rightLift.setPower(0.1);
                }

                else if (!gamepad2.x && !leftLift.isBusy()){

                    leftLift.setPower(0);
                    rightLift.setPower(0);
                }

            }


            if (leftLift.getCurrentPosition() < 0) {
                leftLift.setPower(0.5);
                // moveLiftSingle(leftLift, 0.5, 0);
            }


            if (rightLift.getCurrentPosition() < 0) {
                rightLift.setPower(0.5);
                // moveLiftSingle(rightLift, 0.5, 0);
            }

            // scissor rp fsm
            switch (rpState) {
                case rpSTART:
                    boolean ga2B = gamepad2.b;
                    if (ga2B && !pGA2B) {
                        verticalServoPos = 0;
                        rpTimer.reset();
                        rpState = RpState.rpScissor;
                    }
                    pGA2B = ga2B;
                    break;
                case rpScissor:
                    if (rpTimer.seconds() >= 1.17) {
                        scissorPos = 0.67;
                        rpTimer.reset();
                        rpState = RpState.rpUP;
                    }
                    break;
                case rpUP:
                    if (rpTimer.seconds() >= 0.50) {
                        verticalServoPos = 1;
                        rpState = RpState.rpSTART;
                    }
                    break;
                default:
                    // SHOULD never reach this
                    rpState = RpState.rpSTART;


            }
            // vertical slider
            boolean rightBumper = gamepad2.right_bumper;
            if (rightBumper && !pRightBumper && verticalServoPos < MAX_POSITION) {
                verticalServoPos += 0.25;
            }
            pRightBumper = rightBumper;

            boolean leftBumper = gamepad2.left_bumper;
            if (leftBumper && !pLeftBumper && verticalServoPos > MIN_POSITION) {
                verticalServoPos -= 0.25;
            }
            pLeftBumper = leftBumper;

            // scissor intake
            boolean ga2Y = gamepad2.y;
            if (ga2Y && !pGA2Y) {

                if (scissorPos == 0.67) {
                    scissorPos = 0.48;
                } else {
                    scissorPos = 0.67;
                }
            }
            pGA2Y = ga2Y;

            // lift presets
            boolean ga2X = gamepad2.x;
            if (ga2X && !pGA2X) {
                moveLift(0.5, 0);
            }
            pGA2X = ga2X;

            boolean ga2A = gamepad2.a;
            if (ga2A && !pGA2A) {
                moveLift(0.8, 3150);
            }
            pGA2A = ga2A;



            // set positions to servos
            verticalServo.setPosition(Range.clip(verticalServoPos, MIN_POSITION, MAX_POSITION));
            servoScissor.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));

            // add telemetry data
            telemetry.addData("Left Lift Power: ", leftLift.getPower());
            telemetry.addData("Right Lift Power: ", rightLift.getPower());
            telemetry.addData("Left Lift Encoder: ", leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder: ", rightLift.getCurrentPosition());
            telemetry.addData("Vertical Slider Position: ", verticalServo.getPosition());
            telemetry.addData("Scissor Intake Position: ", servoScissor.getPosition());
            telemetry.addData("Left Touch Sensor: ", liftSensorLeft.isPressed());
            telemetry.addData("Right Touch Sensor: ", liftSensorRight.isPressed());

            telemetry.update();

        }

    }

    /**
     * Powers lift to target position
     * @param power desired power
     * @param ticks target position
     */
    public void moveLift(double power, int ticks) {
        leftLift.setTargetPosition(ticks);
        rightLift.setTargetPosition(ticks);

        setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorPower(power);

        while(leftLift.isBusy()) {

        }

        motorPower(0);

        setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Powers single lift motor to target position
     * @param power desired power
     * @param ticks target position
     */
    public void moveLiftSingle(DcMotor motor, double power, int ticks) {
        motor.setTargetPosition(ticks);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(power);

        while(motor.isBusy()) {

        }

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set power of both lift motors
     * @param power setPower
     */
    public void motorPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    /**
     * Change mode of cascading lift
     * @param mode setMode
     */
    public void setLiftMode(DcMotor.RunMode mode) {
        leftLift.setMode(mode);
        rightLift.setMode(mode);
    }




}