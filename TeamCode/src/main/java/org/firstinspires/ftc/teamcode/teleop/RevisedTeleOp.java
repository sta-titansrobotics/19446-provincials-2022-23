package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class RevisedTeleOp extends LinearOpMode{

    // Movement Motors
    DcMotor motorFL, motorBL, motorFR, motorBR, leftLift, rightLift, Arm;

    Servo ScissorServo;

    boolean pGA2A = false;

    double Plateau = 600;

    double slowSpeed = 0.7;
    double strafeCounteract = 1.1;

    double scissorOpen = 0.67;
    double scissorClosed = 0.5;

    boolean scissorToggle = false;

    double armReturn;
    double armStartPos;
    boolean armModeToggle = false;
    boolean armMoveUp = false;

    @Override
    public void runOpMode() {

        // Initialize Wheel Motors
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Initialize Lift Motors
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        // lift motors properties
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Arm Motor
        DcMotor Arm = hardwareMap.get(DcMotor.class, "arm");

        // arm motor properties
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Scissor Servo
        Servo ScissorServo = hardwareMap.get(Servo.class, "scissor");

        // scissor servo properties
        double scissorPos;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        waitForStart();

        scissorPos = scissorClosed;

        if (isStopRequested()) return;
        while (opModeIsActive()) {

            /* DRIVETRAIN */
            // forward, back
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            // strafing
            double x = gamepad1.left_stick_x * strafeCounteract; // Counteract imperfect strafing

            // turning
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            /* LIFT */
            // Apply power to Lift
            Lift(liftPowerLeft(), liftPowerRight());


            /* ARM */
            // Apply power to Arm
            Arm.setPower(armPower());

            /* SCISSOR */
            // scissor intake
            boolean ga2A = gamepad2.a;

            // toggle scissor
            if (ga2A && !gamepad2.a) {
                scissorToggle = !scissorToggle;
            }

            // pick up (expand scissor)
            if (scissorToggle) {
                scissorPos = scissorOpen;
            }

            // release cone (neutral position)
            else {
                scissorPos = scissorClosed;
            }

            // untoggle scissor
            pGA2A = ga2A;

            // Apply power to Scissor
            ScissorServo.setPosition(Range.clip(scissorPos, MIN_POSITION, MAX_POSITION));

            /* TELEMETRY OUTPUT */
            // motor data
            telemetry.addLine("Wheel Data");
            telemetry.addData("Front Left Motor Busy: ", motorFL.isBusy() + " " + frontLeftPower);
            telemetry.addData("Front Right Motor Busy: ", motorFR.isBusy() + " " + frontRightPower);
            telemetry.addData("Front Left Motor Busy: ", motorBL.isBusy() + " " + backLeftPower);
            telemetry.addData("Front Right Motor Busy: ", motorBR.isBusy() + " " + backRightPower);

            // scissor data
            telemetry.addLine("");
            telemetry.addLine("Scissor Data:");
            telemetry.addData("Scissor Toggle: ", scissorToggle + " " + scissorPos);

            // arm data
            telemetry.addLine("");
            telemetry.addLine("Arm Data:");
            telemetry.addData("Arm Power: ", armPower());

            // update output
            telemetry.update();
        }
    }

    /**
     * @author Gregory L, Anujan K, Andrew C
     * @param inputPowerLeft the value of the power applied to the left lift (range of 0-1)
     * @param inputPowerRight  the value of the power applied to the right lift (range of 0-1)
     * responsible for applying power to the lift.
     */
    public void Lift(double inputPowerLeft, double inputPowerRight){

        leftLift.setPower(inputPowerLeft);
        rightLift.setPower(inputPowerRight);
    }

    /**
     * @author Gregory L, Anujan K, Andrew C
     * @return Power for left lift
     * calculates and returns required power to the left lift.
     */
    public double liftPowerLeft(){

        if (gamepad2.left_stick_y < 0) {
            return -gamepad2.left_stick_y * 0.85;

        } else if (gamepad2.left_stick_y > 0) {

            if (leftLift.getCurrentPosition() > 0) {
                leftLift.setPower(-gamepad2.left_stick_y * 0.30);
                return -gamepad2.left_stick_y * 0.3;
            }

        } else {
            return 0;
        }

        if (leftLift.getCurrentPosition() < 0) {
            return 0.5;
        }

        return 1;
    }

    /**
     * @author Gregory L, Anujan K, Andrew C
     * @return Power for right lift
     * calculates and returns required power to the right lift.
     */
    public double liftPowerRight(){

        if (gamepad2.left_stick_y < 0) {
            return -gamepad2.left_stick_y * 0.85;

        } else if (gamepad2.left_stick_y > 0) {

            if (rightLift.getCurrentPosition() > 0) {
                return -gamepad2.left_stick_y * 0.3;
            }

        } else {
            return 0;
        }

        if (rightLift.getCurrentPosition() < 0) {
            return 0.5;
        }

        return 1;
    }

    /**
     * @author Gregory L
     * @return Power for the arm
     * calculates and returns required power to the arm.
     */
    public double armPower(){


        if (armModeToggle){

            if (gamepad2.right_stick_y > 0 && !armMoveUp) {

                armMoveUp = true;

            }

            if (armMoveUp){

                armStartPos = Arm.getCurrentPosition();
                armMoveUp = false;
            }

            if (gamepad2.right_stick_y > 0){

                if (armStartPos < Plateau){

                    // front to back
                    armReturn = gamepad2.right_stick_y;

                } else {

                    // back to front
                    armReturn = -gamepad2.right_stick_y;

                }

                // back to front slow
                if (Arm.getCurrentPosition() < Plateau && armReturn != Math.abs(armReturn)){
                    armReturn *= slowSpeed;
                }

                // front to back slow
                if (Arm.getCurrentPosition() > Plateau && armReturn == Math.abs(armReturn)){
                    armReturn *= slowSpeed;
                }

            }

        } else {

            // slow down past top point (joystick up)
            if (Arm.getCurrentPosition() > Plateau && gamepad2.right_stick_y > 0) {

                armReturn = gamepad2.right_stick_y * slowSpeed;
            }

            // slow down past top point (joystick down)
            if (Arm.getCurrentPosition() < Plateau && gamepad2.right_stick_y < 0) {

                armReturn = gamepad2.right_stick_y * slowSpeed;
            }

        }


        // return values
        return armReturn;
    }
}