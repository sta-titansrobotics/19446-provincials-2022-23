package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config

public class actualArmTest extends OpMode {
    private PIDController controller;

    public static double p = 0.13, i = 0, d = 0.0001;
    public static double f = 0.2;

    public static int target = 0;

    private final double tickstoDegree = 1.19;

    private DcMotor arm;


    @Override
    public void init() {

        arm = hardwareMap.get(DcMotor.class, "arm");
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    }

    @Override
    public void loop() {

        if (gamepad2.right_stick_y < 0) {

            target += 2;


        } else if (gamepad2.right_stick_y > 0) {

            target -= 2;


        }

        if (arm.getCurrentPosition() < 0) {
            arm.setPower(0.5);
        }



        controller.setPID(p, i, d);
        int armPos = arm.getCurrentPosition();

        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / tickstoDegree)) * f;

        double power = ff + pid;

        arm.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.update();

    }



}