package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Gary 5/29/22
 */

@TeleOp
public class MyFIRSTJavaOpMode1 extends LinearOpMode {
    private DcMotor right_drive;
    private DcMotor left_drive;
    private DcMotor duck;
    private Servo servo;
    private CRServo cr_servo;
    private DistanceSensor sensor_range1;
    private Gyroscope gyro;

    double servoPosition = 0.0;
    double servoPower = 0.0;

    @Override

    public void runOpMode() {
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        duck = hardwareMap.get(DcMotor.class, "duck");
        servo = hardwareMap.get(Servo.class, "servo");
        cr_servo = hardwareMap.get(CRServo.class, "cr_servo");
        sensor_range1 = hardwareMap.get(DistanceSensor.class, "sensor_range1");
        gyro = hardwareMap.get(Gyroscope.class, "gyro");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servoPosition = 0.5;
        servoPower = 0.0;
        servo.setPosition(servoPosition);
        cr_servo.setPower(servoPower);
        time(2000);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Up - No Rotation");
            telemetry.update();
            servoPosition = 0.5;
            servo.setPosition(servoPosition);
            servoPower = 0.0;
            cr_servo.setPower(servoPower);
            time(4000);
            telemetry.addData("Status", "Left - CW Rotation");
            telemetry.update();
            servoPosition = 0.0;
            servo.setPosition(servoPosition);
            servoPower = 1.0;
            cr_servo.setPower(servoPower);
            time(4000);
            telemetry.addData("Status", "Up - No Rotation");
            telemetry.update();
            servoPosition = 0.5;
            servo.setPosition(servoPosition);
            servoPower = 0.0;
            cr_servo.setPower(servoPower);
            time(4000);
            telemetry.addData("Status", "Right - CCW Rotation");
            telemetry.update();
            servoPosition = 1.0;
            servo.setPosition(servoPosition);
            servoPower = -1.0;
            cr_servo.setPower(servoPower);
            time(4000);
        }

    }

    public void time(long Time) {
        sleep(Time);
    }

}