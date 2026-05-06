package org.firstinspires.ftc.teamcode.Systems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Movement {
    public DcMotorEx leftFront,leftRear,rightFront,rightRear;
    public double x,y;

    public Movement(HardwareMap hardwareMap, TelemetryManager telemetry){
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void periodic(Gamepad gamepad) {
        movement(gamepad);
    }
    public void movement(Gamepad gamepad1){
        if(gamepad1.left_stick_y>0.7)y = -1;
        else if (gamepad1.left_stick_y<-0.7)y=1;
        else y=-gamepad1.left_stick_y;
        if(gamepad1.left_stick_x>0.7)x = 1*1.1;
        else if (gamepad1.left_stick_x<-0.7)x=-1*1.1;
        else x=-gamepad1.left_stick_x*1.1;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        if(gamepad1.right_trigger>0.3){
            leftFront.setPower(frontLeftPower/3);
            leftRear.setPower(backLeftPower/3);
            rightFront.setPower(frontRightPower/3);
            rightRear.setPower(backRightPower/3);
        } else {
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);
        }
    }
}