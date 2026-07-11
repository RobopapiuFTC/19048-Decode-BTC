package org.firstinspires.ftc.teamcode.Systems;
import android.graphics.Color;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import com.pedropathing.util.Timer;
public class Intake {
    public DcMotorEx intake,transfer;
    public Servo flick;
    public boolean pornit=false,ip=false,tp=false,intaking=false,done=true;
    public double jos=0.5,sus=0.7;
    public Timer iTimer;

    public static double power=1;
    public static boolean triangle=true;

    public Intake(HardwareMap hw, TelemetryManager telemetry){
        flick=hw.get(Servo.class, "f");
        intake=hw.get(DcMotorEx.class, "i");
        transfer=hw.get(DcMotorEx.class, "t");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        iTimer=new Timer();
    }
    public void periodic(){
        //full();
        run();
        flick();
    }
    public void start() {
        intaking=false;
        pornit=true;
        intake.setPower(1);
        transfer.setPower(power);
        ip = true;
        tp = true;
    }
    public void shoot(){
        intaking=false;
        pornit=true;
        intake.setPower(1);
        transfer.setPower(power);
        ip=true;
        tp=true;
        done=false;
        iTimer.resetTimer();
    }
    public void intake(){
        intaking=true;
        pornit=true;
        intake.setPower(1);
        if(triangle)transfer.setPower(-1);
        else transfer.setPower(1);
        ip=true;
        tp=true;
        done=true;
        flick.setPosition(jos);
    }
    public void flick(){
        if(!done){
            if(iTimer.getElapsedTimeSeconds()>0.15 && iTimer.getElapsedTimeSeconds()<0.3){
                flick.setPosition(sus);
            }
            if(iTimer.getElapsedTimeSeconds()>0.3){
                transfer.setPower(0);
                flick.setPosition(jos);
                done=true;
            }
        }
    }
    public void startI(){
        pornit=true;
        ip=true;
    }

    public void startT(){
        pornit=true;
        tp=true;
    }
    public void stop(){
        intaking=false;
        pornit=false;
        ip=false;
        tp=false;
    }

    public double getVeloI(){
        return intake.getVelocity();
    }
    public double getVeloT(){
        return transfer.getVelocity();
    }
    public double getAT(){return transfer.getCurrent(CurrentUnit.AMPS);}
    public void full(){
        if(!intaking)return;
        if(getAT()>1){
                tp=false;
                transfer.setPower(0);
            }
    }
    public void run(){
        if(!pornit){
            intake.setPower(0);
            transfer.setPower(0);
        }
    }


}