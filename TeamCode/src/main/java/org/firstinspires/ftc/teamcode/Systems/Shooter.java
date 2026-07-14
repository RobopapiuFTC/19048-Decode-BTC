package org.firstinspires.ftc.teamcode.Systems;

import static androidx.core.math.MathUtils.clamp;

import android.service.controls.Control;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.ShotSample;

import com.pedropathing.util.Timer;

import java.util.Arrays;
import java.util.List;
@Configurable
public class Shooter {
    public DcMotorEx SS,SD;
    public Servo SVD,latch;
    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.01;
    public boolean activated = true;
    public static double sv = 355/270;

    public double offset,latching=0.75;
    public static double hood,angle=0.0005*sv,anglemax=0.05;

    /* public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(50, 1230, 0.0005*sv,0.44),
            new ShotSample(55, 1240, 0.0005*sv,0.44),
            new ShotSample(60, 1250, 0.0005*sv,0.5),
            new ShotSample(65, 1260+20, 0.0005*sv,0.7),
            new ShotSample(70, 1270+20, 0.0005*sv,0.7),
            new ShotSample(75, 1280+20, 0.0005*sv,0.7),
            new ShotSample(80, 1290+20, 0.0005*sv,0.7),
            new ShotSample(85, 1310+20, 0.0005*sv,0.7),
            new ShotSample(90, 1330+20, 0.0005*sv,0.7),
            new ShotSample(95, 1350+20, 0.0005*sv,0.7),
            new ShotSample(100, 1370+20, 0.0005*sv,0.7),
            new ShotSample(105, 1390+20, 0.0005*sv,0.7),
            new ShotSample(110, 1410+20, 0.0005*sv,0.7),
            new ShotSample(115, 1450+20, 0.0005*sv,0.7),
            new ShotSample(120, 1490+20, 0.0005*sv,0.7),
            new ShotSample(125, 1510+20, 0.0005*sv,0.55),
            new ShotSample(130, 1530+20, 0.0005*sv,0.55),
            new ShotSample(140, 1570+20, 0.0005*sv,0.55),
            new ShotSample(150, 1630+20, 0.0005*sv,0.55),
            new ShotSample(160, 1650+50, 0.0005*sv,0.55)
    ); */
    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(50, 1160, 0.0005*sv,0.44),
            new ShotSample(55, 1180, 0.0005*sv,0.44),
            new ShotSample(60, 1220, 0.0005*sv,0.5),
            new ShotSample(65, 1240, 0.0005*sv,0.7),
            new ShotSample(70, 1260, 0.0005*sv,0.7),
            new ShotSample(75, 1280, 0.0005*sv,0.7),
            new ShotSample(80, 1310, 0.0005*sv,0.7),
            new ShotSample(85, 1310+20, 0.0005*sv,0.7),
            new ShotSample(90, 1330+20, 0.0005*sv,0.7),
            new ShotSample(95, 1350+20, 0.0005*sv,0.7),
            new ShotSample(100, 1370+20, 0.0005*sv,0.7),
            new ShotSample(105, 1390+20, 0.0005*sv,0.7),
            new ShotSample(110, 1410+20, 0.0005*sv,0.7),
            new ShotSample(115, 1450+20, 0.0005*sv,0.7),
            new ShotSample(120, 1490+20, 0.0005*sv,0.7),
            new ShotSample(125, 1510+20, 0.0005*sv,0.55),
            new ShotSample(130, 1530+20, 0.0005*sv,0.55),
            new ShotSample(140, 1570+20, 0.0005*sv,0.55),
            new ShotSample(150, 1630+20, 0.0005*sv,0.55),
            new ShotSample(160, 1650+50, 0.0005*sv,0.55)
    );

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetry,boolean auto){

        SS=hardwareMap.get(DcMotorEx.class, "SS");
        SD=hardwareMap.get(DcMotorEx.class, "SD");
        SVD=hardwareMap.get(Servo.class, "SVD");
        latch=hardwareMap.get(Servo.class,"latch");

        SD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(auto){
            SVD.setPosition(0.1);
            latch.setPosition(0.75);
        }
        SD.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void periodic(){
        if (activated)
            setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);

        hood();
    }
    public void hood(){
        hood=clamp(anglemax-(getTarget()-getVelocity()-20)*angle,0.1,anglemax);
        SVD.setPosition(hood);
    }
    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return SD.getVelocity();
    }
    public void setPower(double p) {
        SS.setPower(p);
        SD.setPower(p);
    }
    public void off() {
        activated = false;
        setPower(0);
    }
    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    private ShotSample lookupShot(double d) {

        ShotSample before = null;
        ShotSample after  = null;

        // find closest sample <= d  and closest sample >= d
        for (ShotSample s : samples) {
            if (s.distance <= d) {
                if (before == null || s.distance > before.distance) {
                    before = s;
                }
            }
            if (s.distance >= d) {
                if (after == null || s.distance < after.distance) {
                    after = s;
                }
            }
        }

        if (before == null) return after;
        if (after == null) return before;
        if (before.distance == after.distance) {
            return before;
        }

        double t = (d - before.distance) / (after.distance - before.distance);

        double power = lerp(before.power, after.power, t);
        angle = lerp(before.angle, after.angle, t);

        return new ShotSample(d, power, angle,after.anglemax);
    }

    public void on() {
        activated = true;
    }
    public void setTarget(double velocity) {
        t = velocity;
    }
    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }
    public void forDistance(double distance) {
        ShotSample shoot = lookupShot(distance);
        anglemax=shoot.anglemax;
        setTarget(shoot.power + offset);
    }
    public void hoodfar(){
        SVD.setPosition(0.15);
    }
    public void hoodclose(){
        SVD.setPosition(0.15);
    }
    public void latchdown(){latch.setPosition(0.24);}
    public void latchup(){latch.setPosition(latching); }

}