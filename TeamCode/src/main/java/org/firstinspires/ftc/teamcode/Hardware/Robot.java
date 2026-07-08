package org.firstinspires.ftc.teamcode.Hardware;
import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.asin;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Movement;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
public class Robot {
    public HardwareMap h;
    public TelemetryManager t;
    public Gamepad g1,g2;
    public Follower f;
    public Shooter s;
    public Intake i;
    public Turret tu;
    public static Pose currentPose,endPose,futurePose,r1,r2,startingPose;
    public static Pose shootp = new Pose(0 ,144,0);
    public boolean a,auto;
    public static double dist,offsetFar=0,offsetClose=0,batteryVoltage=0,nominalVoltage=12.5;
    public double loopTime,lastLoop,timing=0.7,loops=0;
    public boolean aiming=false,slowmode=false,shoot,oks,turret45,shooting,aim,oki=false;
    public static boolean intake=false;
    public Timer iTimer,rTimer,rsTimer,sTimer,oTimer,loopTimer;

    public Robot(HardwareMap h, Follower f, TelemetryManager t, Gamepad g1, Gamepad g2, boolean blue, boolean auto,Pose startingPose) {
        this.h = h;
        this.t = t;
        this.f = f;
        this.g1 = g1;
        this.g2 = g2;
        this.a = blue;
        this.auto = auto;
        this.startingPose=startingPose;

        s=new Shooter(this.h,this.t,this.a);
        i=new Intake(this.h,this.t);
        tu=new Turret(this.h,this.t,this.auto);
        i.triangle=true;
        loopTimer=new Timer();
        iTimer=new Timer();
        sTimer=new Timer();
        rTimer=new Timer();
        oTimer=new Timer();
    }
    public void stop(){
        endPose=f.getPose();
    }
    public void tStart(){
        f.startTeleOpDrive();
    }
    public void tInit(){
        if(endPose==null){
            f.setPose(startingPose);
        }
        else f.setPose(endPose);
        poses();
        tu.automatic();
    }
    public void tPeriodic(){
        Controls();
        drive();
        loop();
        shoot(currentPose);
        i.periodic();
        s.periodic();
        tu.periodic();
    }
    public void aPeriodic() {
        poses();
        loop();
        if(aiming){
            tu.face(getShootTarget(),currentPose);
            tu.automatic();
        }
        s.periodic();
        tu.periodic();
        i.periodic();
    }

    public void shoot(){
        if(sTimer.getElapsedTimeSeconds()<0.2){
            f.holdPoint(currentPose);
        }
        else{
            i.start();
            f.startTeleOpDrive();
        }
    }
    public void drive(){
            if (slowmode) {
                f.setTeleOpDrive(
                        -g1.left_stick_y * 0.3,
                        -g1.left_stick_x * 0.3,
                        -g1.right_stick_x * 0.3,
                        true
                );
            } else f.setTeleOpDrive(
                    -g1.left_stick_y,
                    -g1.left_stick_x,
                    -g1.right_stick_x,
                    true
            );
    }
    public void Controls(){
        if(g1.y){
            if(rTimer.getElapsedTimeSeconds()>0.3){
                intake=true;
                oki=true;
                shooting=false;
                rTimer.resetTimer();
            }
        }
        if(g1.share){
            if(rTimer.getElapsedTimeSeconds()>0.3){
                i.triangle=!i.triangle;
                rTimer.resetTimer();
            }
        }
        if(g1.x && g1.left_trigger<0.3){
            if(rTimer.getElapsedTimeSeconds()>0.3){
                shooter();
                rTimer.resetTimer();
            }
        }
        if(g1.x && g1.left_trigger>0.3){
            if(rTimer.getElapsedTimeSeconds()>0.3){
                i.pornit=false;
                s.setPower(-1);
                rTimer.resetTimer();
            }
        }
        if(g1.b && !g1.left_bumper){
            f.setPose(r1);
            offsetClose=0;
            offsetFar=0;
        }
        if(g1.b && g1.left_bumper){
            f.setPose(r2);
            offsetClose=0;
            offsetFar=0;
        }
        if(g1.dpad_left && !g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                offset(Math.toRadians(1));
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_right && !g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                offset(-Math.toRadians(1));
                oTimer.resetTimer();
            }
        }if(g1.dpad_left && g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                offset(Math.toRadians(90));
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_right && g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                offset(-Math.toRadians(90));
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_up){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                s.offset=s.offset+10;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_down){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                s.offset=s.offset-10;
                oTimer.resetTimer();
            }
        }
        if(g1.a){
            i.start();
        }
        slowmode = g1.right_trigger > 0.3;
        if(g1.options){
            turret45=true;
            i.pornit=false;
            tu.setYaw(Math.toRadians(45));
        }
    }
    public void sequenceshoot(){
        if(shoot) {
            if(oks){
                s.on();
                sTimer.resetTimer();
                oks=false;
                turret45=false;
            }
            if(sTimer.getElapsedTimeSeconds()>0.2 && sTimer.getElapsedTimeSeconds()<1){
                i.stop();
                s.latchdown();
                shoot=false;
                oks=false;
            }
        }
    }
    public void sequenceintake(){
        if (intake) {
            if (oki) {
                turret45 = false;
                shooting = false;
                i.stop();
                iTimer.resetTimer();
                oki = false;
            }
            if (iTimer.getElapsedTimeSeconds() < 0.6) {
                s.latchup();
            }
            if (iTimer.getElapsedTimeSeconds() > 0.6 && iTimer.getElapsedTimeSeconds() < 1) {
                i.intake();
                intake = false;
                oki = false;
            }
        }
    }

    public void shoot(Pose pose) {
        dist = shootp.distanceFrom(pose);
        s.forDistance(dist);
        tu.face(getShootTarget(),pose);
        tu.automatic();
    }
    public void loop(){
        poses();
        loopTime();
        sequenceshoot();
        sequenceintake();
    }
    public void loopTime(){
        loops++;
        if (loops > 10) {
            double now = loopTimer.getElapsedTime();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }
    }
    public void poses(){
        currentPose=f.getPose();
        futurePose = new Pose(currentPose.getX()+f.getVelocity().getXComponent()*timing, currentPose.getY()+f.getVelocity().getYComponent()*timing,currentPose.getHeading());
        shootTarget();
        offsets();
    }
    public void setRelocalization(Pose relocalization, Pose relocalization2){
        this.r1=relocalization;
        this.r2=relocalization2;
    }
    public void offset(double offset){
        if(currentPose.getY()>40)offsetClose += offset;
        else offsetFar += offset;
    }
    public void offsets(){
        if(currentPose.getY()>40)tu.offset = offsetClose;
        else tu.offset=offsetFar;
    }
    public void shootTarget(){
        if(currentPose.getY()>40){setShootTarget();}
        else setShootTargetFar();
    }
    public void setShootTarget() {
        if (a){shootp = new Pose(0, 144, 0);}
        else {shootp = new Pose(144, 144, 0);}
        i.power=1;
    }
    public void setShootTargetFar(){
        if (a){shootp = new Pose(2, 142, 0);}
        else {shootp = new Pose(142, 142, 0);}
        i.power=0.8;
    }
    public Pose getShootTarget() {
        return shootp;
    }
    public double getLoopTimeMs(){
        return loopTime;
    }
    public double getLoopTimeHz(){
        return 1000/loopTime;
    }
    public void shooter(){
        shoot=true;
        oks=true;
        aiming=true;
        tu.okt=true;
    }
}