package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Util.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Red", group = "...Sigma")
public class TeleRed extends OpMode {

    Robot r;
    private Follower follower;
    TelemetryManager t;
    public static Pose startingPose = new Pose(23, 128, Math.toRadians(234));
    public static Pose parkPose = new Pose(111,40,Math.toRadians(270));
    public static Pose relocalization = new Pose(132,8,Math.toRadians(0));
    public static Pose relocalization2 = new Pose(33,137.5,Math.toRadians(270));
    public static PathChain park;
    public HubBulkRead bulk;


    @Override
    public void init() {
        startingPose.mirror();
        relocalization.mirror();
        relocalization2.mirror();
        parkPose.mirror();
        follower = Constants.createFollower(hardwareMap);
        bulk = new HubBulkRead(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        t = PanelsTelemetry.INSTANCE.getTelemetry();
        r = new Robot(hardwareMap,follower, t, gamepad1 , gamepad2,false,false,startingPose);
        r.tInit();
        r.setRelocalization(relocalization,relocalization2);
    }

    @Override
    public void init_loop(){
        if(gamepad1.y){
            follower.setPose(startingPose);
        }
    }

    @Override
    public void start() {
        r.tStart();
    }

    @Override
    public void loop() {
        bulk.clearCache(HubBulkRead.Hubs.ALL);
        follower.update();
        r.tPeriodic();
        t.addData("Turret Ticks", r.tu.t);
        t.addData("Turret Position", r.tu.target);
        t.addData("Turret Angle",r.tu.target*355);
        t.addData("Velocity: ", r.s.getVelocity());
        t.addData("Transfer Amps: ", r.i.getAT());
        t.addData("Shooter offset: ", r.s.offset);
        t.addData("Follower Pose: ", r.f.getPose().toString());
        t.addData("Loop time: ", r.getLoopTimeMs());
        t.addData("Loop time hz: ", r.getLoopTimeHz());
        t.update(telemetry);
    }
    @Override
    public void stop(){
        r.stop();
    }
}