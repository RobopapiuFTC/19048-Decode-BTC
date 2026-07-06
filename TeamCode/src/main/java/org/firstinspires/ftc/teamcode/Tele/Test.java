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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Util.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Test", group = "...Sigma")
public class Test extends OpMode {

    Robot r;
    private Follower follower;
    private ServoImplEx t1,t2;
    TelemetryManager t;
    public static Pose startingPose = new Pose(23, 128, Math.toRadians(234));
    public static Pose parkPose = new Pose(111,40,Math.toRadians(270));
    public static Pose relocalization = new Pose(132,8,Math.toRadians(0));
    public static Pose relocalization2 = new Pose(18,80,Math.toRadians(180));
    public static PathChain park;
    public HubBulkRead bulk;


    @Override
    public void init() {
        t1=hardwareMap.get(ServoImplEx.class, "t1");
        t2=hardwareMap.get(ServoImplEx.class, "t2");
        t1.setPwmRange(new PwmControl.PwmRange(505,2495));
        t2.setPwmRange(new PwmControl.PwmRange(505,2495));
            t1.setPosition(0);
            t2.setPosition(0);
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }
    @Override
    public void stop(){

    }
}