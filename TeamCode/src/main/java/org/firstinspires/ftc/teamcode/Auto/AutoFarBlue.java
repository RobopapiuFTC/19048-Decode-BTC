package org.firstinspires.ftc.teamcode.Auto;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Util.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto Far Blue 9 Tangent", group="Blue")
public class AutoFarBlue extends OpMode{

    public HubBulkRead bulk;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean okp,okf;
    private Robot r;
    private TelemetryManager t;
    private double latchT=0.5,XPlace=30;
    private Timer latchTimer;

    private int pathState;
    private Pose startPose = new Pose(56, 6, Math.toRadians(90));
    private Pose scorePose = new Pose(52, 10, Math.toRadians(180));
    private Pose humanPose = new Pose(12,8,Math.toRadians(180));
    private Pose linePose = new Pose(10,36,Math.toRadians(180));
    public Pose endPose = new Pose(40,14,Math.toRadians(180));
    private Pose grabPose = new Pose(12,25,Math.toRadians(180));

    private PathChain scorePreload,grabLine,scoreLine,humanGrab,humanScore,grabBall,scoreGrab,end;
    public void buildPaths() {

        HeadingInterpolator grab = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.8,
                        HeadingInterpolator.tangent
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.8,
                        1,
                        HeadingInterpolator.linear(Math.toRadians(160),grabPose.getHeading())
                )
        );
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        grabLine = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(62,48),
                                new Pose(41,33),
                                linePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(startPose.getHeading(),linePose.getHeading(),0.3)
                .build();
        scoreLine = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(linePose.getHeading(),scorePose.getHeading())
                .build();
        humanGrab = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, humanPose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),humanPose.getHeading())
                .build();
        humanScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(humanPose.getHeading(),scorePose.getHeading())
                .build();
        grabBall = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, grabPose)
                )
                .setBrakingStrength(2)
                .setHeadingInterpolation(grab)
                .build();
        scoreGrab = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(grabPose.getHeading(),scorePose.getHeading())
                .build();
        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, endPose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),endPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1);
                follower.followPath(scorePreload,true);
                oktrue();
                r.tu.face(r.getShootTarget(), scorePreload.endPose());
                r.s.on();
                r.s.forDistance(r.getShootTarget().distanceFrom(scorePreload.endPose()));
                r.s.latchdown();
                nextPath();
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.i.start();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(grabLine, true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scoreLine,true);
                    r.tu.face(r.getShootTarget(), scoreLine.endPose());
                    r.s.on();
                    r.s.forDistance(r.getShootTarget().distanceFrom(scoreLine.endPose()));
                    oktrue();
                    nextPath();
                }
                break;

            case 3:
                if(follower.getPose().getX()>XPlace && okf){
                    r.i.stop();
                    okf=false;
                    latchTimer.resetTimer();
                }
                if(!okf && latchTimer.getElapsedTimeSeconds()>latchT)r.s.latchdown();
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.i.start();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(humanGrab,true);
                        intake();
                        oktrue();
                        nextPath();
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.4) {
                        follower.followPath(humanScore, true);
                        r.tu.face(r.getShootTarget(), humanScore.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(humanScore.endPose()));
                        oktrue();
                        nextPath();
                    }
                }
                break;
            case 5:
                if(follower.getPose().getX()>XPlace && okf){
                    r.i.stop();
                    okf=false;
                    latchTimer.resetTimer();
                }
                if(!okf && latchTimer.getElapsedTimeSeconds()>latchT)r.s.latchdown();
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();

                        r.i.start();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(grabBall,true);
                        intake();
                        oktrue();
                        nextPath();
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.4) {
                        follower.followPath(scoreGrab, true);
                        r.tu.face(r.getShootTarget(), scoreGrab.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(scoreGrab.endPose()));
                        oktrue();
                        nextPath();
                    }
                }
                break;
            case 7:
                if(follower.getPose().getX()>XPlace && okf){
                    r.i.stop();
                    okf=false;
                    latchTimer.resetTimer();
                }
                if(!okf && latchTimer.getElapsedTimeSeconds()>latchT)r.s.latchdown();
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.i.start();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(humanGrab,true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.4) {
                        follower.followPath(humanScore, true);
                        r.tu.face(r.getShootTarget(), humanScore.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(humanScore.endPose()));
                        oktrue();
                        nextPath();
                    }
                }
                break;
            case 9:
                if(follower.getPose().getX()>XPlace && okf){
                    r.i.stop();
                    okf=false;
                    latchTimer.resetTimer();
                }
                if(!okf && latchTimer.getElapsedTimeSeconds()>latchT)r.s.latchdown();
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.i.start();

                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(grabBall,true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.4) {
                        follower.followPath(scoreGrab, true);
                        r.tu.face(r.getShootTarget(), scoreGrab.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(scoreGrab.endPose()));
                        oktrue();
                        nextPath();
                    }
                }
                break;
            case 11:
                if(follower.getPose().getX()>XPlace && okf){
                    r.i.stop();
                    okf=false;
                    latchTimer.resetTimer();
                }
                if(!okf && latchTimer.getElapsedTimeSeconds()>latchT)r.s.latchdown();
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.i.start();

                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(humanGrab,true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.4) {
                        follower.followPath(humanScore, true);
                        r.tu.face(r.getShootTarget(), humanScore.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(humanScore.endPose()));
                        oktrue();
                        nextPath();
                    }
                }
                break;
            case 13:
                if(follower.getPose().getX()>XPlace && okf){
                    r.i.stop();
                    okf=false;
                    latchTimer.resetTimer();
                }
                if(!okf && latchTimer.getElapsedTimeSeconds()>latchT)r.s.latchdown();
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();

                        r.i.start();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(end,true);
                        r.aiming=false;
                        r.i.stop();
                        r.i.triangle=true;
                        r.tu.setYaw(0);
                        endPath();
                    }
                }
                break;
        }
    }
    public void oktrue(){
        okp=true;
        okf=true;
    }
    public void intake(){
        r.i.intake();
        r.s.latchup();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void nextPath(){
        pathState++;
        pathTimer.resetTimer();
    }

    public void endPath(){
        pathState=-1;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
        bulk.clearCache(HubBulkRead.Hubs.ALL);
        follower.update();
        r.aPeriodic();
        autonomousPathUpdate();
        telemetry.addData("Follower Pose: ",follower.getPose().toString());
        telemetry.addData("Dist: ", r.dist);
        telemetry.addData("Velocity: ",r.s.getVelocity());
        telemetry.addData("Target Velocity", r.s.getTarget());
        telemetry.addData("Turret Ticks: ", r.tu.getTurret());
        telemetry.addData("Turret Target: ",r.tu.getTurretTarget());
        telemetry.update();

    }

    @Override
    public void init() {
        bulk = new HubBulkRead(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        latchTimer=new Timer();
        latchTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.usePredictiveBraking=true;
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,true,true,startPose);
        r.setShootTargetFar();
        r.i.triangle=true;
    }
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop() {
        r.stop();
    }
}
