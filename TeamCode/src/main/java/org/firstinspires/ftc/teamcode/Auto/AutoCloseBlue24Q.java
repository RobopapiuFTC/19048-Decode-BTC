package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Util.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto Close Blue 24 Quantam", group="Blue")
public class AutoCloseBlue24Q extends OpMode{
    public HubBulkRead bulk;
    private TelemetryManager t;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer,latchTimer;
    private Robot r;
    private boolean okp,okf,intake=false;
    private double latchT=0.5,XPlace=30;
    private int pathState;
    private  Pose startPose = new Pose(23, 128, Math.toRadians(234));
    private  Pose scorePose = new Pose(57, 90, Math.toRadians(210));
    private  Pose doorPose = new Pose(17,61,Math.toRadians(150));
    private  Pose line1Pose = new Pose(21, 84, Math.toRadians(180));
    private  Pose line2Pose = new Pose(21, 60, Math.toRadians(180));
    private  Pose line3Pose = new Pose(17, 35, Math.toRadians(180));
    public  Pose endPose = new Pose(58,110,Math.toRadians(241));
    private PathChain scorePreload,doorPickup,grabPickup1,doorOpen1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,end,scoreDoor,doorMove;
    private HeadingInterpolator score,gate;
    public void buildPaths() {
        score = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.8,
                        HeadingInterpolator.tangent
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.8,
                        1,
                        HeadingInterpolator.constant(Math.toRadians(180))
                )
        );

        gate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.4,
                        HeadingInterpolator.tangent
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.4,
                        1,
                        HeadingInterpolator.constant(doorPose.getHeading())
                )
        );
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,
                                new Pose(50,84,Math.toRadians(180)))
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(startPose.getHeading(),Math.toRadians(180))
                .build();

        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                line1Pose)
                )
                .setBrakingStrength(2)
                .setConstantHeadingInterpolation(180)
                .build();
        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(50,83,0))
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line1Pose.getHeading(),Math.toRadians(180))
                .build();
        grabPickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(46, 56),
                                new Pose(35, 58),
                                line2Pose
                        )
                )
                .setBrakingStrength(2)
                .setHeadingInterpolation(score)
                .build();
        scorePickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                scorePose
                        )
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation().setReversed()
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1);
                follower.followPath(scorePreload,true);
                r.tu.face(r.getShootTarget(), scorePreload.endPose());
                r.s.forDistance(r.getShootTarget().distanceFrom(scorePreload.endPose()));
                r.s.latchdown();
                oktrue();
                nextPath();
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(okp){
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.shoot();
                        okp=false;
                    }
                    if(r.i.done && pathTimer.getElapsedTimeSeconds()>0.1) {
                        follower.followPath(grabPickup1, true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1,true);
                    r.tu.face(r.getShootTarget(), scorePickup1.endPose());
                    r.s.on();
                    r.s.forDistance(r.getShootTarget().distanceFrom(scorePickup1.endPose()));
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
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.shoot();
                        okp=false;
                    }
                    if(r.i.done && pathTimer.getElapsedTimeSeconds()>0.1) {
                        follower.followPath(grabPickup2,true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    // r.tu.face(r.getShootTarget(), new Pose(scorePose.getX(),scorePose.getY(),Math.toRadians(270)));
                    follower.followPath(scorePickup2,true);
                    r.tu.face(r.getShootTarget(), scorePickup2.endPose());
                    r.s.on();
                    r.s.forDistance(r.getShootTarget().distanceFrom(scorePickup2.endPose()));
                    oktrue();
                    nextPath();
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
                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(40,60),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setConstantHeadingInterpolation(doorPose.getHeading())
                            .build();
                    if(okp){
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.shoot();
                        okp=false;
                    }
                    if(r.i.done && pathTimer.getElapsedTimeSeconds()>0.1) {
                        follower.followPath(doorPickup, true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(follower::getPose,
                                            scorePose)
                            )
                            .setBrakingStrength(2)
                            .setTangentHeadingInterpolation().setReversed()
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(scoreDoor, true);
                        r.tu.face(r.getShootTarget(), scoreDoor.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(scoreDoor.endPose()));
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
                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(40,60),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setConstantHeadingInterpolation(doorPose.getHeading())
                            .build();
                    if(okp){
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.shoot();
                        okp=false;
                    }
                    if(r.i.done && pathTimer.getElapsedTimeSeconds()>0.1) {
                        follower.followPath(doorPickup, true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(follower::getPose,
                                            scorePose)
                            )
                            .setBrakingStrength(2)
                            .setTangentHeadingInterpolation().setReversed()
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(scoreDoor, true);
                        r.tu.face(r.getShootTarget(), scoreDoor.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(scoreDoor.endPose()));
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
                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(40,60),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setConstantHeadingInterpolation(doorPose.getHeading())
                            .build();
                    if(okp){
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.shoot();
                        okp=false;
                    }
                    if(r.i.done && pathTimer.getElapsedTimeSeconds()>0.1) {
                        follower.followPath(doorPickup, true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(follower::getPose,
                                            scorePose)
                            )
                            .setBrakingStrength(2)
                            .setTangentHeadingInterpolation().setReversed()
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(scoreDoor, true);
                        r.tu.face(r.getShootTarget(), scoreDoor.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(scoreDoor.endPose()));
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
                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(40,60),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setConstantHeadingInterpolation(doorPose.getHeading())
                            .build();
                    if(okp){
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.shoot();
                        okp=false;
                    }
                    if(r.i.done && pathTimer.getElapsedTimeSeconds()>0.1) {
                        follower.followPath(doorPickup, true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(follower::getPose,
                                            scorePose)
                            )
                            .setBrakingStrength(2)
                            .setTangentHeadingInterpolation().setReversed()
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(scoreDoor, true);
                        r.tu.face(r.getShootTarget(), scoreDoor.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(scoreDoor.endPose()));
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
                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(40,60),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setConstantHeadingInterpolation(doorPose.getHeading())
                            .build();
                    if(okp){
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.shoot();
                        okp=false;
                    }
                    if(r.i.done && pathTimer.getElapsedTimeSeconds()>0.1) {
                        follower.followPath(doorPickup, true);
                        intake();
                        oktrue();
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(follower::getPose,
                                            endPose)
                            )
                            .setBrakingStrength(2)
                            .setTangentHeadingInterpolation().setReversed()
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(scoreDoor, true);
                        r.tu.face(r.getShootTarget(), scoreDoor.endPose());
                        r.s.on();
                        r.s.forDistance(r.getShootTarget().distanceFrom(scoreDoor.endPose()));
                        oktrue();
                        nextPath();
                    }
                }
                break;
            case 15:
                if(follower.getPose().getX()>XPlace && okf){
                    r.i.stop();
                    okf=false;
                    latchTimer.resetTimer();
                }
                if(!okf && latchTimer.getElapsedTimeSeconds()>latchT)r.s.latchdown();
                if(!follower.isBusy()) {
                    if(okp){
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.shoot();
                        okp=false;
                    }
                    if(r.i.done && pathTimer.getElapsedTimeSeconds()>0.1) {
                        oktrue();
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
        latchTimer = new Timer();
        latchTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.usePredictiveBraking=true;
        follower.setStartingPose(startPose);
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,true,true,startPose);
        r.setShootTarget();
        r.i.triangle=false;
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
