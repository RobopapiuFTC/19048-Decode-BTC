package org.firstinspires.ftc.teamcode.Systems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Camera {
    private Limelight3A limelight;
    public LLResult result;
    private Telemetry telemetry;
    private boolean blue;
    public double tx,ty,ta;

    public Camera(HardwareMap hardwareMap, TelemetryManager telemetry, boolean blue){

        limelight=hardwareMap.get(Limelight3A.class, "limelight");
        if(blue)limelight.pipelineSwitch(3); //april tag pipeline blue
        else limelight.pipelineSwitch(4); //april tag pipeline red
    }

    public void periodic(){
        detect();
    }
    public void start(){
        limelight.start();
    }
    public void detect(){
        result = limelight.getLatestResult();
        if(result!=null && result.isValid()){
            tx= result.getTx();
            ty= result.getTy();
            ta= result.getTa();
        }
    }
}