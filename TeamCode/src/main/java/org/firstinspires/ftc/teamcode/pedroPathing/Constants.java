package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration((-26 + (-29) + (-27.2)) / 3.0)
            .lateralZeroPowerAcceleration(((-51) + (-60.556) + (-47.047)) / 3.0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.1211, 0.0017011))
           // .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3,0.0607646989,0.00277414))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, .01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.003,0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.02, 0, 0.0004, 0.6, 0)
            )
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.02, 0, 0.0008, 0.6, 0)
            );
         /*
         public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
            .forwardZeroPowerAcceleration((-38.04 + (-32) + (-40.17)) / 3.0)
            .lateralZeroPowerAcceleration(((-70.57) + (-66.93) + (-67.08)) / 3.0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.05, 0.11, 0.0008))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.000)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.14, 0, 0.008, 0.0))
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.2, 0, 0.012, 0.0)
            )
            .headingPIDFCoefficients(new PIDFCoefficients(1.6, 0, 0.08, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.08, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.02, 0, 0.0004, 0.6, 0)
            )
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.02, 0, 0.0008, 0.6, 0)
            );
          */

    public static PathConstraints pathConstraints = new PathConstraints(0.95,100 , 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(84)
            .yVelocity(68)
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true)
            .nominalVoltage(12.5);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(-4.331)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}