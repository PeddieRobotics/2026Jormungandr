package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.utils.Constants.HubAlignConstants;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Retro;

public class AlignToHubBasisVectorWithTranslation extends Command {
    private Drivetrain drivetrain;
    private PIDController lateralPIDController, depthPIDController,
            rotationalPIDController;
    private double lateralP, lateralI, lateralD, lateralFF,
            depthP, depthI, depthD, depthFF,
            rotationalP, rotationalI, rotationalD, rotationalFF;
    private double rotationalLowerP; // lower P if error is small, since degrees is larger unit
    private double lateralErrorThreshold, depthErrorThreshold,
            rotationalErrorThreshold, // determines when error is small enough
            rotationalLowerPThreshold; // determines which rotationalP to use
    private Limelight frontRLimelight;
    private double tagAngle, desiredX, desiredY;
    private Translation2d desiredPose;
    private double rotationalError, lateralError, depthError;

    public AlignToHubBasisVectorWithTranslation() {
        drivetrain = Drivetrain.getInstance();
        frontRLimelight = LimelightFrontRight.getInstance();

        rotationalP = HubAlignConstants.kRotationalP;
        rotationalI = HubAlignConstants.kRotationalI;
        rotationalD = HubAlignConstants.kRotationalD;
        rotationalFF = HubAlignConstants.kRotationalFF;
        rotationalLowerP = HubAlignConstants.kRotationLowerP;
        rotationalErrorThreshold = HubAlignConstants.kRotationalErrorThreshold;
        rotationalLowerPThreshold = HubAlignConstants.kRotationLowerPThreshold;
        rotationalPIDController = new PIDController(rotationalP, rotationalI, rotationalD);
        rotationalPIDController.enableContinuousInput(-180, 180);

        // lateralP = HubAlignConstants.kLateralP;
        // lateralI = HubAlignConstants.kLateralI;
        // lateralD = HubAlignConstants.kLateralD;
        // lateralFF = HubAlignConstants.kLateralFF;
        // lateralErrorThreshold = HubAlignConstants.kLateralErrorThreshold;
        // lateralPIDController = new PIDController(lateralP, lateralI, lateralD);
        SmartDashboard.putNumber("lateralP", 0);
        SmartDashboard.putNumber("lateralErrorThreshold", 0);
        SmartDashboard.putNumber("lateralError", 0);

        // depthP = HubAlignConstants.kDepthP;
        // depthI = HubAlignConstants.kDepthI;
        // depthD = HubAlignConstants.kDepthD;
        // depthFF = HubAlignConstants.kDepthFF;
        // depthErrorThreshold = HubAlignConstants.kDepthErrorThreshold;
        // depthPIDController = new PIDController(depthP, depthI, depthD);

        addRequirements(drivetrain);
        SmartDashboard.putNumber("Target April Tag ID", 0);
    }

    @Override
    public void initialize() {
        Pose2d tagPose = Limelight.getAprilTagPose((int) SmartDashboard.getNumber("Target April Tag ID", 0));
        tagAngle = tagPose.getRotation().getRadians();
        desiredX = 0;

        rotationalP = SmartDashboard.getNumber("rotationalP", 0);
        rotationalI = SmartDashboard.getNumber("rotationalI", 0);
        rotationalD = SmartDashboard.getNumber("rotationalD", 0);
        rotationalFF = SmartDashboard.getNumber("rotationalFF", 0);
        rotationalLowerP = SmartDashboard.getNumber("rotationalLowerP", 0);
        rotationalErrorThreshold = SmartDashboard.getNumber("rotationalErrorThreshold", 0);
        rotationalLowerPThreshold = SmartDashboard.getNumber("rotationalLowerPThreshold", 0);
        SmartDashboard.putNumber("rotationalError", rotationalError);

        rotationalP = SmartDashboard.getNumber("rotationalP", 0);
        rotationalErrorThreshold = SmartDashboard.getNumber("rotationalErrorThreshold", 0);
        SmartDashboard.putNumber("lateralError", lateralError);
    }

    @Override
    public void execute() {
        rotationalError = drivetrain.getHeadingBlue() - tagAngle;
        if (Math.abs(rotationalError) > rotationalLowerPThreshold)
            rotationalPIDController.setP(rotationalP);
        else
            rotationalPIDController.setP(rotationalLowerP);
        rotationalPIDController.setI(rotationalI);
        rotationalPIDController.setD(rotationalD);

        double rotation = 0;
        if (Math.abs(rotationalError) > rotationalErrorThreshold) {
            rotation = rotationalPIDController.calculate(rotationalError) + Math.signum(rotationalError) * rotationalFF;
        }

        double lateral = 0;
        lateralError = frontRLimelight.getTx() - desiredX;
        if (Math.abs(lateralError) > lateralErrorThreshold) {
            lateral = lateralPIDController.calculate(lateralError) + Math.signum(lateralError) * lateralFF;
        }

        drivetrain.drive(new Translation2d(lateral, 0), rotation, true, null);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}