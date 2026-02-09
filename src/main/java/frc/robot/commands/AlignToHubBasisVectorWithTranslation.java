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
import frc.robot.subsystems.LimelightBack;
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
    private double rotationalLowerP, lateralLowerP; // lower P if error is small, since degrees is larger unit
    private double lateralErrorThreshold, depthErrorThreshold,
            rotationalErrorThreshold, // determines when error is small enough
            rotationalLowerPThreshold, lateralLowerPThreshold; // determines which rotationalP to use
    private Limelight backLimelight;
    private double tagAngle, desiredX, desiredY;
    private Translation2d desiredPose;
    private double rotationalError, lateralError, depthError;
    private double lateral, rotation;

    public AlignToHubBasisVectorWithTranslation() {
        drivetrain = Drivetrain.getInstance();
        backLimelight = LimelightBack.getInstance();

        rotationalP = HubAlignConstants.kRotationalP;
        rotationalI = HubAlignConstants.kRotationalI;
        rotationalD = HubAlignConstants.kRotationalD;
        rotationalFF = HubAlignConstants.kRotationalFF;
        rotationalLowerP = HubAlignConstants.kRotationLowerP;
        rotationalErrorThreshold = HubAlignConstants.kRotationalErrorThreshold;
        rotationalLowerPThreshold = HubAlignConstants.kRotationLowerPThreshold;
        rotationalPIDController = new PIDController(rotationalP, rotationalI, rotationalD);

        // SmartDashboard.putNumber("rotationalP", 0);
        // SmartDashboard.putNumber("rotationalI", 0);
        // SmartDashboard.putNumber("rotationalD", 0);
        // SmartDashboard.putNumber("rotationalFF", 0);
        // SmartDashboard.putNumber("rotationalLowerP", 0);
        // SmartDashboard.putNumber("rotationalErrorThreshold", 0);
        // SmartDashboard.putNumber("rotationalLowerPThreshold", 0);
        // SmartDashboard.putNumber("rotationalError", 0);
        // rotationalPIDController.enableContinuousInput(-180, 180);

        // lateralP = HubAlignConstants.kLateralP;
        // lateralI = HubAlignConstants.kLateralI;
        // lateralD = HubAlignConstants.kLateralD;
        // lateralFF = HubAlignConstants.kLateralFF;
        // lateralErrorThreshold = HubAlignConstants.kLateralErrorThreshold;
        SmartDashboard.putNumber("lateralP", 0);
        SmartDashboard.putNumber("lateralI", 0);
        SmartDashboard.putNumber("lateralD", 0);
        SmartDashboard.putNumber("lateralFF", 0);
        SmartDashboard.putNumber("lateralErrorThreshold", 0);
        SmartDashboard.putNumber("lateralError", 0);
        SmartDashboard.putNumber("lateralLowerPThreshold", lateralLowerPThreshold);
        SmartDashboard.putNumber("lateralLowerP", lateralLowerP);
        lateralPIDController = new PIDController(lateralP, lateralI, lateralD);
        SmartDashboard.putNumber("lateral", lateral);
        SmartDashboard.putNumber("rotational", rotation);
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
        //tagAngle = tagPose.getRotation().getRadians();
        tagAngle = 180;
        desiredX = tagPose.getX();

        // rotationalP = SmartDashboard.getNumber("rotationalP", 0);
        // rotationalI = SmartDashboard.getNumber("rotationalI", 0);
        // rotationalD = SmartDashboard.getNumber("rotationalD", 0);
        // rotationalFF = SmartDashboard.getNumber("rotationalFF", 0);
        // rotationalLowerP = SmartDashboard.getNumber("rotationalLowerP", 0);
        // rotationalErrorThreshold =
        // SmartDashboard.getNumber("rotationalErrorThreshold", 0);
        // rotationalLowerPThreshold =
        // SmartDashboard.getNumber("rotationalLowerPThreshold", 0);
        SmartDashboard.putNumber("rotationalError", rotationalError);

        lateralP = SmartDashboard.getNumber("lateralP", 0);
        lateralI = SmartDashboard.getNumber("lateralI", 0);
        lateralD = SmartDashboard.getNumber("lateralD", 0);
        lateralFF = SmartDashboard.getNumber("lateralFF", 0);
        lateralLowerPThreshold = SmartDashboard.getNumber("LateralLowerPThreshold", 0);
        lateralLowerP = SmartDashboard.getNumber("LateralLowerP", 0);
        lateralErrorThreshold = SmartDashboard.getNumber("lateralErrorThreshold", 0);
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

        //double rotation = 0;
        if (Math.abs(rotationalError) > rotationalErrorThreshold) {
            SmartDashboard.putNumber("rotational", rotation);
            rotation = rotationalPIDController.calculate(rotationalError) + Math.signum(rotationalError) * rotationalFF;
        }
        
        lateralPIDController.setP(lateralP);
        lateralPIDController.setI(lateralI);
        lateralPIDController.setD(lateralD);

        if (Math.abs(lateralError) > lateralLowerPThreshold) {
            lateralPIDController.setP(lateralP);
        } else {
            lateralPIDController.setP(lateralLowerP);
        }

        // Optional<Pose2d> estimatedPoseOptional = backLimelight.getEstimatedPoseMT2();
        // Pose2d estimatedPose = estimatedPoseOptional.get();
        //lateralError = estimatedPose.getX() - desiredX;
        lateralError = backLimelight.getTx();
        if (Math.abs(lateralError) > lateralErrorThreshold) {
            SmartDashboard.putNumber("lateral", lateral);
            lateral = lateralPIDController.calculate(lateralError) + Math.signum(lateralError) * lateralFF;
        }

        drivetrain.drive(new Translation2d(0, lateral), 0, true, null);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}