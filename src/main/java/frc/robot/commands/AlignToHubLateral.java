package frc.robot.commands;

import java.net.ContentHandler;
import java.util.Optional;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.HubAlignConstants;

public class AlignToHubLateral extends Command {

    // Create a Drivetrain
    private Drivetrain drivetrain;

    // Create PID Controllers
    private PIDController lateralPIDController, rotationalPIDController;//, depthPIDController;

    // Create the Lateral, Depth, and Rotational PID FF 
    private double lateralP, lateralI, lateralD, lateralFF;
    //private double depthP, depthI, depthD, depthFF;
    private double rotationalP, rotationalI, rotationalD, rotationalFF;

    // Create Threshold for all 3
    private double rotationalLowerPThreshold,
                    rotationalErrorThreshold; // determines if the rotational error is small enough to use lower P threshold
    private double rotationalLowerP;
    private double rotationalError, lateralError, depthError;
    private double lateralThreshold; //depthThreshold; // Unused

    // Create Limelight
    private Limelight frontLimelight;
    
    private double tagAngle, tagX, tagY;
    


    public AlignToHubLateral() {
        drivetrain = Drivetrain.getInstance();
        frontLimelight = LimelightFrontRight.getInstance();

        // rotationalP = Constants.HubAlignConstants.kRotationalP; // Set rotational P
        // rotationalI = Constants.HubAlignConstants.kRotationalI; // Set rotational I
        // rotationalD = Constants.HubAlignConstants.kRotationalD; // Set rotational D

        // // Set all rotational thresholds
        // rotationalErrorThreshold = Constants.HubAlignConstants.kRotationalErrorThreshold;
        // rotationalLowerP = Constants.HubAlignConstants.kRotationalLowerP;
        // rotationalLowerPThreshold = Constants.HubAlignConstants.kRotationalLowerPThreshold;
        // rotationalPIDController = new PIDController(rotationalP, rotationalI, rotationalD); // Set PID Controller for rotationals

        // Set all Lateral PID onto PIDController and Error Threshold
        // lateralP = HubAlignConstants.kLateralP;
        // lateralI = HubAlignConstants.kLateralI;
        // lateralD = HubAlignConstants.kLateralD;
        // lateralFF = HubAlignConstants.kLateralFF;
        // lateralThreshold = HubAlignConstants.kLateralErrorThreshold;

        lateralPIDController = new PIDController(lateralP, lateralI, lateralD);

        // Set all Depth PID onto PIDController and Error Threshold
        // depthP = HubAlignConstants.kDepthP;
        // depthI = HubAlignConstants.kDepthI;
        // depthD = HubAlignConstants.kDepthD;
        // depthThreshold = HubAlignConstants.kDpethErrorThreshold;

        // depthPIDController = new PIDController(depthP, depthI, depthD);


        addRequirements(drivetrain);
        SmartDashboard.putNumber("Target AprilTag ID", 0);
        SmartDashboard.putNumber("Rotational P", rotationalP);
        SmartDashboard.putNumber("Rotational I", rotationalI);
        SmartDashboard.putNumber("Rotational D", rotationalD);
        SmartDashboard.putNumber("Rotational FF", rotationalFF);

        SmartDashboard.putNumber("Rotational Error Threshold", rotationalErrorThreshold);
        SmartDashboard.putNumber("Rotational Lower P", rotationalLowerP);
        SmartDashboard.putNumber("Rotational Lower P Threshold", rotationalLowerPThreshold);
        SmartDashboard.putNumber("Rotational Error", rotationalError);
        
        //Be able to change the rotational values on SmartDashboard
        SmartDashboard.putNumber("Lateral P", lateralP);
        SmartDashboard.putNumber("Lateral I", lateralI);
        SmartDashboard.putNumber("Lateral D", lateralD);
        SmartDashboard.putNumber("Lateral FF", lateralFF);

        SmartDashboard.putNumber("Lateral Threshold", lateralThreshold);
        SmartDashboard.putNumber("Lateral Error", lateralError);


    }

    @Override
    public void initialize() {
        Pose2d tagPose = Limelight.getAprilTagPose((int)SmartDashboard.getNumber("Target AprilTag ID", 0));
        tagAngle = tagPose.getRotation().getRadians();
        
        tagX = 0; // tagPose.getX()
        // tagY = tagPose.getY();
        rotationalP = SmartDashboard.getNumber("RotationalP", 0);
        //Be able to change the rotational values on SmartDashboard
        SmartDashboard.putNumber("Rotational P", rotationalP);
        SmartDashboard.putNumber("Rotational I", rotationalI);
        SmartDashboard.putNumber("Rotational D", rotationalD);
        SmartDashboard.putNumber("Rotational FF", rotationalFF);

        SmartDashboard.putNumber("Rotational Error Threshold", rotationalErrorThreshold);
        SmartDashboard.putNumber("Rotational Lower P", rotationalLowerP);
        SmartDashboard.putNumber("Rotational Lower P Threshold", rotationalLowerPThreshold);
        SmartDashboard.putNumber("Rotational Error", rotationalError);
        
        //Be able to change the rotational values on SmartDashboard
        SmartDashboard.putNumber("Lateral P", lateralP);
        SmartDashboard.putNumber("Lateral I", lateralI);
        SmartDashboard.putNumber("Lateral D", lateralD);
        SmartDashboard.putNumber("Lateral FF", lateralFF);

        SmartDashboard.putNumber("Lateral Threshold", lateralThreshold);
        SmartDashboard.putNumber("Lateral Error", lateralError);
    }

    @Override
    public void execute() {
        //rotational
        rotationalError = drivetrain.getHeadingBlue() - tagAngle;

        if (Math.abs(rotationalError) > rotationalLowerPThreshold) {
            rotationalPIDController.setP(rotationalP);
        } else {
            rotationalPIDController.setP(rotationalLowerP);
        }

        rotationalPIDController.setI(rotationalI);
        rotationalPIDController.setD(rotationalD);


        double rotation = 0;

        if(Math.abs(rotationalError) > rotationalErrorThreshold) {
            rotation = rotationalPIDController.calculate(rotationalError) + Math.signum(rotationalError) * rotationalFF;
        }

        //lateral
        lateralError = frontLimelight.getTx() - tagX;
        
        double lateral = 0;

        if(Math.abs(lateralError) > lateralThreshold) {
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
