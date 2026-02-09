package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToHPBasisVector;
import frc.robot.commands.AlignToReefBasisVector;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Logger;

public class Autonomous {

    private static Autonomous autonomous;
    private SendableChooser<Command> autoChooser;
    private SendableChooser<Double> autoStartPosition;

    public static Autonomous getInstance() {
        if (autonomous == null)
            autonomous = new Autonomous();
        return autonomous;
    }

    RobotConfig config;

    private void startSnakeDrive() {
        PPHolonomicDriveController.overrideRotationFeedback(() -> {
            return Drivetrain.getInstance().getRotationOverride();
        });
    }

    private void stopSnakeDrive() {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    private Autonomous() {
        Drivetrain drivetrain = Drivetrain.getInstance();

        // config = new RobotConfig(
        //     74.088,
        //     6.883,
        //     new ModuleConfig(0.048, 5.450, 1.200, DCMotor.getKrakenX60(1), 60, 1),
        //     new Translation2d(0.273, 0.273),
        //     new Translation2d(0.273, -0.273),
        //     new Translation2d(-0.273, 0.273),
        //     new Translation2d(-0.273, -0.273)
        // );

        try {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            drivetrain::getPose,
            drivetrain::setPose,
            drivetrain::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            drivetrain // Reference to this subsystem to set requirements
        );

        autoChooser = new SendableChooser<>();
        
        // Command rightOutDepClimbAuto = new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         Drivetrain.getInstance().setStartingPose(new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)));
        //         startSnakeDrive();
        //     }),
        //     new AutoDriveCommand(
        //         List.of(
        //             new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)), 
        //             new Pose2d(1.257, 0.639, Rotation2d.fromDegrees(180.000)),
        //             new Pose2d(2.380, 3.729, Rotation2d.fromDegrees(61.557)),
        //             new Pose2d(1.257, 5.882, Rotation2d.fromDegrees(156.038)),
        //             new Pose2d(1.069, 4.702, Rotation2d.fromDegrees(-59.534))
        //         ),
        //         List.of(
        //             new EventMarker("Stop snake drive", 2.42, new InstantCommand(() -> stopSnakeDrive()))
        //         ),
        //         new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
        //         // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
        //         new IdealStartingState(0, Rotation2d.fromDegrees(180)),
        //         new GoalEndState(0, Rotation2d.fromDegrees(180))
        //     )
        // );

        Command leftDepotOutpostClimbRAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(3.560, 5.853, Rotation2d.fromDegrees(180)));
                //startSnakeDrive();
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(3.560, 5.853, Rotation2d.fromDegrees(180)), 
                    new Pose2d(0.960, 6.008, Rotation2d.fromDegrees(-159.829))                    
                ),
                new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                new GoalEndState(0, Rotation2d.fromDegrees(180))
            ),
            new WaitCommand(.5),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(0.960, 6.008, Rotation2d.fromDegrees(-159.829)),                    
                    new Pose2d(2.448, 3.550, Rotation2d.fromDegrees(-83.367)),
                    new Pose2d(0.791, 1.053, Rotation2d.fromDegrees(92.651)),
                    new Pose2d(1.076, 3.162, Rotation2d.fromDegrees(95.631))
                ), new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                new GoalEndState(0, Rotation2d.fromDegrees(180))
                )
        );

        Command rightOutpostDepotClimbLAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(3.529, 0.685, Rotation2d.fromDegrees(-176.6)));
                //startSnakeDrive();
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(3.529, 0.685, Rotation2d.fromDegrees(-176.6)), 
                    new Pose2d(0.653, 0.685, Rotation2d.fromDegrees(127.694))
                ),
                new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                new GoalEndState(0, Rotation2d.fromDegrees(0))
            ),
            new WaitCommand(.5), //wait at outpost
            new AutoDriveCommand(
                List.of(
                    new Pose2d(0.653, 0.685, Rotation2d.fromDegrees(127.694)),
                    new Pose2d(2.003, 4.272, Rotation2d.fromDegrees(104.342)),
                    new Pose2d(1.107, 5.890, Rotation2d.fromDegrees(-89.012)), 
                    new Pose2d(1.107, 4.839, Rotation2d.fromDegrees(-90.000))
                ),
                new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),                
                new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                new GoalEndState(0, Rotation2d.fromDegrees(0))
        ));

        Command rightMidOutpostClimbRAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(4.529,0.623,Rotation2d.fromDegrees(-3.691)));
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(4.529, 0.623, Rotation2d.fromDegrees(-3.691)),
                    new Pose2d(7.497, 1.170, Rotation2d.fromDegrees(60.396)),
                    new Pose2d(7.641, 6.890, Rotation2d.fromDegrees(110.851)),
                    new Pose2d(7.497, 1.170, Rotation2d.fromDegrees(-124.743)),
                    new Pose2d(0.684, 0.623, Rotation2d.fromDegrees(147.995))
                ),
                new PathConstraints(1,1,Math.PI,Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                new GoalEndState(0, Rotation2d.fromDegrees(0))
            ),
            new WaitCommand(0.5),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(0.684, 0.623, Rotation2d.fromDegrees(147.995)),
                    new Pose2d(1.045, 2.664, Rotation2d.fromDegrees(57.933))
                ),
                new PathConstraints(1,1,Math.PI,Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                new GoalEndState(0, Rotation2d.fromDegrees(0))
            )
        );

        Command leftMidClimbRAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(4.467, 7.416, Rotation2d.fromDegrees(16.39)));
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(4.467, 7.416, Rotation2d.fromDegrees(16.39)),
                    new Pose2d(7.610, 6.220, Rotation2d.fromDegrees(-80.287)),
                    new Pose2d(7.610, 0.922, Rotation2d.fromDegrees(-94.362)),
                    new Pose2d(3.044, 0.706, Rotation2d.fromDegrees(174.071)),
                    new Pose2d(1.055, 2.613, Rotation2d.fromDegrees(83.501))
                ),
                new PathConstraints(1,1,Math.PI,Math.PI),
                new IdealStartingState(0,Rotation2d.fromDegrees(0)),
                new GoalEndState(0, Rotation2d.fromDegrees(0))
            )
        );

        Command rightMidClimbRAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(4.529, 0.623, Rotation2d.fromDegrees(-3.691)));
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(4.529, 0.623, Rotation2d.fromDegrees(-3.691)),
                    new Pose2d(7.497, 1.170, Rotation2d.fromDegrees(60.396)),
                    new Pose2d(7.641, 6.890, Rotation2d.fromDegrees(110.851)),
                    new Pose2d(7.497, 1.170, Rotation2d.fromDegrees(-124.743)),
                    new Pose2d(1.066, 2.613, Rotation2d.fromDegrees(90))                     
                ),
                new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                new GoalEndState(0, Rotation2d.fromDegrees(180))
            )
        );

        Command rightMidDepotClimbLAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(4.477, 0.634, Rotation2d.fromDegrees(-8.616)));
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(4.477, 0.634, Rotation2d.fromDegrees(-8.616)),
                    new Pose2d(7.796, 1.427, Rotation2d.fromDegrees(89.310)),
                    new Pose2d(7.631, 6.589, Rotation2d.fromDegrees(111.218)),
                    new Pose2d(1.107, 5.911, Rotation2d.fromDegrees(-94.764)),
                    new Pose2d(1.107, 4.808, Rotation2d.fromDegrees(-87.493))
                ),
                new PathConstraints(1,1,Math.PI,Math.PI),
                new IdealStartingState(0,Rotation2d.fromDegrees(0)),
                new GoalEndState(0, Rotation2d.fromDegrees(0))
            )
        );
        
        autoChooser.setDefaultOption("Left/Depot/Outpost/ClimbR", leftDepotOutpostClimbRAuto);
        autoChooser.setDefaultOption("Right/Outpost/Depot/ClimbL", rightOutpostDepotClimbLAuto);
        autoChooser.setDefaultOption("Right/Mid/Outpost/ClimbR", rightMidOutpostClimbRAuto);
        autoChooser.setDefaultOption("Left/Mid/ClimbR", leftMidClimbRAuto);
        autoChooser.setDefaultOption("Right/Mid/ClimbR", rightMidClimbRAuto);
        autoChooser.setDefaultOption("Right/Mid/Depot/ClimbL", rightMidDepotClimbLAuto);
    
        SmartDashboard.putData("Auto Routines", autoChooser);

        autoStartPosition = new SendableChooser<>();
        autoStartPosition.setDefaultOption("1anywhere", 0.0);

        SmartDashboard.putData("Auto Starting Position", autoStartPosition);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}