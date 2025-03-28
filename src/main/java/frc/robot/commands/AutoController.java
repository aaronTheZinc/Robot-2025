package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorArmProfiles;
import frc.robot.subsystems.DriveSubsystem;

public class AutoController {
    private final DriveSubsystem m_chassis;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private String m_autoSelected;
    private final ScoreCommand g_score;
    // private Supplier<Pose2d> getEstimatedPose;

    public AutoController(DriveSubsystem chassis, Supplier<Pose2d> poseSupplier, ScoreCommand score) {
        this.g_score = score;

        this.m_chassis = chassis;
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            // AutoBuilder.configure(
            //         RobotContainer.pose_estimator::getEstimatedPose2D, // Robot pose supplier
            //         this.m_chassis::resetOdometry, // Method to reset odometry (will be called if your auto has a
            //                                        // starting pose)
            //         this.m_chassis::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            //         (speeds, feedforwards) -> this.m_chassis.driveRobotRelative(speeds),
            //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
            //                                         // for holonomic drive trains
            //                 new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
            //                 new PIDConstants(2.0, 0.0, 0.0) // Rotation PID constants
            //         ),

            //         config, // The robot configuration
            //         () -> {

            //             var alliance = DriverStation.getAlliance();
            //             if (alliance.isPresent()) {
            //                 return alliance.get() == DriverStation.Alliance.Red;
            //             }
            //             return false;
            //         },
            //         this.m_chassis // Reference to this subsystem to set requirements
            // );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last

    }

    public void registerCommands(ScoreCommand g_score) {
        List<Pair<String, Command>> commands = new ArrayList<>();

        commands.add(new Pair<>("score_position_collect",
                g_score.getScoreCommand(ElevatorArmProfiles.kPositionCollect, "Collect")));
        commands.add(new Pair<>("score_level_1", g_score.getScoreCommand(ElevatorArmProfiles.kLevel1, "L1")));
        commands.add(new Pair<>("score_level_2", g_score.getScoreCommand(ElevatorArmProfiles.kLevel2, "L2")));
        commands.add(new Pair<>("score_level_3", g_score.getScoreCommand(ElevatorArmProfiles.kLevel3, "L3")));
        commands.add(new Pair<>("score_level_4", g_score.getScoreCommand(ElevatorArmProfiles.kLevel4, "L4")));

        NamedCommands.registerCommands(commands);
    };

    public void loadAutos() {
        List<String> autoNames = AutoBuilder.getAllAutoNames();
        System.out.println("[Initializing Autos]");
        for (int i = 0; i < autoNames.size(); i++) {
            System.out.println("[Initializing Auto] " + autoNames.get(i));
            m_chooser.addOption(autoNames.get(i), autoNames.get(i));
        }
        ;

        SmartDashboard.putData("Auto choices", m_chooser);

    }

    public String getSelectedAuto() {
        m_autoSelected = m_chooser.getSelected();
        return m_autoSelected;
    };

    public Command getScoreLevel4() {

        return Commands.sequence(
            new InstantCommand(() -> RobotContainer.m_robotDrive.resetOdometry(new Pose2d()), RobotContainer.m_robotDrive),
               g_score.getScoreCommand(Constants.ElevatorArmProfiles.kLevel4, "l4 auto"),
            getDriveL4(),
            g_score.getReleaseCommand(true));
        
    };

    public Command get2CoralAuto() {
        return Commands.sequence(
            RobotContainer.m_alignment.getFollowPathCommand("start-path-right", Rotation2d.fromDegrees(100)),
            new InstantCommand(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, false), RobotContainer.m_robotDrive),
            new WaitCommand(3),
            RobotContainer.m_alignment.getFollowPathCommand("spr-to-feeder", Rotation2d.fromDegrees(45)),
            new InstantCommand(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, false), RobotContainer.m_robotDrive)
        
        );
    };

    public Command getDriveL4() {
        return new RunCommand(() -> {
            RobotContainer.m_robotDrive.drive(0.3, 0, 0, false);
        } , RobotContainer.m_robotDrive).until(() -> {
            SmartDashboard.putNumber("Auto/L4", RobotContainer.m_robotDrive.getPose().getX());
           return RobotContainer.m_robotDrive.getPose().getX() > 1.619;
        });
    }

}
