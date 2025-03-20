package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator pose_estimator;
    private Alliance current_aliance;

    public PoseEstimator() {
        Rotation2d intialRotation2D = new Rotation2d();
        Pose2d initialPose2D = new Pose2d();
        SwerveModulePosition[] initialSwerveModulePositions = new SwerveModulePosition[4];

        initialSwerveModulePositions[0] = new SwerveModulePosition();
        initialSwerveModulePositions[1] = new SwerveModulePosition();
        initialSwerveModulePositions[2] = new SwerveModulePosition();
        initialSwerveModulePositions[3] = new SwerveModulePosition();
        this.pose_estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, intialRotation2D,
                initialSwerveModulePositions, initialPose2D);

        Optional<Alliance> team = DriverStation.getAlliance();
        if (team.isEmpty()) {
            return;
        };

        current_aliance = team.get();
    }

    public Pose2d getTeamBotPose(String limelightName) {
        if (current_aliance == Alliance.Blue) {
            return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
        } else {
            return LimelightHelpers.getBotPose2d_wpiRed(limelightName);
        }
    }

    public Pose2d getTargetedTagPose(String limelightName) {
        Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
        return pose.toPose2d();
    }


    

    @Override
    public void periodic() {
        double time = getTime();
        
        // add chassis measurement
        // this.pose_estimator.update(Rotation2d.fromDegrees(RobotContainer.m_robotDrive.getHeading()), RobotContainer.m_robotDrive.getModulePositions());
        Pose2d averaged_vision_pose;
        Boolean frontHasTarget =  LimelightHelpers.getTV(VisionConstants.kLimelightFront);
        Boolean backHasTarget =  LimelightHelpers.getTV(VisionConstants.kLimelightFront);

        if(frontHasTarget) {
            pose_estimator.addVisionMeasurement(getTeamBotPose(VisionConstants.kLimelightFront), time);
        }

        if (frontHasTarget && backHasTarget) {
            Pose2d limelight_f = getTeamBotPose(VisionConstants.kLimelightFront);
            Pose2d limelight_b = getTeamBotPose(VisionConstants.kLimelightBack);

            averaged_vision_pose = new Pose2d(
                (limelight_f.getX() + limelight_b.getX()) / 2,
                (limelight_f.getY() + limelight_b.getY()) / 2,
                 Rotation2d.fromRadians((limelight_f.getRotation().getRadians() + limelight_b.getRotation().getRadians()) /2)
            );

            limelight_f.getX();
            limelight_f.getY();
        
        }else {
            averaged_vision_pose = getTeamBotPose(frontHasTarget ? VisionConstants.kLimelightFront : VisionConstants.kLimelightBack);
        };

        pose_estimator.addVisionMeasurement(averaged_vision_pose, time);




    }


    public Pose2d getEstimatedPose2D() {
        return pose_estimator.getEstimatedPosition();
    }

    private static Double getTime() {
        return Timer.getFPGATimestamp();
    }
}
