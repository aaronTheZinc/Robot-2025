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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator pose_estimator;
    private final SwerveDrivePoseEstimator relative_estimator;
    private Alliance current_aliance;
    private double lastDetectedTagID = -1;


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
        this.relative_estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, intialRotation2D,
        initialSwerveModulePositions, initialPose2D);       

        Optional<Alliance> team = DriverStation.getAlliance();
        if (team.isEmpty()) {
            return;
        };

     
        current_aliance = team.get();
    }

    public Pose2d getTeamBotPose(String limelightName) {

        if (current_aliance == Alliance.Blue) {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).pose;
        } else {
            return LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName).pose;
        }
    }

    public Pose2d getTargetedTagPose(String limelightName) {
        Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
        return pose.toPose2d();
    }

    public double getDegrees360(double angle) {
        double degrees = 0;
        if(angle < 0) {
          degrees = 360 - Math.abs(angle);
        } else {
            degrees = angle;
        }

        return degrees;
    }

    private void log() {
        Pose2d tagRelative = getTagRelativePose();
        Pose2d fieldPose = pose_estimator.getEstimatedPosition();
        SmartDashboard.putNumber("PoseEstimator/TagRelative/X", tagRelative.getX());
        SmartDashboard.putNumber("PoseEstimator/TagRelative/Y", tagRelative.getY());
        SmartDashboard.putNumber("PoseEstimator/TagRelative/Degrees", tagRelative.getRotation().getDegrees());
        SmartDashboard.putNumber("PoseEstimator/LastDetectedTagID", lastDetectedTagID);
        SmartDashboard.putBoolean("PoseEstimator/TagDetected", LimelightHelpers.getTV(VisionConstants.kLimelightFront));

        SmartDashboard.putNumber("PoseEstimator/Field/X", fieldPose.getX());
        SmartDashboard.putNumber("PoseEstimator/Field/Y", fieldPose.getY());
        SmartDashboard.putNumber("PoseEstimator/Field/R", fieldPose.getRotation().getDegrees());
        SmartDashboard.putNumber("PoseEstimator/Field/R360", fieldPose.getRotation().getDegrees() % 360);
    }


    

    @Override
    public void periodic() {
        double time = getTime();

        Rotation2d chassisRotation = Rotation2d.fromDegrees(RobotContainer.m_robotDrive.getHeading());
        SmartDashboard.putNumber("PoseEstimator/Chassis Rotation: ", chassisRotation.getDegrees());
        SmartDashboard.putNumber("PoseEstimator/Estimated Rotation: ", pose_estimator.getEstimatedPosition().getRotation().getDegrees());
        // add chassis measurement
        this.pose_estimator.update(chassisRotation, RobotContainer.m_robotDrive.getModulePositions());
        this.relative_estimator.update(Rotation2d.fromDegrees(RobotContainer.m_robotDrive.getHeading()), RobotContainer.m_robotDrive.getModulePositions());
        
        log();

        Boolean frontHasTarget =  LimelightHelpers.getTV(VisionConstants.kLimelightFront);
     
        if(frontHasTarget) {
            Pose2d fieldRelativePose = getTeamBotPose(VisionConstants.kLimelightFront);
            Pose2d tagRelativePose = getTargetedTagPose(VisionConstants.kLimelightFront);
            double detectedID = LimelightHelpers.getFiducialID(VisionConstants.kLimelightFront);

            // if(lastDetectedTagID != detectedID) {
            //     relative_estimator.resetPose(new Pose2d());
            // }
            lastDetectedTagID = detectedID;
            pose_estimator.addVisionMeasurement(fieldRelativePose, time);
            //ignore detected tags on other side
            if(Math.abs(tagRelativePose.getRotation().getDegrees()) > 20) return;
            relative_estimator.addVisionMeasurement(tagRelativePose, time); 
        }
    };

    public Pose2d getTagRelativePose() {
        return relative_estimator.getEstimatedPosition();
    }


    public Pose2d getEstimatedPose2D() {
        return pose_estimator.getEstimatedPosition();
    }

    private static Double getTime() {
        return Timer.getFPGATimestamp();
    }
}
