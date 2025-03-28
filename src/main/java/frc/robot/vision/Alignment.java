package frc.robot.vision;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Alignment extends SubsystemBase {
private final SimpleMotorFeedforward translationXFF = new SimpleMotorFeedforward(0.1, 0.2, 0.01);
private final SimpleMotorFeedforward translationYFF = new SimpleMotorFeedforward(0.1, 0.2, 0.01);
private final SimpleMotorFeedforward translationThetaFF = new SimpleMotorFeedforward(0.1, 0.2, 0.01);

  PIDController translationXPID = new PIDController(VisionConstants.kTranslationAlignmentP, 0, 0.05);
  PIDController translationYPID = new PIDController(VisionConstants.kTranslationAlignmentP, 0, 0.05);
  PIDController thetaPID = new PIDController(VisionConstants.kThetaAlignmentP, 0, 0);


  private Pose2d simulatedPose = new Pose2d(7.322, 2.611, Rotation2d.fromDegrees(180));
  private Pose2d targetedTagPose = new Pose2d();
  private Pose2d poseOffsetRight = new Pose2d();
  private Pose2d poseOffsetLeft = new Pose2d();
  private Pose2d poseOffsetCenter = new Pose2d();
  private boolean autoBypass = false;
  public boolean lockChassisControls = false;

  private double last_timeStamp = 0;

  // target pose to reach
  private Pose2d targetPose = new Pose2d();

  public Alignment() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // double[] cameraPose = { 0.178, -0.127, 0.267, 0.0, 0.0, 0.0 };
    // table.getEntry("camerapose_robotspace").setDoubleArray(cameraPose);
  }

  public void setSimulatedPose(Pose2d pose) {
    simulatedPose = pose;
  }
  public Pose2d getSimulatedPose() {
    return simulatedPose;
  }

  public void setAutoBypass(boolean b) {
    autoBypass = b;
  }

  public boolean inAlignmentRange() {
    boolean xInRange = Math.abs(targetedTagPose.getX() - targetPose.getX()) < 0.2;
    boolean yInRange = Math.abs(targetedTagPose.getX() - targetPose.getX()) < 0.2;

    return xInRange && yInRange;
  };

  public void setTagRelativePose(Pose2d pose) {
    targetPose = pose;
  }

  public double calculateRotationSpeed(double currentAngle, double targetAngle) {
    double error = targetAngle - currentAngle;
    error = Math.IEEEremainder(error, 2 * Math.PI); // Normalize to [-π, π]
    
    return thetaPID.calculate(currentAngle, targetAngle);
}

public void resetTime() {
    last_timeStamp = Timer.getFPGATimestamp();
}

private double applyDeadband(double output, double min) {
    if (Math.abs(output) < min) {
        return Math.signum(output) * min; // Maintain direction but apply minimum speed
    }
    return output;
}

public List<Pose2d> applyFinalRotation(List<Pose2d> trajectory, double degrees, double effectLastPoses) {
  for(int i = trajectory.size(); i > trajectory.size() - 1 - effectLastPoses; i--) {
   Pose2d pose = trajectory.get(i);
   trajectory.set(i, new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(degrees)));
  }

return trajectory;
}

public Command getFollowPathCommand(String pathName, Rotation2d endRotation) {
    try {

        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        List<Pose2d> trajectoryPoses = path.getPathPoses();

        if (trajectoryPoses.isEmpty()) {
            DriverStation.reportError("PathPlanner path is empty!", false);
            return Commands.none();
        }

        // Create a sequential command group to execute movement through each waypoint
        Command sequence = new SequentialCommandGroup(
           trajectoryPoses.stream()
                .map(pose -> getAlignToWaypointCommand(
                  new Pose2d(pose.getX(), pose.getY(), endRotation)
                  , () -> simulatedPose))
                .toArray(Command[]::new)
        );

        // Ensure the robot stops at the end
        return sequence;
        
    } catch (Exception e) {
        SmartDashboard.putString("ERROR", "Big oops: " + e.getMessage());
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
}

public Command getAlignToWaypointCommand(Pose2d targetPose, Supplier<Pose2d> poseSupplier) {
    return new FunctionalCommand(
        () -> last_timeStamp = Timer.getFPGATimestamp(), // Init
        () -> { // Execute
            double now = Timer.getFPGATimestamp();
            double dt = now - last_timeStamp;
            last_timeStamp = now;

            Pose2d currentPose = poseSupplier.get();

            // PID Correction
            double xCorrection = translationXPID.calculate(currentPose.getX(), targetPose.getX());
            double yCorrection = translationYPID.calculate(currentPose.getY(), targetPose.getY());
            double thetaCorrection = thetaPID.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            // Feedforward Velocity Calculation
            double xVelocity = (targetPose.getX() - currentPose.getX()) / dt;
            double yVelocity = (targetPose.getY() - currentPose.getY()) / dt;
            double thetaVelocity = (targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()) / dt;

            double xFF = translationXFF.calculate(xVelocity);
            double yFF = translationYFF.calculate(yVelocity);
            double thetaFF = translationThetaFF.calculate(thetaVelocity);

            RobotContainer.m_robotDrive.drive(MathUtil.clamp(xCorrection, -0.4, 0.4), MathUtil.clamp(yCorrection, -0.4, 0.4), 0, false);


            // Apply motion with deadband
            simulatedPose = new Pose2d(
                -(simulatedPose.getX() + (xCorrection + xFF) * dt),
                -(simulatedPose.getY() + (yCorrection + yFF)) * dt,
                Rotation2d.fromDegrees(simulatedPose.getRotation().getRadians() + thetaCorrection * dt)
            );
        },
        (interrupted) -> {}, // End
        () -> { // Condition to stop
            Pose2d currentPose = poseSupplier.get();
            return Math.abs(targetPose.getX() - currentPose.getX()) < AutoConstants.kMaxTranslationError &&
                   Math.abs(targetPose.getY() - currentPose.getY()) < AutoConstants.kMaxTranslationError;
        },
        this // Subsystem requirement
    );
}

  @Override
  public void periodic() {
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // double[] cameraPose = {0.178, -0.127, 0.267, 0.0, 0.0, 0.0};
    // table.getEntry("camerapose_robotspace").setDoubleArray(cameraPose);
    // get pose relative to robot so we can minimize coordinates
    targetedTagPose = RobotContainer.pose_estimator.getTagRelativePose();

    SmartDashboard.putNumber("Alignment/Relative_X", targetedTagPose.getX());
    SmartDashboard.putNumber("Alignment/Relative_Y", targetedTagPose.getY());
    SmartDashboard.putNumber("Alignment/Relative_Rotation", targetedTagPose.getRotation().getDegrees());

    boolean lbActive = RobotContainer.m_driverController.getLeftBumperButton();
    boolean rbActive = RobotContainer.m_driverController.getRightBumperButton();
    boolean startBtnActive = RobotContainer.m_driverController.getStartButton();

    // bind controls that apply relative offsets to targeted tag

    if (lbActive) {
      poseOffsetLeft = VisionConstants.kScoreL4;
      targetPose = poseOffsetLeft;
    }

    if (rbActive) {
      poseOffsetRight = VisionConstants.kScoreAuto2;
      targetPose = poseOffsetRight;
    }

    // minimize distances to offset angles defined in vision constants
    if (lbActive || rbActive || autoBypass) {
      targetedTagPose = RobotContainer.pose_estimator.getEstimatedPose2D();

      lockChassisControls = true;
      double xSpeed = translationXPID.calculate(targetedTagPose.getX(), targetPose.getX());
      double ySpeed = translationYPID.calculate(targetedTagPose.getY(), targetPose.getY());
      double thetaSpeed = thetaPID.calculate(targetedTagPose.getRotation().getRadians(),
          targetPose.getRotation().getRadians());
      if (Math.abs(targetedTagPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) > 30) {
        xSpeed = 0;
        ySpeed = 0;
      }
      RobotContainer.m_robotDrive.drive(xSpeed, ySpeed, thetaSpeed, true);
    } else {
      lockChassisControls = false;
    }

  }

}
