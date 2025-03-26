package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Alignment extends SubsystemBase {
  PIDController translationXPID = new PIDController(VisionConstants.kTranslationAlignmentP,0, 0.05);
  PIDController translationYPID = new PIDController(VisionConstants.kTranslationAlignmentP,0, 0.05);
  PIDController thetaPID = new PIDController(VisionConstants.kThetaAlignmentP,0, 0);

   private Pose2d targetedTagPose = new Pose2d(); 
   private Pose2d poseOffsetRight = new Pose2d(); 
   private Pose2d poseOffsetLeft = new Pose2d(); 
   private Pose2d poseOffsetCenter = new Pose2d(); 
   private boolean autoBypass = false;
   public boolean lockChassisControls = false;

   //target pose to reach
   private Pose2d targetPose = new Pose2d();
   public Alignment() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double[] cameraPose = {0.178, -0.127, 0.267, 0.0, 0.0, 0.0};
    table.getEntry("camerapose_robotspace").setDoubleArray(cameraPose);
   } 

   private Pose2d getPoseOffset(Pose2d pose1, Pose2d pose2) {
    double x1 = pose2.getX(); 
    double y1 = pose2.getY();
    double theta1 = pose2.getRotation().getDegrees();

     double x2 = pose2.getX(); 
     double y2 = pose2.getY();
     double theta2 = pose2.getRotation().getDegrees();

     

     return  new Pose2d(x1 + x2, y1 + y2, Rotation2d.fromDegrees(theta1 +theta2) );

   };

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

   @Override
   public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double[] cameraPose = {0.178, -0.127, 0.267, 0.0, 0.0, 0.0};
    table.getEntry("camerapose_robotspace").setDoubleArray(cameraPose);
    //get pose relative to robot so we can minimize coordinates
     targetedTagPose = RobotContainer.pose_estimator.getTagRelativePose();

     SmartDashboard.putNumber("Alignment/Relative_X", targetedTagPose.getX());
     SmartDashboard.putNumber("Alignment/Relative_Y", targetedTagPose.getY());
     SmartDashboard.putNumber("Alignment/Relative_Rotation", targetedTagPose.getRotation().getDegrees());


    boolean lbActive = RobotContainer.m_driverController.getLeftBumperButton();
    boolean rbActive = RobotContainer.m_driverController.getRightBumperButton();
    boolean startBtnActive = RobotContainer.m_driverController.getStartButton();

    //bind controls that apply relative offsets to targeted tag
        
        if(lbActive) {
        poseOffsetLeft = VisionConstants.kScoreL4;
        targetPose = poseOffsetLeft;
        }
        
        if(rbActive) {
        poseOffsetRight = VisionConstants.kScoreAuto2;
        targetPose = poseOffsetRight;
        }

    //minimize distances to offset angles defined in vision constants
    if(lbActive || rbActive || autoBypass) {
      targetedTagPose = RobotContainer.pose_estimator.getEstimatedPose2D();

        lockChassisControls = true;
        double xSpeed = translationXPID.calculate(targetedTagPose.getX(), targetPose.getX());
        double ySpeed = translationYPID.calculate(targetedTagPose.getY(), targetPose.getY());
        double thetaSpeed = thetaPID.calculate(targetedTagPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        if(Math.abs(targetedTagPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) > 30) {
          xSpeed = 0;
          ySpeed = 0;
        }
        RobotContainer.m_robotDrive.drive(xSpeed, ySpeed, thetaSpeed, true);
    } else {
        lockChassisControls = false;
    }

   }

}
