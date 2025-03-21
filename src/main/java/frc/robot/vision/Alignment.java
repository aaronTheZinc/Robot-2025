package frc.robot.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class Alignment extends SubsystemBase {
  PIDController translationPID = new PIDController(VisionConstants.kTranslationAlignmentP,0, 0);
  PIDController thetaPID = new PIDController(VisionConstants.kThetaAlignmentP,0, 0);

   private Pose2d targetedTagPose = new Pose2d(); 
   private Pose2d poseOffsetRight = new Pose2d(); 
   private Pose2d poseOffsetLeft = new Pose2d(); 
   private Pose2d poseOffsetCenter = new Pose2d(); 

   public boolean lockChassisControls = false;

   //target pose to reach
   private Pose2d targetPose = new Pose2d();
   public Alignment() {
    
   } 

   private Pose2d getPoseOffset(Pose2d pose1, Pose2d pose2) {
    double x1 = pose2.getX(); 
    double y1 = pose2.getY();
    double theta1 = pose2.getRotation().getDegrees();

     double x2 = pose2.getX(); 
     double y2 = pose2.getY();
     double theta2 = pose2.getRotation().getDegrees();

     

     return  new Pose2d(x1 + x2, y1 + y2, Rotation2d.fromDegrees(theta1 +theta2) );

   }

   @Override
   public void periodic() {
    //get pose relative to robot so we can minimize coordinates
     targetedTagPose = RobotContainer.pose_estimator.getTargetedTagPose(Constants.VisionConstants.kLimelightFront);

     SmartDashboard.putNumber("Alignment/Relative_X", targetedTagPose.getX());
     SmartDashboard.putNumber("Alignment/Relative_Y", targetedTagPose.getY());
     SmartDashboard.putNumber("Alignment/Relative_Rotation", targetedTagPose.getRotation().getDegrees());

    boolean lbActive = RobotContainer.m_driverController.getLeftBumperButton();
    boolean rbActive = RobotContainer.m_driverController.getRightBumperButton();
    boolean startBtnActive = RobotContainer.m_driverController.getStartButton();

    //bind controls that apply relative offsets to targeted tag
    if(startBtnActive) {
        poseOffsetCenter = VisionConstants.kCenterReefOffset;
        targetPose = poseOffsetCenter;
        }
        
        if(lbActive) {
        poseOffsetLeft = VisionConstants.kLeftReefOffset;
        targetPose = poseOffsetLeft;
        }
        
        if(rbActive) {
        poseOffsetRight = VisionConstants.kRightReefOffset;
        targetPose = poseOffsetRight;
        }

    //minimize distances to offset angles defined in vision constants
    if(lbActive || rbActive || startBtnActive) {
        lockChassisControls = true;
        double xSpeed = translationPID.calculate(targetedTagPose.getX(), targetPose.getX());
        double ySpeed = translationPID.calculate(targetedTagPose.getY(), targetPose.getY());
        double thetaSpeed = thetaPID.calculate(targetedTagPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        RobotContainer.m_robotDrive.drive(xSpeed, ySpeed, thetaSpeed, false);
    } else {
        lockChassisControls = false;
    }

   }

}
