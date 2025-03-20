// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorArmProfiles;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoController;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.vision.PoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public static final ArmSubsystem m_arm = new ArmSubsystem();
  public static final PoseEstimator pose_estimator = new PoseEstimator();
  private final AutoController a_auto = new AutoController(m_robotDrive, () -> pose_estimator.getEstimatedPose2D());
  private final ScoreCommand g_score = new ScoreCommand(m_elevator, m_arm);
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static XboxController m_SysController = new XboxController(OIConstants.kSystemControllerPort);

  public void Intake() {

  }



  public void scheduleIntakeCommand() {
    // new RunCommand(() -> Intake(), m_arm).schedule();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    a_auto.loadAutos();
    a_auto.registerCommands(g_score);

    // m_elevator.setDefaultCommand(new RunCommand(() ->
    // m_elevator.set(m_SysController.getLeftY()), m_elevator));
    // m_arm.setDefaultCommand(new RunCommand(() -> Intake(), m_arm));
    // m_arm.setDefaultCommand(new RunCommand(() ->
    // m_arm.set(m_driverController.getRightY()), m_arm));

    m_arm.setDefaultCommand(new RunCommand(() -> {
      if (m_SysController.getRightBumperButtonPressed()) {
        //set to current score position
          m_arm.setTarget(g_score.currentProfile.score);
          //schedule release when at target
          g_score.getReleaseCommand().schedule();
      }
  }, m_arm));


    // Configure default commands
    m_robotDrive.setDefaultCommand( // default command similar to a loop system
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // new JoystickButton(m_SysController, XboxController.Axis.kLeftTrigger.value)
    // .onTrue(new RunCommand(() -> m_arm.intake(), m_elevator))
    // .onFalse(new RunCommand(() -> m_arm.stopIntake(), m_elevator));

    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_elevator.increment(), m_elevator));
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> m_elevator.decrement(), m_elevator));


    //Release Command moves arm down & Spits out coral 
    // new Trigger(() -> m_SysController.getRightBumperButtonPressed()).onTrue(g_score.getReleaseCommand());

    // new Trigger(() -> m_SysController.getRightBumperButtonPressed())
    // .onTrue(new InstantCommand(() -> g_score.getReleaseCommand().schedule()));



    new JoystickButton(m_SysController, XboxController.Button.kY.value)
    .onTrue(g_score.getScoreCommand(ElevatorArmProfiles.kLevel1, "L1"));

    new JoystickButton(m_SysController, XboxController.Button.kX.value)
    .onTrue(g_score.getScoreCommand(ElevatorArmProfiles.kLevel2, "L2"));

    new JoystickButton(m_SysController, XboxController.Button.kB.value)
    .onTrue(g_score.getScoreCommand(ElevatorArmProfiles.kLevel3, "L3"));

    new JoystickButton(m_SysController, XboxController.Button.kA.value)
    .onTrue(g_score.getScoreCommand(ElevatorArmProfiles.kLevel4, "L4"));

    new JoystickButton(m_SysController, XboxController.Button.kLeftBumper.value)
    .onTrue(g_score.getScoreCommand(ElevatorArmProfiles.kPositionCollect, "Rest"));


    //if this doesnt work use control command below this one
    new JoystickButton(m_SysController, XboxController.Button.kStart.value)
    .onTrue(g_score.getScoreCommand(ElevatorArmProfiles.kStore, "reset height"));

    // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    // .onTrue(g_score.getScoreCommand(ElevatorArmProfiles.kStore, "reset height"));

  }

  public Command followPath() {
    try {
      Pose2d startingPose2d = new Pose2d(7.456, 3.953, Rotation2d.fromDegrees(0));
      PathPlannerPath centerPath = PathPlannerPath.fromPathFile("center-score");
      m_robotDrive.resetOdometry(startingPose2d);
      return Commands.sequence(AutoBuilder.followPath(centerPath));
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("auto-1-right");
  }
}
