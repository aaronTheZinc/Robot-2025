// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose.
 * All constants should be declared globally (i.e. public static). Do not put
 * anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot,
    // rather the allowed maximum speeds.
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5); // Distance between centers of right and left
                                                                         // wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5); // Distance between front and back wheels on
                                                                        // robot

    // Swerve Drive Kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class ElevatorConstants {
    public static final double kPositionCollect = -18;
    public static final double kLevel1 = -4.96;
    public static final double kLevel2 = -5.01;
    public static final double kLevel3 = -12.0;
    public static final double kLevel4 = -76;
    public static final double kStore = -24;

    public static final double kMaxTargetOffset = 0.1;
    public static final double elevatorWinchDiameter = 1.65;
    public static final double elevatorGearRatio = 9;
    public static final double kEncoderConversionFactor = Math.PI * (elevatorWinchDiameter) / elevatorGearRatio;

  }

  public static final class ArmConstants {
    public static final double kPositionCollect = 184;
    public static final double kMaxTargetOffset = 0.05;

    public static final double kLevel1 = 89;
    public static final double kLevel2 = 44.6;
    public static final double kLevel3 = 52.0;
    public static final double kLevel4 = 42.5;

    public static final double armHeightMeters = Units.inchesToMeters(60);
    public static final double armGearRatio = 9;
    public static final double kEncoderConversionFactor = armHeightMeters / armGearRatio;

    public static final double kMaxCurrentIntake = 10;
    // 90
    // 66
    // 55.8

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSystemControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  // Define a static inner class for ElevatorArmProfiles
  public static final class ElevatorArmProfiles {
    // Declare the "struct-like" constants (immutable instances)
    public static final ElevatorArmState kPositionCollect = new ElevatorArmState(ElevatorConstants.kPositionCollect,
        ArmConstants.kPositionCollect, ArmConstants.kPositionCollect);
    public static final ElevatorArmState kLevel1 = new ElevatorArmState(ElevatorConstants.kLevel1, ArmConstants.kLevel1,
        90);
    public static final ElevatorArmState kLevel2 = new ElevatorArmState(ElevatorConstants.kLevel2, ArmConstants.kLevel2,
        55.8);
    public static final ElevatorArmState kLevel3 = new ElevatorArmState(ElevatorConstants.kLevel3, ArmConstants.kLevel3,
        66);
    public static final ElevatorArmState kLevel4 = new ElevatorArmState(ElevatorConstants.kLevel4, ArmConstants.kLevel4,
        90);
     public static final ElevatorArmState kStore = new ElevatorArmState(ElevatorConstants.kStore, ArmConstants.kPositionCollect,
        ArmConstants.kPositionCollect);

  }

  // Define the ElevatorArmState class
  public static class ElevatorArmState {
    public  double elevator;
    public  double arm;
    public  double score;

    // Constructor to initialize the fields
    public ElevatorArmState(double elevator, double arm, double score) {
      this.elevator = elevator;
      this.arm = arm;
      this.score = score;
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionConstants {
    public static final String kLimelightFront = "limelight";
    public static final String kLimelightBack = "limelight-back";

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
