package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax arm_motor;
    private final SparkMax intake_motor_1;
    private final SparkMax intake_motor_2;

    private final AbsoluteEncoder arm_encoder;
    private final SparkClosedLoopController arm_ClosedLoopController;
    private double accum = 0;
    private double currentPosition;
    private boolean lockIntakeCommand = false;

    private double targetPosition;

    public ArmSubsystem() {
        arm_motor = new SparkMax(20, MotorType.kBrushless);
        intake_motor_1 = new SparkMax(30, MotorType.kBrushless);
        intake_motor_2 = new SparkMax(31, MotorType.kBrushless);
        arm_ClosedLoopController = arm_motor.getClosedLoopController();

        arm_motor.configure(Configs.Arm.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        arm_encoder = arm_motor.getAbsoluteEncoder();
    };

    public boolean hasGamePiece() {
        double current = (intake_motor_1.getOutputCurrent() + intake_motor_2.getOutputCurrent()) / 2;
        SmartDashboard.putNumber("ArmSubsystem/IntakeCurrent", current);
        return current >= ArmConstants.kMaxCurrentIntake;
    }


    public void setTarget(double input) {
        targetPosition = input;
    }

    public Command getSetArmPositionCommand(double input) {
        return Commands.runOnce(() -> setTarget(input))
                .until(() -> this.atTarget());
    }

    public Command getCollectGamePieceCommand() {
        return Commands.sequence(
                getSetArmPositionCommand(ArmConstants.kPositionCollect),
                Commands.runOnce(() -> this.intake()).until(() -> hasGamePiece()));
    }

    public double geCurrentPositon() {
        return arm_encoder.getPosition();
    }

    public void stopIntake() {
        intake_motor_1.stopMotor();
        intake_motor_2.stopMotor();
    }

    public void intake() {
        intake_motor_1.set(-1 * 0.3);
        intake_motor_2.set(0.3);
    }

    public void spitOut() {
        intake_motor_2.set(-1 * 0.25);
        intake_motor_1.set(0.25);
    }

    public Boolean atTarget() {
        double error = targetPosition - Rotation2d.fromRadians(currentPosition).getDegrees();
        return Math.abs(error) < ArmConstants.kMaxTargetOffset;
    }

    public void set(double s) {
        arm_motor.set(s * 0.5);
        SmartDashboard.putNumber("ArmSubsytem/speed", s);
    };

    public void increment() {
        targetPosition++;
    };

    public void decrement() {
        targetPosition--;
    }

    public Command getReleaseCommand() {
        return Commands.sequence(Commands.runOnce(() -> lockIntakeCommand = true), Commands.runOnce(() -> spitOut()), new WaitCommand(2), Commands.runOnce(() -> stopIntake()),Commands.runOnce(() -> lockIntakeCommand = false));
    }

    @Override
    public void periodic() {
        currentPosition = arm_encoder.getPosition();

        double error = targetPosition - currentPosition;
        SmartDashboard.putBoolean("ArmSubsystem/Ready", atTarget());
        SmartDashboard.putNumber("ArmSubsystem/TargetPosition", targetPosition);
        SmartDashboard.putNumber("ArmSubsystem/Position", Rotation2d.fromRadians(currentPosition).getDegrees());
        SmartDashboard.putNumber("ArmSubsystem/Error", error);
        SmartDashboard.putBoolean("ArmSubsystem/Has_Coral", hasGamePiece());

        if(!lockIntakeCommand) {
        if (RobotContainer.m_SysController.getLeftTriggerAxis() > 0) {
            intake();
        } else if (RobotContainer.m_SysController.getRightTriggerAxis() == 0) {
            stopIntake();
        }

        if (RobotContainer.m_SysController.getRightTriggerAxis() > 0) {
            spitOut();
        }
    }
        if(targetPosition > 180) {
            targetPosition = 0;
        }
         arm_ClosedLoopController.setReference(Rotation2d.fromDegrees(targetPosition).getRadians(),
        ControlType.kPosition);

        // if(!atTarget()) {
        // currentPosition += MathUtil.clamp(error / 100, -0.8, 0.8);
        // }
        //arm_ClosedLoopController.setReference(targetPosition, ControlType.kPosition);

        // double pos = arm_encoder.getPosition(); //double means float

        // double speed = MathUtil.clamp(pid.calculate(pos, targetPosition), -0.15,
        // 0.15);

        // SmartDashboard.putNumber("[ARM POSITION]", pos);

        // arm_motor.set(speed);
    }
}
