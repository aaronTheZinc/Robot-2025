package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevator_motor;
    private final SparkClosedLoopController elevator_controller;
    private final RelativeEncoder elevator_encoder;
    private double targetPosition;
    private double currentPosition;

    public ElevatorSubsystem() {
        elevator_motor = new SparkMax(40, MotorType.kBrushless);     
        elevator_motor.configure(Configs.Elevator.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator_encoder = elevator_motor.getEncoder();

        elevator_controller = elevator_motor.getClosedLoopController();
        elevator_encoder.setPosition(0);
    }

    private void setTarget(double input) {
        targetPosition = input;
    }

    public Command getSetElevatorPositionCommand(double input) {
        return Commands.runOnce(() -> setTarget(input))
        .until(() -> this.atTarget());
    }

    private Boolean atTarget() {
        double error = targetPosition - currentPosition;
        return Math.abs(error) < ElevatorConstants.kMaxTargetOffset;
    }

    public void set(double s) {
        elevator_motor.set(s);
    }

    public void increment() {
        targetPosition++;
    };

    public void decrement() {
        targetPosition--;
    }

    

    @Override
    public void periodic() {

        currentPosition = elevator_encoder.getPosition();

        SmartDashboard.putBoolean("ElevatorSubsystem/Ready", atTarget());
        SmartDashboard.putNumber("ElevatorSubsystem/Target",targetPosition);
        SmartDashboard.putNumber("ElevatorSubsystem/Current",currentPosition);

        double error = targetPosition - currentPosition;
        SmartDashboard.putNumber("ElevatorSubsystem/error", error);

        elevator_controller.setReference(targetPosition, ControlType.kPosition);

        if(!atTarget()) {
            // currentPosition += MathUtil.clamp(error / 100, -0.8, 0.8);
        }

        
       



        // double pos = elevator_encoder.getPosition(); //double means float

        // double speed = MathUtil.clamp(pid.calculate(pos, targetPosition), -1, 1);

        // elevator_motor.set(speed);
    }
}