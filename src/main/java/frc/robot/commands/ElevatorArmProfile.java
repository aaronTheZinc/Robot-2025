package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorArmState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorArmProfile {
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;

    public ElevatorArmProfile(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.m_arm = arm;
        this.m_elevator = elevator;
    };

    public Command syncElevatorArm(ElevatorArmState state) {
        Command elevatorPositionCommand = m_elevator.getSetElevatorPositionCommand(state.elevator);
        Command armPositionCommand = m_arm.getSetArmPositionCommand(state.arm);
        
        elevatorPositionCommand.setName("Elevator Positon");
        armPositionCommand.setName("Arm Position");
        
        return Commands.parallel( armPositionCommand, elevatorPositionCommand);
    }


    public Command sequenceElevatorArm(ElevatorArmState state) {
            Command elevatorPositionCommand = m_elevator.getSetElevatorPositionCommand(state.elevator);
            Command armPositionCommand = m_arm.getSetArmPositionCommand(state.arm);
            
            elevatorPositionCommand.setName("Elevator Positon");
            armPositionCommand.setName("Arm Position");
            
            return Commands.sequence(elevatorPositionCommand, armPositionCommand);
    }



    // public Command score() {
    //     return Commands.sequence(null)
    // }
}
