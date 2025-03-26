package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorArmProfiles;
import frc.robot.Constants.ElevatorArmState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;
    public final ElevatorArmProfile m_effector;
    public ElevatorArmState currentProfile = new ElevatorArmState(0, 0, 90);
    public double scoreDeg = 0;
    public ScoreCommand(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.m_arm = arm;
        this.m_elevator = elevator;
        this.m_effector = new ElevatorArmProfile(elevator, arm);
    };

    public double getScoreDegrees() {
        return scoreDeg;
    }


    public Command getAlgae3() {
        return Commands.sequence(
            new InstantCommand(() -> m_arm.yeetThatHoe(), m_arm),
            m_effector.syncElevatorArm(Constants.ElevatorArmProfiles.kAlgae3)
            );
    }

    public Command getAlgae2() {
        return Commands.sequence(
            new InstantCommand(() -> m_arm.yeetThatHoe(), m_arm),
            m_effector.syncElevatorArm(Constants.ElevatorArmProfiles.kAlgae2)
            );
    }

    public Command getScoreCommand(ElevatorArmState profile, String name) {
        currentProfile.arm = profile.arm;
        currentProfile.elevator = profile.elevator;
        currentProfile.score = profile.score;
        scoreDeg = profile.score;

        SmartDashboard.putNumber("ScoreCommand/SocreDegrees", scoreDeg);
        // Command collectGamePieceCommand = m_arm.getCollectGamePieceCommand();
        Command moveEffectorCommand = m_effector.syncElevatorArm(profile); // move elevator & arm together;
        Command cmd = Commands.sequence(new InstantCommand(() -> m_arm.stopIntake(), m_arm),Commands.runOnce(() -> scoreDeg = profile.score),moveEffectorCommand);

        cmd.setName(name);
        SmartDashboard.putData(cmd);
        return cmd;
    };

    public Command getStoreCommand() {
        Command moveEffectorCommand = m_effector.sequenceElevatorArm(ElevatorArmProfiles.kStore); // move elevator & arm together;
        return Commands.sequence(moveEffectorCommand);
    }

    public Command getReleaseCommand(boolean reverseIntake) {
        Command releaseCommand = m_arm.getReleaseCommand(); // release coral

        return Commands.sequence(Commands.waitUntil(() -> m_arm.atTarget()), reverseIntake ? releaseCommand: Commands.none());
    }

        public Command getLevel1Command() {
        Command elevatorPositionCommand = m_elevator.getSetElevatorPositionCommand(Constants.ElevatorConstants.kLevel1);
        Command armPositionCommand = m_arm.getSetArmPositionCommand(Constants.ArmConstants.kLevel1);
        return Commands.sequence(armPositionCommand, elevatorPositionCommand);
    }

}
