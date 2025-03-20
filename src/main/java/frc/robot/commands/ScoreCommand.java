package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Command getScoreCommand(ElevatorArmState profile, String name) {
        currentProfile.arm = profile.arm;
        currentProfile.elevator = profile.elevator;
        currentProfile.score = profile.score;
        scoreDeg = profile.score;

        SmartDashboard.putNumber("ScoreCommand/SocreDegrees", scoreDeg);
        // Command collectGamePieceCommand = m_arm.getCollectGamePieceCommand();
        Command moveEffectorCommand = m_effector.syncElevatorArm(profile); // move elevator & arm together;
        Command cmd = Commands.sequence(moveEffectorCommand);

        cmd.setName(name);
        SmartDashboard.putData(cmd);
        return cmd;
    };

    public Command getReleaseCommand() {
        Command releaseCommand = m_arm.getReleaseCommand(); // release coral

        return Commands.sequence(Commands.waitUntil(() -> m_arm.atTarget()),releaseCommand);
    }

}
