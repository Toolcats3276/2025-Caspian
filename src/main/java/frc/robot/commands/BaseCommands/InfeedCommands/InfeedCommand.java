package frc.robot.commands.BaseCommands.InfeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfeedSS;

public class InfeedCommand extends Command {
    
    private InfeedSS s_Infeed;
    private double left_Speed;
    private double right_Speed;

    public InfeedCommand(InfeedSS s_Infeed, double left_Speed, double right_Speed) {
        this.s_Infeed = s_Infeed;
        this.left_Speed = left_Speed;
        this.right_Speed = right_Speed;
        addRequirements(s_Infeed);
    }

    @Override
    public void initialize() {
        s_Infeed.setSpeed(left_Speed, right_Speed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
