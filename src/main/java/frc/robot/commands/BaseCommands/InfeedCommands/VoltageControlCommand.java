package frc.robot.commands.BaseCommands.InfeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfeedSS;

public class VoltageControlCommand extends Command {
    
    private InfeedSS s_infeed;
    private double Voltage;

    public VoltageControlCommand(InfeedSS s_infeed, Double Voltage) {
        this.Voltage = Voltage;
        this.s_infeed = s_infeed;
    }

    @Override
    public void initialize() {
        s_infeed.setVoltage(Voltage);
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
