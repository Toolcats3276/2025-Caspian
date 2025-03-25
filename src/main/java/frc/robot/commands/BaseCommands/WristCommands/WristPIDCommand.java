package frc.robot.commands.BaseCommands.WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;

public class WristPIDCommand extends Command{

    private WristSS s_Wrist;
    private double setPoint;
    private double maxSpeed;

    public WristPIDCommand(WristSS s_Wrist, double setPoint, double maxSpeed){
        this.s_Wrist = s_Wrist;
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        addRequirements(s_Wrist);
    }


    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        s_Wrist.PID(setPoint, maxSpeed);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
