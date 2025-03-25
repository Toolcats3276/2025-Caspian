package frc.robot.commands.BaseCommands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSS;

public class ClimberIn extends Command{

    private ClimberSS s_Climber;



    public ClimberIn(ClimberSS s_Climber){
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
    }


    @Override
    public void initialize(){
        s_Climber.ManualIn();
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
