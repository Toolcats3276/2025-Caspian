package frc.robot.commands.BaseCommands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSS;

public class ArmPIDCommand extends Command{

    private ArmSS s_Arm;
    private double setPoint;
    private double maxSpeed;


    public ArmPIDCommand(ArmSS s_Arm, double setPoint, double maxSpeed){
        this.s_Arm = s_Arm;
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        addRequirements(s_Arm);
    }


    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_Arm.PID(setPoint, maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
