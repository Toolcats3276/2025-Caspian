package frc.robot.commands.BaseCommands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSS;

public class ElevatorPIDCommand extends Command{

    private ElevatorSS s_Elevator;
    private double setPoint;
    private double maxSpeed;


    public ElevatorPIDCommand(ElevatorSS s_Elevator, double setPoint, double maxSpeed){
        this.s_Elevator = s_Elevator;
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        addRequirements(s_Elevator);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_Elevator.PID(setPoint, maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
