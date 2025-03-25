package frc.robot.commands.BaseCommands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSS;

public class ManualElevatorCommand extends Command {
    
    private ElevatorSS s_Elevator;
    private DoubleSupplier elevatorSup;
    private DoubleSupplier negativeElevatorSup;

    public ManualElevatorCommand(ElevatorSS s_Elevator, DoubleSupplier elevatorSup, DoubleSupplier negativeElevatorSup) {
        this.s_Elevator = s_Elevator;
        this.elevatorSup = elevatorSup;
        this.negativeElevatorSup = negativeElevatorSup;
        addRequirements(s_Elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double elevatorVal = MathUtil.applyDeadband(elevatorSup.getAsDouble() - negativeElevatorSup.getAsDouble(), Constants.stickDeadband);
        s_Elevator.Manual(elevatorVal);
        SmartDashboard.putNumber("elevatorVal", elevatorVal);
      }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
