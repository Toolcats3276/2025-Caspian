package frc.robot.commands.BaseCommands.WristCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.WristSS;

public class ManualWristCommand extends Command {
    
    private WristSS s_Wrist;
    private DoubleSupplier wristSup;

    public ManualWristCommand(WristSS s_Wrist, DoubleSupplier wristSup) {
        this.s_Wrist = s_Wrist;
        this.wristSup = wristSup;
        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double WristVal = MathUtil.applyDeadband(wristSup.getAsDouble(), Constants.stickDeadband)/5;
        s_Wrist.Manual(WristVal);
        SmartDashboard.putNumber("WristVal", WristVal);
      }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
