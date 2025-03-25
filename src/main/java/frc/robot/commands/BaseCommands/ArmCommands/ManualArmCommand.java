package frc.robot.commands.BaseCommands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSS;

public class ManualArmCommand extends Command {
    
    private ArmSS s_Arm;
    private DoubleSupplier armSup;

    public ManualArmCommand(ArmSS s_Arm, DoubleSupplier armSup) {
        this.s_Arm = s_Arm;
        this.armSup = armSup;
        addRequirements(s_Arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double armVal = MathUtil.applyDeadband(armSup.getAsDouble(), Constants.stickDeadband)/4;
        s_Arm.Manual(armVal);
        SmartDashboard.putNumber("armVal", armVal);
      }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
