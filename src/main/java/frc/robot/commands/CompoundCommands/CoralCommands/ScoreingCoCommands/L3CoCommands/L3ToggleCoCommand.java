package frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L3CoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;

public class L3ToggleCoCommand extends SequentialCommandGroup{



    public L3ToggleCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

        addCommands(
                new ConditionalCommand(
                    new ParallelCommandGroup(
                        new L3BackCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed)
                        ),
                        
                    new ParallelCommandGroup(
                        new L3FrontCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed)
                    ),

                    () -> s_Arm.returnSetPoint() == ArmConstants.L3_INVERSE
                )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}