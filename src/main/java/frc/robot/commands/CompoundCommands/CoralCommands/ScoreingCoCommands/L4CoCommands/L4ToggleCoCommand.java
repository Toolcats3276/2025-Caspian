package frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;

public class L4ToggleCoCommand extends SequentialCommandGroup{

    public L4ToggleCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

        addCommands(
                new ConditionalCommand(
                    new ParallelCommandGroup(
                        new L4BackCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed)
                    ),
                    new ParallelCommandGroup(
                        new L4FrontCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed)
                    ),
                    () -> s_Arm.returnSetPoint() == ArmConstants.L4_Front
                )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}