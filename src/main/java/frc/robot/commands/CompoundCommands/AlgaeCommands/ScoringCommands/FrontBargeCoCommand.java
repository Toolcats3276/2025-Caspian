
package frc.robot.commands.CompoundCommands.AlgaeCommands.ScoringCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;

public class FrontBargeCoCommand extends SequentialCommandGroup{



    public FrontBargeCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

        addCommands(
                    //on true
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new ArmPIDCommand(s_Arm, ArmConstants.BARGE_Front, ArmConstants.ALGAE_BARGE_PID_OUTPUT),
                            new WristPIDCommand(s_Wrist, WristConstants.BARGE_Front, WristConstants.BARGE_PID_OUTPUT),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.BARGE_Front, ElevatorConstants.MAX_PID_OUTPUT),
                            new InstantCommand(() -> s_Infeed.setVoltage(1))
                        )
                    )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
}