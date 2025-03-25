package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;

public class AlgaeInfeedL2CoCommand extends SequentialCommandGroup{



    public AlgaeInfeedL2CoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor) {

        addCommands(
            new ParallelCommandGroup(
                new ArmPIDCommand(s_Arm, ArmConstants.ALGAE_INFEED_L2, ArmConstants.MAX_PID_OUTPUT),
                new WristPIDCommand(s_Wrist, WristConstants.ALGAE_INFEED_L2, WristConstants.MAX_PID_OUTPUT),
                new ElevatorPIDCommand(s_Elevator, ElevatorConstants.ALGAE_INFEED_L2, ElevatorConstants.MAX_PID_OUTPUT),
                new InfeedCommand(s_Infeed, InfeedConstants.ALGAE_INFEED, InfeedConstants.ALGAE_INFEED),
                new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_L2))
            )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}
