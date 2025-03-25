package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds;

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

public class AlgaeInfeedCommand extends SequentialCommandGroup{



    public AlgaeInfeedCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor) {

        addCommands(
                    new ParallelCommandGroup(
                        new ArmPIDCommand(s_Arm, ArmConstants.ALGAE_INFEED, ArmConstants.MAX_PID_OUTPUT),
                        new WristPIDCommand(s_Wrist, WristConstants.ALGAE_INFEED, WristConstants.MAX_PID_OUTPUT),
                        new ElevatorPIDCommand(s_Elevator, ElevatorConstants.ALGAE_INFEED_Ground, ElevatorConstants.MAX_PID_OUTPUT),
                        new InfeedCommand(s_Infeed, InfeedConstants.ALGAE_INFEED, InfeedConstants.ALGAE_INFEED),
                        // new InfeedCommand(s_Infeed, 0)
                        new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND))
                    )
                );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}