package frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands;

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

public class CoralInfeedCoCommand extends SequentialCommandGroup{



    public CoralInfeedCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor) {

        addCommands(
                new ParallelCommandGroup(
                    new ConditionalCommand(
                        new ParallelCommandGroup(                        
                            new ArmPIDCommand(s_Arm, ArmConstants.CORAL_INFEED, ArmConstants.MAX_PID_OUTPUT),
                            new WristPIDCommand(s_Wrist, WristConstants.CORAL_INFEED, WristConstants.MAX_PID_OUTPUT),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                            new InfeedCommand(s_Infeed, InfeedConstants.LEFT_CORAL_INFEED, InfeedConstants.RIGHT_CORAL_INFEED)
                        ),
                        new ParallelCommandGroup(
                            new WristPIDCommand(s_Wrist, WristConstants.CORAL_INFEED, WristConstants.MAX_PID_OUTPUT),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                            new InfeedCommand(s_Infeed, InfeedConstants.LEFT_CORAL_INFEED, InfeedConstants.RIGHT_CORAL_INFEED)
                        ),
                        () -> s_Elevator.atCompPose())

                )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}