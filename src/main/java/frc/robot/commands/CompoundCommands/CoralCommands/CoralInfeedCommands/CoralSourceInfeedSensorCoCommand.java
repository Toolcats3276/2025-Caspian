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

public class CoralSourceInfeedSensorCoCommand extends SequentialCommandGroup{

public static boolean endCommand = false;

    public CoralSourceInfeedSensorCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor) {

        addCommands(
            new RepeatCommand(  
                new ConditionalCommand(
                    //while true
                    new SequentialCommandGroup(
                        new InfeedCommand(s_Infeed, 0, 0),
                        new InfeedCommand(s_Infeed, -0.145, -0.145),
                        new WaitCommand(0.18),
                        new InfeedCommand(s_Infeed, 0, 0),
                        new InstantCommand(() -> endCommand = true)
                    ),
                    //while false
                    new ParallelCommandGroup(
                        new InstantCommand(() -> endCommand = false),
                        new ArmPIDCommand(s_Arm, ArmConstants.CORAL_SOURCE_INFEED, ArmConstants.MAX_PID_OUTPUT),
                        new WristPIDCommand(s_Wrist, WristConstants.CORAL_SOURCE_INFEED, WristConstants.MAX_PID_OUTPUT),
                        new ElevatorPIDCommand(s_Elevator, ElevatorConstants.CORAL_SOURCE_INFEED, ElevatorConstants.MAX_PID_OUTPUT),
                        new InfeedCommand(s_Infeed, InfeedConstants.CORAL_SOURCE_INFEED, InfeedConstants.CORAL_SOURCE_INFEED)
                        // new InfeedCommand(s_Infeed, 0)
                    ),

                    //condition
                    () -> s_Sensor.coralSensed()
                )
            )
            .until(() -> endCommand)
            // .finallyDo(() -> new InstantCommand(() -> endCommand = false))
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}