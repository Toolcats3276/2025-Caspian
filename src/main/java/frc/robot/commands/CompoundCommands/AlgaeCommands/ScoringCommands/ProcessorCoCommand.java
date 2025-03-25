package frc.robot.commands.CompoundCommands.AlgaeCommands.ScoringCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;

public class ProcessorCoCommand extends SequentialCommandGroup{

boolean endCommand = false;

    public ProcessorCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

        addCommands(
            new RepeatCommand(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new ArmPIDCommand(s_Arm, ArmConstants.PROCESSOR, ArmConstants.MAX_PID_OUTPUT),
                            new WristPIDCommand(s_Wrist, WristConstants.PROCESSOR, WristConstants.PROCESSOR_PID_OUTPUT),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.PROCESSOR, ElevatorConstants.MAX_PID_OUTPUT),
                            new InstantCommand(() -> s_Infeed.setVoltage(InfeedConstants.IDLE_ALGAE_VOLTAGE))
                        ),
                        new InstantCommand(() -> endCommand = true)
                    ), 
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new InstantCommand(() -> endCommand = false),
                            // new WristPIDCommand(s_Wrist, WristConstants.PROCESSOR, WristConstants.PROCESSOR_PID_OUTPUT),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.PROCESSOR, ElevatorConstants.MAX_PID_OUTPUT),
                            new InstantCommand(() -> s_Infeed.setVoltage(InfeedConstants.IDLE_ALGAE_VOLTAGE))
                        )
                    ), 
                    () -> s_Elevator.atCompPose()
                )
            ).until(() -> endCommand).finallyDo(() -> new InstantCommand(() -> endCommand = false))
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}