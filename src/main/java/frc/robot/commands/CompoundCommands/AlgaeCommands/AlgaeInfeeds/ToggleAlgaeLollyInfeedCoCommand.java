
package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.ScoringCommands.ProcessorCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;

public class ToggleAlgaeLollyInfeedCoCommand extends SequentialCommandGroup{

    private boolean endCommand = false;


    public ToggleAlgaeLollyInfeedCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor) {

        addCommands(
            new RepeatCommand(
                
                new ConditionalCommand(
                    //on true
                        new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(0.2),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT)
                        ),
                        new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT),
                        new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.ALGAE_INFEED_PID_OUTPUT),
                        new InstantCommand(() -> s_Infeed.setVoltage(1))
                        ),
                    //on false
                        new ParallelCommandGroup(
                            new InstantCommand(() -> endCommand = false),
                            new ConditionalCommand(
                                //on true
                                new AlgaeInfeedCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                //on false
                                new AlgaeInfeedLollypopCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                //condition
                                () -> s_Sensor.getInfeedState()
                            )
                        ),
                    //condition
                    () -> s_Sensor.bottomAlgaeSensed()
                    
                )
            )
            .until(() -> endCommand)
            .finallyDo(() -> 
                new ParallelCommandGroup(
                    new InstantCommand(() -> endCommand = false), 
                    new InstantCommand(() -> s_Sensor.setInfeedState(true))
                )
            )
        );
        addRequirements(s_Infeed, s_Wrist, s_Arm, s_Elevator);
    }
    
   
}