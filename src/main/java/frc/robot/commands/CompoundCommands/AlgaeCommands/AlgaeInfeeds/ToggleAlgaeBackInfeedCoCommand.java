
package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.InfeedConstants;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL1CoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL2CoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;

public class ToggleAlgaeBackInfeedCoCommand extends SequentialCommandGroup{

    private boolean endCommand = false;


    public ToggleAlgaeBackInfeedCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor) {

        addCommands(
            new RepeatCommand(
                
                new ConditionalCommand(
                    //on true
                    new SequentialCommandGroup(
                      new InstantCommand(() -> s_Infeed.setVoltage(InfeedConstants.IDLE_ALGAE_VOLTAGE))
                    ),

                    //on false
                        new ParallelCommandGroup(
                            new InstantCommand(() -> endCommand = false),
                            new ConditionalCommand(
                                //on true
                                new AlgaeInfeedL2CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                //on false
                                new AlgaeInfeedL1CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
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