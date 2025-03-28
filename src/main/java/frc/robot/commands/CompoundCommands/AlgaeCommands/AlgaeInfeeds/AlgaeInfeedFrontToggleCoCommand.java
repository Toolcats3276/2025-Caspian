
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
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL1CoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL1FrontCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL2CoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL2FrontCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;

public class AlgaeInfeedFrontToggleCoCommand extends SequentialCommandGroup{

    private boolean endCommand = false;


    public AlgaeInfeedFrontToggleCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor) {

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
                                new AlgaeInfeedL2FrontCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                //on false
                                new AlgaeInfeedL1FrontCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
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