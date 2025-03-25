package frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SensorSS;


public class ToggleCoralInfeedStateCoCommand extends SequentialCommandGroup{

    
    public ToggleCoralInfeedStateCoCommand(SensorSS s_Sensor) {

        addCommands(
            new SequentialCommandGroup(
                new ConditionalCommand(
                    new InstantCommand(() -> s_Sensor.setInfeedState(false)), 
                    new InstantCommand(() -> s_Sensor.setInfeedState(true)), 
                    () -> s_Sensor.getInfeedState())
                    // new InstantCommand(() -> s_Sensor.setShuttleState(false))
            ).handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setInfeedState(false)))
        );   
    }
    
    
} 