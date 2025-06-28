package frc.robot.commands.CompoundCommands.AlgaeCommands.ScoringCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;


public class ToggleAlgaeBargeCoCommand extends SequentialCommandGroup{



    public ToggleAlgaeBargeCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor, BooleanSupplier  endcommand) {

        addCommands(
                new ConditionalCommand(
                    //true  
                    new BargeCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed),
                    ///false
                    new FrontBargeCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed),
                    //condition
                    () -> s_Arm.returnSetPoint() == ArmConstants.BARGE_Front
            )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}