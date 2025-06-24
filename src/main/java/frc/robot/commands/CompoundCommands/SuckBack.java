package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;

public class SuckBack extends SequentialCommandGroup{



    public SuckBack(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor) {

        addCommands(
            new SequentialCommandGroup(
                new InfeedCommand(s_Infeed, 0, 0),
                new WaitCommand(0.1),
                new InfeedCommand(s_Infeed, -0.1, -0.1),
                new WaitCommand(0.08),
                new InfeedCommand(s_Infeed, 0, 0)
            )      
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
}