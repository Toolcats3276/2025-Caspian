package frc.robot.commands.CompoundCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.InfeedConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ManualArmCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ManualElevatorCommand;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.ManualWristCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;

public class CancelCoCommand extends SequentialCommandGroup{



    public CancelCoCommand(WristSS s_Wrist, ArmSS s_Arm, ClimberSS s_Climber, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor, DoubleSupplier WristAxis, DoubleSupplier ArmAxis, DoubleSupplier ElevatorAxis, DoubleSupplier NegativeElevatorAxis) {

        addCommands(
            new ParallelCommandGroup(
                new ManualWristCommand(s_Wrist, WristAxis),
                new ManualArmCommand(s_Arm, ArmAxis),
                new ManualElevatorCommand(s_Elevator, ElevatorAxis, NegativeElevatorAxis),
                new InfeedCommand(s_Infeed, 0, 0),
                new InstantCommand(() -> s_Climber.ManualStop()),
                new InstantCommand(() -> s_Sensor.setInfeedState(false)),
                new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND))
            )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}