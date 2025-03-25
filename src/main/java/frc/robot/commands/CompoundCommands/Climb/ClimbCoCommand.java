package frc.robot.commands.CompoundCommands.Climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.Climber.ClimberIn;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ClimberSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;

public class ClimbCoCommand extends SequentialCommandGroup{

private boolean endCommand = false;

    public ClimbCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, ClimberSS s_Climber) {

        addCommands(
            new RepeatCommand(
                new ConditionalCommand(
                    //on true
                    new SequentialCommandGroup(
                        new WaitCommand(.25),
                        new ParallelCommandGroup(
                            new ArmPIDCommand(s_Arm, ArmConstants.CLIMBED, ArmConstants.MAX_PID_OUTPUT)
                        ),
                        new InstantCommand(() -> endCommand = true)
                    ),
                    //on false
                    new ParallelCommandGroup(
                        new InstantCommand(() -> endCommand = false),
                        new ArmPIDCommand(s_Arm, ArmConstants.CLIMB_READY, ArmConstants.MAX_PID_OUTPUT),
                        new WristPIDCommand(s_Wrist, WristConstants.CORAL_INFEED, WristConstants.MAX_PID_OUTPUT),
                        new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                        new ClimberIn(s_Climber)
                    ),
                    //condition
                    () -> s_Climber.returnLeftClimbLimit() && s_Climber.returnRightClimbLimit()

                )
            ).until(() -> endCommand)
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator);
    }
    
   
}