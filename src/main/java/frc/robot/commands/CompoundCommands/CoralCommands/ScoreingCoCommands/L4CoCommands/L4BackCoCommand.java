// package frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
// import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
// import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
// import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.ElevatorSS;
// import frc.robot.subsystems.WristSS;
// import frc.robot.subsystems.InfeedSS;

// public class L4CoCommand extends SequentialCommandGroup{



//     public L4CoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

//         addCommands(
//             new ParallelCommandGroup(
//                 new ArmPIDCommand(s_Arm, ArmConstants.L4),
//                 new ElevatorPIDCommand(s_Elevator, ElevatorConstants.L4),
//                 new InfeedCommand(s_Infeed, 0),
//                 new SequentialCommandGroup(
//                     new WaitCommand(1.25),
//                     new WristPIDCommand(s_Wrist, WristConstants.L4)
//                 )
//             ),
//             new WaitCommand(0.5),
//             new SequentialCommandGroup(
//                 new InfeedCommand(s_Infeed, 0.25),
//                 new WaitCommand(1),
//                 new ParallelCommandGroup(
//                     new InfeedCommand(s_Infeed, 0),
//                     new WristPIDCommand(s_Wrist, WristConstants.COMP)
//                 ),
//                 new ParallelCommandGroup(
//                     new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP),
//                     new SequentialCommandGroup(
//                         new WaitCommand(1.25),
//                         new ArmPIDCommand(s_Arm, ArmConstants.COMP)
//                     )
//                 )
//             )
//         );
//         addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
//     }
    
   
// }
package frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;

public class L4BackCoCommand extends SequentialCommandGroup{



    public L4BackCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

        addCommands(
            new ParallelCommandGroup(
                new WristPIDCommand(s_Wrist, WristConstants.L4, WristConstants.MAX_PID_OUTPUT),
                new ArmPIDCommand(s_Arm, ArmConstants.L4, ArmConstants.MAX_PID_OUTPUT),
                new ElevatorPIDCommand(s_Elevator, ElevatorConstants.L4, ElevatorConstants.MAX_PID_OUTPUT),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new InfeedCommand(s_Infeed, 0.18, 0.18),
                    new WaitCommand(0.08),
                    new InfeedCommand(s_Infeed, 0, 0)
                )
            )
            
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}