package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;

public class CompCoCommand extends SequentialCommandGroup{

    public boolean endCommand = false;

    public CompCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor) {

        addCommands(
            new RepeatCommand(
                new ConditionalCommand(  
                    // algae comp commands (barge, processor, etc.)    
                    new ConditionalCommand(
                        // after elevator is down
                        new ParallelCommandGroup(
                            new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.ALGAE_INFEED_PID_OUTPUT),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                            new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT), 
                            new InfeedCommand(s_Infeed, 0, 0),
                            new InstantCommand(() -> s_Infeed.setVoltage(1)),
                            new InstantCommand(() -> s_Sensor.setInfeedState(false)),
                            new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND)),
                            new InstantCommand(() -> endCommand = true)
                        ), 
                        // before elevator is down
                        new ParallelCommandGroup(
                            new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.ALGAE_INFEED_PID_OUTPUT),
                            new InstantCommand(() -> endCommand = false),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                            new InfeedCommand(s_Infeed, 0, 0),
                            new InstantCommand(() -> s_Infeed.setVoltage(1)),
                            new InstantCommand(() -> s_Sensor.setInfeedState(false)),
                            new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND))
                        ), 
                        () -> s_Elevator.atCompPose()
                    ),
                    
                    // general comp command (coral, infeed, etc.)
                    new ConditionalCommand(
                    // after elevator is down
                        new ConditionalCommand(
                            // L4 suck back
                            new ParallelCommandGroup(
                                new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.MAX_PID_OUTPUT),
                                new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                                new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT), 
                                new SequentialCommandGroup(
                                    new InfeedCommand(s_Infeed, -0.18, -0.18),
                                    new WaitCommand(0.06),
                                    new InfeedCommand(s_Infeed, 0, 0),
                                    new InstantCommand(() -> endCommand = true)
                                ),
                                new InstantCommand(() -> s_Sensor.setInfeedState(false)),
                                new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND))
                            ), 
                            // general comp command
                            new ParallelCommandGroup(
                                new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.MAX_PID_OUTPUT),
                                new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                                new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT), 
                                new InfeedCommand(s_Infeed, 0, 0),
                                new InstantCommand(() -> s_Sensor.setInfeedState(false)),
                                new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND)),
                                new InstantCommand(() -> endCommand = true)

                            ), 
                            () -> s_Arm.returnSetPoint() == ArmConstants.L4 ||
                                  s_Arm.returnSetPoint() == ArmConstants.L3 ||
                                  s_Arm.returnSetPoint() == ArmConstants.L2),
                                  
                    // before elevator is down
                        new ParallelCommandGroup(
                            new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.ALGAE_INFEED_PID_OUTPUT),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                            new InfeedCommand(s_Infeed, 0, 0),
                            new InstantCommand(() -> s_Sensor.setInfeedState(false)),
                            new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND)),
                            new InstantCommand(() -> endCommand = false)
                        ), 
                        () -> s_Elevator.atCompPose()
                    ),

                    () -> s_Sensor.algaeSensed()
                )
            ).until(() -> endCommand)

        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}
// package frc.robot.commands.CompoundCommands;

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

// public class CompCoCommand extends SequentialCommandGroup{



//     public CompCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

//         addCommands(
//             new RepeatCommand(       
//                 new ConditionalCommand(
//                     new ParallelCommandGroup(
//                         new WristPIDCommand(s_Wrist, WristConstants.COMP),
//                         new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP),
//                         new ArmPIDCommand(s_Arm, ArmConstants.COMP), 
//                         new InfeedCommand(s_Infeed, 0)
//                     ), 
//                     new ParallelCommandGroup(
//                         new WristPIDCommand(s_Wrist, WristConstants.COMP),
//                         new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP),
//                         new InfeedCommand(s_Infeed, 0)
//                     ), 
//                     () -> s_Elevator.atCompPose()
//                 )
//             )

//         );
//         addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
//     }
    
   
// }








// // package frc.robot.commands.CompoundCommands;

// // import edu.wpi.first.wpilibj2.command.*;
// // import frc.robot.Constants.ArmConstants;
// // import frc.robot.Constants.ElevatorConstants;
// // import frc.robot.Constants.WristConstants;
// // import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
// // import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
// // import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
// // import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
// // import frc.robot.subsystems.ArmSS;
// // import frc.robot.subsystems.ElevatorSS;
// // import frc.robot.subsystems.WristSS;
// // import frc.robot.subsystems.InfeedSS;

// // public class CompCoCommand extends SequentialCommandGroup{



// //     public CompCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

// //         addCommands(
// //             new ParallelCommandGroup(
// //                 new WristPIDCommand(s_Wrist, WristConstants.COMP),
// //                 new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP),
// //                 new InfeedCommand(s_Infeed, 0)
// //             )
// //             .until(() -> s_Elevator.atSetpoint()
// //             )
// //             .andThen(
// //                 new ParallelCommandGroup(
// //                     new WristPIDCommand(s_Wrist, WristConstants.COMP),
// //                     new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP),
// //                     new ArmPIDCommand(s_Arm, ArmConstants.COMP), 
// //                     new InfeedCommand(s_Infeed, 0)
// //                 )
// //             )
// //         );
// //         addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
// //     }
    
   
// // }