// package frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
// import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
// import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
// import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
// import frc.robot.commands.CompoundCommands.CompCoCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.ElevatorSS;
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.SensorSS;
// import frc.robot.subsystems.WristSS;

// public class ToggleCoralInfeedCoCommand extends SequentialCommandGroup{

//     private boolean endCommand = false;


//     public ToggleCoralInfeedCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor) {

//         addCommands(
//             new RepeatCommand(
                
//                 new ConditionalCommand(
//                     //on true
//                         new ConditionalCommand(
//                             //on true
//                             new SequentialCommandGroup(
//                                 new ParallelCommandGroup(
//                                     new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.MAX_PID_OUTPUT),
//                                     new SequentialCommandGroup(
//                                         new InfeedCommand(s_Infeed, 0, 0),
//                                         new WaitCommand(0.1),
//                                         new InfeedCommand(s_Infeed, -0.1, -0.1),
//                                         new WaitCommand(0.1),
//                                         new InfeedCommand(s_Infeed, 0, 0)
//                                     )
//                                 ),
//                                 new WaitCommand(0.25),
//                                 new ParallelCommandGroup(
//                                     new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT),
//                                     new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT)
//                                 ),
//                                 new InstantCommand(() -> s_Sensor.setEndCoralInfeedCommand(true))
//                             ),
//                             //on false
//                             new SequentialCommandGroup(
//                                 new InfeedCommand(s_Infeed, 0, 0),
//                                 new InfeedCommand(s_Infeed, -0.145, -0.145),
//                                 new WaitCommand(0.28),
//                                 new InfeedCommand(s_Infeed, 0, 0),
//                                 new InstantCommand(() -> s_Sensor.setEndCoralInfeedCommand(true))
//                             ),
//                             () -> s_Sensor.getInfeedState()
//                         ),

//                     //on false
//                         new ParallelCommandGroup(
//                             new InstantCommand(() -> s_Sensor.setEndCoralInfeedCommand(false)),
//                             new ConditionalCommand(
//                                 //on true
//                                 new CoralInfeedCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
//                                 //on false
//                                 new CoralSourceInfeedCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed),
//                                 //condition
//                                 () -> s_Sensor.getInfeedState()
//                             )
//                         ),
//                     //condition
//                     () -> s_Sensor.coralSensed()
                    
//                 )
//             ).finallyDo(() -> new InstantCommand(() -> s_Sensor.setInfeedState(false)))
//         );
//         addRequirements(s_Infeed, s_Wrist, s_Arm, s_Elevator);
//     }
    
   
// }

package frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands;

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
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;

public class ToggleCoralInfeedCoCommand extends SequentialCommandGroup{

    private boolean endCommand = false;


    public ToggleCoralInfeedCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor) {

        addCommands(
            new RepeatCommand(
                
                new ConditionalCommand(
                    //on true
                        new ConditionalCommand(
                            //on true
                            new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                    new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.MAX_PID_OUTPUT),
                                    new SequentialCommandGroup(
                                        new InfeedCommand(s_Infeed, 0, 0),
                                        new WaitCommand(0.1),
                                        new InfeedCommand(s_Infeed, -0.1, -0.1),
                                        new WaitCommand(0.08), //0.09
                                        new InfeedCommand(s_Infeed, 0, 0)
                                    ),
                                    new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT),
                                    new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT)
                                ),
                                new InstantCommand(() -> s_Sensor.setInfeedState(true)),
                                new InstantCommand(() -> endCommand = true)
                            ),
                            //on false
                            new SequentialCommandGroup(
                                new InfeedCommand(s_Infeed, 0, 0),
                                new InfeedCommand(s_Infeed, -0.145, -0.145),
                                new WaitCommand(0.28),
                                new InfeedCommand(s_Infeed, 0, 0),
                                new InstantCommand(() -> endCommand = true)
                            ),
                            () -> s_Sensor.getInfeedState()
                        ),

                    //on false
                        new ParallelCommandGroup(
                            new InstantCommand(() -> endCommand = false),
                            new ConditionalCommand(
                                //on true
                                new CoralInfeedCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                //on false
                                new CoralSourceInfeedCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed),
                                //condition
                                () -> s_Sensor.getInfeedState()
                            )
                        ),
                    //condition
                    () -> s_Sensor.coralSensed()
                    
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