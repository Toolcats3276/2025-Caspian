package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;

public class ShootCoCommand extends SequentialCommandGroup{



    public ShootCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor) {

        addCommands(
            // new RepeatCommand(  
                new ParallelCommandGroup(
                    new InstantCommand(() -> s_Sensor.setInfeedState(false)),
                    new ConditionalCommand(
                        //while true
                        new ConditionalCommand(
                            new SequentialCommandGroup(
                                new InfeedCommand(s_Infeed, InfeedConstants.CORAL_SHOT, InfeedConstants.CORAL_SHOT),
                                new WaitCommand(0.25), //FLIP BACK DELAY
                                new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.MAX_PID_OUTPUT)
                            ),
                            new ConditionalCommand(
                                new ConditionalCommand(
                                    new InfeedCommand(s_Infeed, InfeedConstants.CORAL_SHOT, InfeedConstants.CORAL_SHOT),
                                    new InfeedCommand(s_Infeed, -InfeedConstants.CORAL_SHOT, -InfeedConstants.CORAL_SHOT),
                                    () -> s_Sensor.coralSensed()
                                ),
                                new ConditionalCommand(
                                    new InfeedCommand(s_Infeed, 0.05, 0.4),
                                    new ConditionalCommand(
                                        new InfeedCommand(s_Infeed, InfeedConstants.L3_SHOT, InfeedConstants.L3_SHOT),
                                        new InfeedCommand(s_Infeed, InfeedConstants.CORAL_SHOT, InfeedConstants.CORAL_SHOT),
                                        () -> s_Arm.returnSetPoint() == ArmConstants.L3_Front),
                                    () -> s_Arm.returnSetPoint() == ArmConstants.L1_Front),

                                () -> s_Arm.returnSetPoint() == ArmConstants.COMP),


                            () -> s_Arm.returnSetPoint() == ArmConstants.L4_Front
                        ),
                        //while false
                        new ConditionalCommand(
                            new SequentialCommandGroup(
                                new InfeedCommand(s_Infeed, -InfeedConstants.CORAL_SHOT, -InfeedConstants.CORAL_SHOT),
                                new WaitCommand(0.25),//FLIP BACK DELAY
                                new WristPIDCommand(s_Wrist, WristConstants.L4_FLIP_BACK, WristConstants.MAX_PID_OUTPUT)
                            ),

                            new ConditionalCommand(
                                new ConditionalCommand(
                                    new InfeedCommand(s_Infeed, -InfeedConstants.PROCESSOR_SHOT, -InfeedConstants.PROCESSOR_SHOT),
                                    new InfeedCommand(s_Infeed, -InfeedConstants.TELE_BARGE_SHOT, -InfeedConstants.TELE_BARGE_SHOT), 
                                    () -> s_Arm.returnSetPoint() == ArmConstants.PROCESSOR
                                ),
                                new InfeedCommand(s_Infeed, -InfeedConstants.CORAL_SHOT, -InfeedConstants.CORAL_SHOT),
                                () -> s_Sensor.bottomAlgaeSensed()
                            ),
                            
                            () -> s_Arm.returnSetPoint() == ArmConstants.L4
                        ),

                        //condition
                        () -> s_Arm.returnSetPoint() == ArmConstants.L2_Front ||
                         s_Arm.returnSetPoint() == ArmConstants.L3_Front ||
                         s_Arm.returnSetPoint() == ArmConstants.L4_Front ||
                         s_Arm.returnSetPoint() == ArmConstants.COMP ||
                         s_Arm.returnSetPoint() == ArmConstants.L1_Front
                    )
                )
        );
        addRequirements(s_Infeed, s_Wrist, s_Arm, s_Elevator);
    }
    
   
}
