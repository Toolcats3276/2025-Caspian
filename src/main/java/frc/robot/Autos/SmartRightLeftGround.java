package frc.robot.Autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;


public class SmartRightLeftGround extends SequentialCommandGroup {
  /** Creates a new SmartRightLeftGround. */
  public SmartRightLeftGround(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor) {

    addCommands(

      new SequentialCommandGroup(
        new PathPlannerAuto("Smart RLG 1"),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new PathPlannerAuto("Smart RLG 2"), 
            new ConditionalCommand(
              new PathPlannerAuto("Smart RLG 3"), 
              new PathPlannerAuto("Smart RLG 4"), 
              () -> s_Sensor.coralSensed()
            )
          ),

          new SequentialCommandGroup(
            new PathPlannerAuto("Smart RLG 1- 2"),
            new ConditionalCommand(
              new PathPlannerAuto("Smart RLG 2-3"), 
              new PathPlannerAuto("Smart RLG 2-S"), 
              () -> s_Sensor.coralSensed()
            ) 
          ),
          // new PathPlannerAuto("Smart RLG 1- 2"),
          () -> s_Sensor.coralSensed()
        )
      )
    );
  }
}
