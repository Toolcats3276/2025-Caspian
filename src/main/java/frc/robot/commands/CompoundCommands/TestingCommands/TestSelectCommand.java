package frc.robot.commands.CompoundCommands.TestingCommands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;

public class TestSelectCommand extends SequentialCommandGroup{

  // The enum used as keys for selecting the command to run.
  private enum CommandSelector {
    ONE,
    TWO,
    THREE
  }

  // An example selector method for the selectcommand.  Returns the selector that will select
  // which command to run.  Can base this choice on logical conditions evaluated at runtime.
  private CommandSelector select() {
    return CommandSelector.ONE;
  }

  // An example selectcommand.  Will select from the three commands based on the value returned
  // by the selector method at runtime.  Note that selectcommand works on Object(), so the
  // selector does not have to be an enum; it could be any desired type (string, integer,
  // boolean, double...)
  @SuppressWarnings("unused")
  private final Command m_exampleSelectCommand =
      new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(CommandSelector.ONE, new PrintCommand("Command one was selected!")),
              Map.entry(CommandSelector.TWO, new PrintCommand("Command two was selected!")),
              Map.entry(CommandSelector.THREE, new PrintCommand("Command three was selected!"))),
          this::select);
    
   
}