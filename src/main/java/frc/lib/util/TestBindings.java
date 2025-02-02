package frc.lib.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface TestBindings {
  void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller);
}
