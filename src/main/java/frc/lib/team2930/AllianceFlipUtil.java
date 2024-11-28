package frc.lib.team2930;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class AllianceFlipUtil {

  public static Translation2d flipVelocitiesForAlliance(Translation2d originalVelocity) {
    return Constants.isRedAlliance() ? originalVelocity.unaryMinus() : originalVelocity;
  }
}
