package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Setpoints
{

  public static class Wrist {
    public static final double rest = 0.59;
    public static final double active = 0.37;
  }

  public static class Elevator
  {

    public static class Coral
    {

      public static double L1 = 0;
      public static double L2 = 0.014;
      public static double L3 = 0.014;
      public static double L4 = 0.47;
      public static double HP = 0;
    }

    public static class Algae
    {
      public static final double L23       = 0.039;
      public static final double L34       = 0.0566;
      public static final double NET       = 0.735;
      public static final double PROCESSOR = 0.059;
    }
  }

  public static class Arm
  {

    public static class Coral
    {

      public static double HP = 13;// TOtAl GuesS
      public static double L1 = 0;
      public static double L2 = 10;
      public static double L3 = 45.14;
      public static double L4 = 67.9;
    }

    public static class Algae
    {

      public static final double L34       = 37.2;
      public static final double L23       = 2.637;
      public static final double NET       = 90;
      public static final double PROCESSOR = -32; // Guess
    }
  }

  public static class AutoScoring
  {
    public static class Processor {
      public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                               Inches.of(0).in(Meters),
                                                               Rotation2d.fromDegrees(0));
    }
    public static class Reef
    {
// x + front ->, y + left 
      public static final Transform2d coralOffset = new Transform2d(Inches.of(29).in(Meters),
                                                                    Inches.of(5).in(Meters),
                                                                    Rotation2d.fromDegrees(180));
      public static final Transform2d     algaeOffset = new Transform2d(Inches.of(23).in(Meters),
                                                                    Inches.of(-19).in(Meters),
                                                                    Rotation2d.fromDegrees(180));
    }

    public static class HumanPlayer
    {

      public static class Left
      {

        public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                                 Inches.of(0).in(Meters),
                                                                 Rotation2d.fromDegrees(0));
      }

      public static class Right
      {

        public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                                 Inches.of(0).in(Meters),
                                                                 Rotation2d.fromDegrees(0));
      }
    }
  }


}
