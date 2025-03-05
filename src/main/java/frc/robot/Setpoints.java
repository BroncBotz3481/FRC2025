package frc.robot;

public class Setpoints
{
  public static class Elevator
  {
    public static class Coral {
      public static double L1 = 0;
      public static double L2 = 0.014;
      public static double L3 = 0.014;
      public static double L4 = 0.47;
      public static double HP = 0;
    }

    public static class Algae
    {
      public static final double L23 = 0.039;
      public static final double L34 = 0.0566;
      public static final double NET = 0.735;
      public static final double PROCESSOR = 0.014;
    }
  }

  public static class Arm
  {

    public static class Coral
    {
      public static double L1 = 0;
      public static double L2 = 10;
      public static double L3 = 36.14;
      public static double L4 = 57.9;
    }

    public static class Algae
    {
      public static final double L23 = 33.2;
      public static final double L34 = 2.637;
      public static final double NET = 90;
      public static final double PROCESSOR = -50; // Guess
    }
  }


}
