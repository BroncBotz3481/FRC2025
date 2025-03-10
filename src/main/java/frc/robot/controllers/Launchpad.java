package frc.robot.controllers;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Launchpad
{

  //numbers must be within 0-63
  //[63,0,0] - Red || [0,63,0] - Green || [0,0,63] - Blue  || [63,63,63] - White || [0,0,0] - Blank/Black
  //This will update the LEDs once the python attaches to NT
  //This table is visually accurate to the launchpad each of these rbg sets will change it's respective button
  private IntegerArrayPublisher rgbTablePublisher;
  private long[]                rgbHex      = new long[9 * 9 + 1];
  private long[]                savedRgbHex = new long[9 * 9 + 1];

  private Trigger[][] buttons = new Trigger[9][9];

  private NetworkTableInstance table;
  private NetworkTable         launch;
  private CommandGenericHID[]  vjoys;

  /**
   * Creates the Launchpad Controller with the program "launchpad.py" running on the driver station with vJoy
   * configured.
   *
   * @param vjoy1        The first vJoy controller with 32 buttons. Corresponds to the first 32 buttons on the
   *                     Launchpad.
   * @param vjoy2        The second vJoy controller with 32 buttons. Corresponds to the second 32 buttons on the
   *                     Launchpad.
   * @param vjoy3        The third vJoy controller with 18 buttons. Corresponds to the last 32 buttons on the
   *                     Launchpad.
   * @param pressedColor Color to be shown when a button is pressed.
   */
  public Launchpad(int vjoy1, int vjoy2, int vjoy3, Color8Bit pressedColor)
  {
    table = NetworkTableInstance.getDefault();
    launch = table.getTable("launchpad");
    rgbTablePublisher = launch.getIntegerArrayTopic("colors").publish();
    defaultLEDs();
    vjoys = new CommandGenericHID[]{new CommandGenericHID(vjoy1), new CommandGenericHID(vjoy2), new CommandGenericHID(
        vjoy3)};
    for (int i = 0; i < 9 * 9; i++)
    {
      int vjoy_num   = Math.floorDiv(i, 32);
      int row        = Math.floorDiv(i, 9);
      int col        = i % 9;
      int button_num = getButtonNum(col, row);

      // Change me!

      int button = button_num - (32 * Math.floorDiv(i, 32));
      if(button != 0)
      {
      changeLED(col, row, new Color8Bit(Color.kOrange));

      buttons[row][col] = vjoys[vjoy_num].button(button);
      buttons[row][col].onTrue(Commands.print("(" + row + "," + (col-1) + ") pressed"));
      buttons[row][col].onFalse(Commands.runOnce((() -> this.restoreSavedLED(col, row))));
      buttons[row][col].whileTrue(Commands.run((() -> this.feedback(col, row, pressedColor))));
      }
    }
  }

  /**
   * Gets the trigger connected to a button
   *
   * @param x x Position of the launchpad, where +x is to the right, starting at the top left (0-7)
   * @param y x Position of the launchpad, where +y is down, starting at the top left (0-7)
   * @return The trigger class connected to the button
   * @throws IllegalArgumentException if x or y is not within 0-7
   */
  public Trigger getButton(int x, int y)
  {
    if (x > 8 || y > 8 || x < 0 || y < 0)
    {
      throw new IllegalArgumentException("Coords must be less than 7");
    }
    return buttons[y][x];
  }

  // Function to shrink the long[] array to a smaller size
  private long[] shrinkArray(long[] originalArray, int originalSize)
  {
    if (originalArray == null || originalArray.length != originalSize)
    {
      throw new IllegalArgumentException("Input array size does not match the given original size.");
    }

    // Calculate the size of the result array based on the original size
    int resultSize = (originalSize + 1) /
                     2;  // Each resulting element combines two original ones (rounding up for odd sizes)

    // Array of the new size for the result
    long[] resultArray = new long[resultSize];

    // Iterate through the original array and combine two long values in one step
    for (int i = 0; i < resultSize; i++)
    {
      long combinedValue = (originalArray[i * 2] & 0xFFFFFFFFL);  // Lower 4 bytes
      if (i * 2 + 1 < originalSize)
      {
        combinedValue |= (originalArray[i * 2 + 1] & 0xFFFFFFFFL) << 32;  // Upper 4 bytes
      }
      resultArray[i] = combinedValue;
    }

    return resultArray;
  }

  private int getButtonNum(int x, int y)
  {
    int button_num = y * 9 + x;
    if (y > 0)
    {
      button_num += 1;
    }
    return button_num;
  }

  /**
   * Changes the color of a button's LED
   *
   * @param x   x Position of the launchpad, where +x is to the right, starting at the top left (0-7)
   * @param y   y Position of the launchpad, where +y is down, starting at the top left (0-7)
   * @param rgb the long[3] array that describes the color of the button, each position of the array must be (0-63)
   * @throws IllegalArgumentException if x or y is not within 0-7, or if array is invalid.
   */
  public void changeLED(int x, int y, long[] rgb)
  {
    if (x > 8 || y > 8 || x < 0 || y < 0)
    {
      throw new IllegalArgumentException("Coords must be within 0-8");
    }
    if (rgb.length != 3)
    {
      throw new IllegalArgumentException("Array length must be 3");
    }
    for (long i : rgb)
    {
      if (i > 63 || i < 0)
      {
        throw new IllegalArgumentException(i + " is an invalid rgb input, must be within 0-63");
      }
    }
    int button_num = getButtonNum(x, y);
//        System.out.println("Setting LED "+button_num+" color.");

    long rgbHeax = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
    rgbHex[button_num] = rgbHeax;
    savedRgbHex[button_num] = rgbHeax;
    rgbTablePublisher.set(shrinkArray(rgbHex, rgbHex.length));
  }

  /**
   * Changes the color of a button's LED
   *
   * @param x         x Position of the launchpad, where +x is to the right, starting at the top left (0-7)
   * @param y         y Position of the launchpad, where +y is down, starting at the top left (0-7)
   * @param color8Bit the Color8Bit object that describes the color of the button
   * @throws IllegalArgumentException if x or y is not within 0-7
   */
  public void changeLED(int x, int y, Color8Bit color8Bit)
  {
    if (x > 8 || y > 8 || x < 0 || y < 0)
    {
      throw new IllegalArgumentException("Coords must be within 0-7");
    }
    int button_num = getButtonNum(x, y);
//        System.out.println("Setting LED "+button_num+" color.");

    long[] rgb     = convertColor8Bit(color8Bit);
    long   rgbHeax = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
    rgbHex[button_num] = rgbHeax;
    savedRgbHex[button_num] = rgbHeax;
    rgbTablePublisher.set(shrinkArray(rgbHex, rgbHex.length));
  }

  private void feedback(int x, int y, Color8Bit color8Bit)
  {
    int  button_num = getButtonNum(x, y);
    long saved      = savedRgbHex[button_num];
    changeLED(x, y, color8Bit);
    savedRgbHex[button_num] = saved;
  }

  private void feedback(int x, int y, long[] rgb)
  {

    int  button_num = getButtonNum(x, y);
    long saved      = savedRgbHex[button_num];
    changeLED(x, y, rgb);
    savedRgbHex[button_num] = saved;
  }

  private void restoreSavedLED(int x, int y)
  {
    int button_num = getButtonNum(x, y);
    rgbHex[button_num] = savedRgbHex[button_num];
    rgbTablePublisher.set(shrinkArray(rgbHex, rgbHex.length));
  }

  /**
   * Converts Color8Bit to proper long[3] array
   *
   * @param color Color8Bit to convert
   * @return long[3] for the rgb value
   */
  public long[] convertColor8Bit(Color8Bit color)
  {
    return new long[]{(color.red / 4), (color.green / 4), (color.blue / 4)};
  }

  /**
   * Changes the color of the whole LED board based on the 8x8 rgbTable array in the class
   *
   * @throws IllegalArgumentException if anything in the rgbTable array is invalid
   */
  public void defaultLEDs()
  {
//        if (rgbTable.length != 9) {
//            throw new IllegalArgumentException(rgbTable.length + " is an invalid rgbTable length");
//        }
//        for (int i=0; i < rgbTable.length; i++) {
//            if (rgbTable[i].length != 9) {
//                throw new IllegalArgumentException(rgbTable[i].length + " is an invalid rgbTable length");
//            }
//            for (int j=0; j < rgbTable[i].length; j++) {
//                for (long l : rgbTable[i][j]) {
//                    if (l > 63 || l < 0) {
//                        throw new IllegalArgumentException(i + " is an invalid rgb input, must be within 0-63");
//                    }
//                }
//                launch.getSubTable(Integer.toString(i)).putValue(Integer.toString(j), NetworkTableValue.makeIntegerArray(rgbTable[i][j]));
//            }
//        }
  }

}