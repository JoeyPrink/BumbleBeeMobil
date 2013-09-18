package bumblebee.bumpercar;

public class Util {
	public static void pause(int tim) {
		try {
			Thread.sleep(tim);
		} catch (Exception e) {
		}
	}
}