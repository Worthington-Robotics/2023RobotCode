package frc.robot;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ExampleTest {
	@Test
	public void testExample() {
		System.out.println("Testing for 1 = 1");
		System.out.println("These print statements will " 
			+ "be captured in the report");
			System.out.println("They are located on the standard output tab");
		assertEquals(1, 1);
	}

	@Test
	public void printOnlyTest() {
		System.out.println("this is a test that only has print statements");
		System.out.println("there are no assertions here");
	}
}
