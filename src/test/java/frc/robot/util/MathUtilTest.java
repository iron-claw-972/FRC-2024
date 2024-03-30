package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class MathUtilTest {

	@Test
	public void firmwareTest() {
		assertEquals(5633, MathUtils.encodeFirmwareVersion(22, 1, 1));
	}

}

