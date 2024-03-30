package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class MathUtilTest {

	@Test
	public void firmwareTest() {
		assertEquals(5633, MathUtils.talonEncodeFirmwareVersion(22, 1, 1));
		assertEquals(402653184, MathUtils.revFlexEncodeFirmwareVersion(24, 0, 0, 0));
	}

}

