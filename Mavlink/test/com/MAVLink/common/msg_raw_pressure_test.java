/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */
         
// MESSAGE RAW_PRESSURE PACKING
package com.MAVLink.common;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Parser;
import com.MAVLink.ardupilotmega.CRC;
import java.nio.ByteBuffer;
import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;

/**
* The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
*/
public class msg_raw_pressure_test{

public static final int MAVLINK_MSG_ID_RAW_PRESSURE = 28;
public static final int MAVLINK_MSG_LENGTH = 16;
private static final long serialVersionUID = MAVLINK_MSG_ID_RAW_PRESSURE;

private Parser parser = new Parser();

public CRC generateCRC(byte[] packet){
    CRC crc = new CRC();
    for (int i = 1; i < packet.length - 2; i++) {
        crc.update_checksum(packet[i] & 0xFF);
    }
    crc.finish_checksum(MAVLINK_MSG_ID_RAW_PRESSURE);
    return crc;
}

public byte[] generateTestPacket(){
    ByteBuffer payload = ByteBuffer.allocate(6 + MAVLINK_MSG_LENGTH + 2);
    payload.put((byte)MAVLinkPacket.MAVLINK_STX); //stx
    payload.put((byte)MAVLINK_MSG_LENGTH); //len
    payload.put((byte)0); //seq
    payload.put((byte)255); //sysid
    payload.put((byte)190); //comp id
    payload.put((byte)MAVLINK_MSG_ID_RAW_PRESSURE); //msg id
    payload.putLong((long)93372036854775807L); //time_usec
    payload.putShort((short)17651); //press_abs
    payload.putShort((short)17755); //press_diff1
    payload.putShort((short)17859); //press_diff2
    payload.putShort((short)17963); //temperature
    
    CRC crc = generateCRC(payload.array());
    payload.put((byte)crc.getLSB());
    payload.put((byte)crc.getMSB());
    return payload.array();
}

@Test
public void test(){
    byte[] packet = generateTestPacket();
    for(int i = 0; i < packet.length - 1; i++){
        parser.mavlink_parse_char(packet[i] & 0xFF);
    }
    MAVLinkPacket m = parser.mavlink_parse_char(packet[packet.length - 1] & 0xFF);
    byte[] processedPacket = m.encodePacket();
    assertArrayEquals("msg_raw_pressure", processedPacket, packet);
}
}
        