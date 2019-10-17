package org.traccar.protocol;

import org.junit.Test;
import org.traccar.ProtocolTest;
import org.traccar.model.Position;

public class UproProtocolDecoderTest extends ProtocolTest {

    @Test
    public void testDecode() throws Exception {

        UproProtocolDecoder decoder = new UproProtocolDecoder(null);

        verifyAttribute(decoder, buffer(
                "*VK200867282036729446,BA&A1759265051877702037465660022210819&B0000000000&W00&G000030&M830&N26&O1706&o11&T0783#"),
                Position.KEY_BATTERY_LEVEL, 83.0);

        verifyAttributes(decoder, buffer(
                "*VK201867282035754650,AH&B0000000000&W00&M990&N31&Z02&b2&T0458#"));

        verifyAttributes(decoder, buffer(
                "*VK201867282035455779,AH&B0000000000&W00&M940&N30&Z02&Y12922&T0268#"));

        verifyPosition(decoder, buffer(
                "*VK200867282035455779,BA&A0850065052928902036605660013170719&B0000000000&W00&G000030&M850&N20&O1808&o10&Y12922&T0081#"));

        verifyAttributes(decoder, buffer(
                "*VK200867282035455779,BA&X260,6,1016,13931,60;1016,13929,81;1016,14174,82;1016,13930,82&E190717103920&B0100000000&W00&G000030&M900&N23&O0000&o07&Y14014&T0015#"));

        verifyPosition(decoder, buffer(
                "*HQ200861810538000002,BA&A0206033302618209658563620115180119&B0100000040&C6328680=&F0039&R2710&V0036&T09&K50000&N04&P0200#"));

        verifyPosition(decoder, buffer(
                "*HQ200999999,AB1&A1656512233362911356523660000230618&B0100060010&C00000<6<&F0000&R2405&V0109&W0000003E&K00100&T65&X(k89860045191536000374)#"));

        verifyPosition(decoder, buffer(
                "*HQ20113800138000,YAA&A0732142233550011405829060520190314&B0100000000&C00001234&R3109&T80#"));

        verifyPosition(decoder, binary(
                "2a4d473230313836383530303032303030343836372c414226413035303032343138313438373536303636303131373732323030303031313132313626583331302c3236302c34383837322c353639312c37333b34383837322c3732322c38363b34383837322c353639332c38383b34383837322c323336332c39303b34383837322c323336322c393726423030303030303030303026573030264e3230265a31342659313430303323"));

        verifyPosition(decoder, binary(
                "2a4d473230303639333530323030303033353537332c42412641303834313237333332363334353230373033383933373630303030303235313131362642303130303030303030302647303036323030264d393930264e3235264f3035303026433030313a363b363926510411058c0c125c0d0a2fff4237ee614d66454c140826555f50000000000300000000000000000026543139333723"));

        verifyPosition(decoder, buffer(
                "*MG201693502000035441,BA&A1213073325458307036690710000151116&P0730000032ce4fb3&D1&B0000000000&C005799?7&S3,20161115120025,07035.54659E,3324.87721N,3000,0,0,0,0,847,599,8,40,0,19,20&U_P\0\0\0\0\0\0\0\0\0\0\0\0\0\0&T0107"));

        verifyPosition(decoder, buffer(
                "*MG201693502000034964,AB&A0800253335360507036975710000091116&P0730000032d2a94d&B0000000000&N13&Z12&U_P\0\0\0\u0004\0\0\0\0\0\0\0\0\0\0"),
                position("2016-11-09 08:00:25.000", true, -33.58934, -70.61626));

        verifyNull(decoder, buffer(
                "*MG20113800138000,AH"));

        verifyPosition(decoder, buffer(
                "*MG201693502000034964,AB&A0200183324418107033792800009051116&B0000000000&N15&Z94&U_P\0\0\0\0\0\0\0\0\0\0\0\0\0\0"));

        verifyPosition(decoder, buffer(
                "*MG201693502000034964,AB&A0200543324412007033805910000051116&P0730000032d66785&B0000000000&N15&Z92&U_P\0\0\0\0\0\0\0\0\0\0\0\0\0\0"));

        verifyPosition(decoder, buffer(
                "*AI2000905447674,BA&A2003064913201201845107561627121016&B0100000000&C05>8=961&F0333&K023101002154A7"));

        verifyPosition(decoder, buffer(
                "*AI200905300036,AH&A0317264913209801844913060000251115&B0500000000&C0;4?72:9&F0000"),
                position("2015-11-25 03:17:26.000", false, 49.22016, 18.74855));

        verifyPosition(decoder, buffer(
                "*AI2000905300036,AS&A1647304913209801844913060000251115&B0400000000&C0;4?72:9&F0000"));

        verifyPosition(decoder, buffer(
                "*AI2000905300036,AC1&A1648014913209801844913060000251115&B0400000000&C0;4?72:9&F0000"));

        verifyPosition(decoder, buffer(
                "*AI2000905300036,AB1&A1702464913231101844949860000251115&B0500000000&C0;4?72:9&F0000"));

        verifyPosition(decoder, buffer(
                "*AI2000905300036,AD1&A1703054913231101844949860000251115&B0500000000&C0;4?72:9&F0000"));

    }

}
