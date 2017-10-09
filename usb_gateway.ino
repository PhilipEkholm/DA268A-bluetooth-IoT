/*
 * Description: This is a dummy sketch enabling the Arduino Uno to act
 * as an USB-gateway toward the BTBee Pro Xbee shield.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *-------------------------------------------------------------------------------
 * IMPORTANT NOTICE!!!!!
 *
 * Make sure that the Xbee shield jumpers are set to run towards Arduino and
 * in UBSB-gateway mode. This is done through the following jumper-settings:
 *
 * - Identify the D0..D13 tri-rail pins with "DOUT" & "DIN" marked on the bottom.
 *   The pins in the middle are the actual D0..D13 pins and the DIN/DOUT rails are
 *   short-circuited in parallell with these pins.
 *
 * - Set the first jumper to connect D0 to the DIN pin rail
 * - Set the second jumper to connect D1 to the DOUT pin rail
 * - Build and flash the sketch (short command: Ctrl+U)
 * - Start the Arduino Serial Monitor (short command: Ctrl+Shift+M)
 */
 
void setup() {
}

void loop() {
}
