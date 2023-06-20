/*
* Tuan Luong
* tdluong@crimson.ua.edu
* 
* 
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "matek3901.h"

/* Matek3901 object on UART4 */
bfs::Matek3901 opflow(&Serial4);

int main() {
  /* Serial to display data */
  Serial.begin(9600);
  while (!Serial) {}
  /* Initialize communication */
  if (!opflow.Begin()) {
    Serial.println("Unable to communicate with Matek3901");
    while (1) {}
  }
  while (1) {
    /* Check for new data */
    if (opflow.Read()) {
      Serial.print(opflow.range_mm());
      Serial.print("\t");
      Serial.print(opflow.range_qual());
      Serial.print("\n");
    }
  }
}
