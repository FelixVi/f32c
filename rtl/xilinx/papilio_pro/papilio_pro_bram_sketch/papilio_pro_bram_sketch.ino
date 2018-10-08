//
// Copyright (c) 2018 Felix Vietmeyer
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.
//

extern "C" {
  #include <dev/io.h>
  #include <dev/spi.h>
}

#define  IO_SPI_USER IO_ADDR(0x360)  /* half, RW */

int incomingByte = 0;   // for incoming serial data

void setup() {
  // put your setup code here, to run once:
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  Serial.begin(115200);
  while(!Serial);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while(Serial.available() > 0) {
          // read the incoming byte:
          incomingByte = Serial.read();

          //process input
          switch(incomingByte){
            case 'a':
            digitalWrite(10, LOW);
            Serial.println("Turning LED off");
            break;
            case 'b':
            digitalWrite(10, HIGH);
            Serial.println("Turning LED on");
            break;
            case 'c':
            Serial.println("SPI write");
            spi_start_transaction(IO_SPI_USER);
            spi_byte(IO_SPI_USER, 0xac);
            spi_byte(IO_SPI_USER, 0x00);
            spi_byte(IO_SPI_USER, 0x0F);
          }
  }
  
}
