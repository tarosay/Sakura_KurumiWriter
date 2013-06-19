  GR-SAKURAを用いて、GR-KURUMIにプログラムを書き込むツールです。

License
-----
The MIT License (MIT)

Copyright (c) 2013 fujita nozomu

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

-----

[GR-KURUMIの接続方法]  
   SCI_SCI0P2x を使用する場合、  

    GR-KURUMI  -  GR-SAKURA
          GND  -  GND
          CTS  -  N.C.未接続
          VCC  -  VCC(3.3V)
          RXI  -  IO 1
          TXD  -  IO 0
          DTR  -  IO 2

 SCI_SCI1JTAG を使用する場合、  

    GR-KURUMI  -  GR-SAKURA JTAG コネクタ(ピン番号)
          GND  -  (13)GND
          CTS  -  N.C.未接続
          VCC  -  (11)VCC
          RXI  -  (3)TXD1
          TXD  -  (6)RXD1
          DTR  -  (2)P34
  
Sakura_KurumiWriter
===================
