/*
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
*/

#include <rxduino.h>
#include <iodefine_gcc63n.h>

#define UseStatusDisplay 1      // ステータス表示使用スイッチ
#define UseDebugStdout 0        // デバグ用標準出力使用スイッチ

#define DefaultBaudRate 115200  // UART 通信の初期ボー・レート
#if 0
#define MainBaudRate 115200     // UART 通信のボー・レート 115200bps
#elif 0
#define MainBaudRate 250000     // UART 通信のボー・レート 250kbps
#elif 0
#define MainBaudRate 500000     // UART 通信のボー・レート 500kbps
#elif 1
#define MainBaudRate 1000000    // UART 通信のボー・レート 1Mbps (オススメ)
#else
#error
#endif

#if 1   // SCI_SCI0P2x を使用
#define SCI_KURUMI SCI_SCI0P2x  // KURUMI と接続に使用する SCI ポート
#define PFS_RXD P21PFS          // KURUMI からのシリアル通信受信ピン割り当て
#define PFS_TXD P20PFS          // KURUMI へのシリアル通信送信ピン割り当て
#define PIN_RXD PIN_P21         // KURUMI からのシリアル通信受信ピン
#define PIN_TXD PIN_P20         // KURUMI へのシリアル通信送信ピン
#define PIN_RESET PIN_P22       // KURUMI リセットピン
#elif 1 // SCI_SCI1JTAG を使用
#define SCI_KURUMI SCI_SCI1JTAG // KURUMI と接続に使用する SCI ポート
#define PFS_RXD P30PFS          // KURUMI からのシリアル通信受信ピン割り当て
#define PFS_TXD P26PFS          // KURUMI へのシリアル通信送信ピン割り当て
#define PIN_RXD PIN_P30         // KURUMI からのシリアル通信受信ピン
#define PIN_TXD PIN_P26         // KURUMI へのシリアル通信送信ピン
#define PIN_RESET PIN_P34       // KURUMI リセットピン
#else
#error
#endif

enum {  // JTAG コネクタ I/O ピン
    PIN_P26 = 201,
    PIN_P27,
    PIN_P30,
    PIN_P31,
    PIN_P34,
};

// JTAG コネクタのポート対応版 pinMode()
static void myPinMode(int pin, int mode)
{
    if (pin >= 0 && pin <= PIN_LAST_BOARDPIN) {
        pinMode(pin, mode);
    } else {
        int pdr = (mode == INPUT || mode == INPUT_PULLUP) ? 0 : 1;
        int podr = 0;
        int pmr = 0;
        int odr = 0;
        int pcr = (mode == INPUT_PULLUP) ? 1 : 0;
        int dscr = (mode == OUTPUT_HIGH) ? 1 : 0;
        int psel = 6;
        switch (pin) {
        case PIN_P26:
            PORT2.PDR.BIT.B6 = pdr;
            PORT2.PODR.BIT.B6 = podr;
            PORT2.PMR.BIT.B6 = pmr;
            PORT2.ODR1.BIT.B4 = odr;
            PORT2.PCR.BIT.B6 = pcr;
            PORT2.DSCR.BIT.B6 = dscr;
            MPC.P26PFS.BIT.PSEL = psel;
            break;
        case PIN_P27:
            PORT2.PDR.BIT.B7 = pdr;
            PORT2.PODR.BIT.B7 = podr;
            PORT2.PMR.BIT.B7 = pmr;
            PORT2.ODR1.BIT.B6 = odr;
            PORT2.PCR.BIT.B7 = pcr;
            PORT2.DSCR.BIT.B7 = dscr;
            MPC.P27PFS.BIT.PSEL = psel;
            break;
        case PIN_P30:
            PORT3.PDR.BIT.B0 = pdr;
            PORT3.PODR.BIT.B0 = podr;
            PORT3.PMR.BIT.B0 = pmr;
            PORT3.ODR0.BIT.B0 = odr;
            PORT3.PCR.BIT.B0 = pcr;
            MPC.P30PFS.BIT.PSEL = psel;
            break;
        case PIN_P31:
            PORT3.PDR.BIT.B1 = pdr;
            PORT3.PODR.BIT.B1 = podr;
            PORT3.PMR.BIT.B1 = pmr;
            PORT3.ODR0.BIT.B2 = odr;
            PORT3.PCR.BIT.B1 = pcr;
            MPC.P31PFS.BIT.PSEL = psel;
            break;
        case PIN_P34:
            PORT3.PDR.BIT.B4 = pdr;
            PORT3.PODR.BIT.B4 = podr;
            PORT3.PMR.BIT.B4 = pmr;
            PORT3.ODR1.BIT.B0 = odr;
            PORT3.PCR.BIT.B4 = pcr;
            MPC.P34PFS.BIT.PSEL = psel;
            break;
        default:
            break;
        }
    }
}
#define pinMode(pin, mode) myPinMode(pin, mode)

// JTAG コネクタのポート対応版 digitalWrite()
static void myDigitalWrite(int pin, int value)
{
    if (pin >= 0 && pin <= PIN_LAST_BOARDPIN) {
        digitalWrite(pin, value);
    } else {
        int podr = value ? 1 : 0;
        switch (pin) {
        case PIN_P26:
            PORT2.PODR.BIT.B6 = podr;
            break;
        case PIN_P27:
            PORT2.PODR.BIT.B7 = podr;
            break;
        case PIN_P30:
            PORT3.PODR.BIT.B0 = podr;
            break;
        case PIN_P31:
            PORT3.PODR.BIT.B1 = podr;
            break;
        case PIN_P34:
            PORT3.PODR.BIT.B4 = podr;
            break;
        default:
            break;
        }
    }
}
#define digitalWrite(pin, mode) myDigitalWrite(pin, mode)

// CSerial の不具合対策と機能の拡張
class CMySerial : public CSerial {
private:
    volatile struct st_sci0* sci;

public:
    CMySerial() : CSerial() {
        sci = NULL;
    }

    void begin(int bps, SCI_PORT port)
    {
        CSerial::begin(bps, port);
        switch (port) {
        case SCI_SCI0P2x:
            sci = &SCI0;
            break;
        case SCI_SCI1JTAG:
            sci = &SCI1;
            break;
        case SCI_SCI2A:
        case SCI_SCI2B:
            sci = &SCI2;
            break;
        case SCI_SCI6A:
        case SCI_SCI6B:
            sci = &SCI6;
            break;
        default:
            sci = NULL;
            break;
        }
        if (sci != NULL) {
            sci->SCR.BIT.TIE = 0;       // TXI 割り込み禁止
            sci->SCR.BIT.TEIE = 0;      // TEI 割り込み禁止
        }
    }

    void end()
    {
        CSerial::end();
        if (sci != NULL) {
            // 送受信禁止
            sci->SCR.BIT.TE = 0;
            sci->SCR.BIT.RE = 0;
        }
        sci = NULL;
    }

    // TDR が空になるまで待つ
    void flush()
    {
        while (sci != NULL && sci->SSR.BIT.TEND == 0) {
            ;
        }
    }

    // 送信するデータをバッファリングせず、TDR が空になるまで待って送信
    size_t write(unsigned char val)
    {
        if (sci != NULL) {
            flush();

            // 送信データ書き込み
            sci->TDR = val;
            return 1;
        } else {
            return 0;
        }
    }

    void setStopBit(unsigned int stopBit)
    {
        if (sci != NULL) {
            // 送受信禁止
            sci->SCR.BIT.TE = 0;
            sci->SCR.BIT.RE = 0;

            // 送信ストップビット長を変更
            sci->SMR.BIT.STOP = stopBit - 1;

            // 送受信許可
            sci->SCR.BIT.TE = 1;
            sci->SCR.BIT.RE = 1;
        }
    }

    void setBaudRate(unsigned int baudRate)
    {
        if (sci != NULL) {
            // 送受信禁止
            sci->SCR.BIT.TE = 0;
            sci->SCR.BIT.RE = 0;

            // ボーレート設定
            sci->SMR.BIT.CKS = 0;       // n = 0
            sci->SEMR.BIT.ABCS = 1;     // 調歩同期基本クロックセレクトビットを 1 にする
            sci->BRR = (48 * 1000000 / (32 / 2) + (baudRate / 2)) / baudRate - 1;   // ビットレートレジスタを設定

            // 送受信許可
            sci->SCR.BIT.TE = 1;
            sci->SCR.BIT.RE = 1;
        }
    }
};

// PC とのシリアル通信オブジェクト
static CSerial PcSerial;

// KURUMI とのシリアル通信オブジェクト
static CMySerial KurumiSerial;

// タイマー割り込みによるリセット監視
static void timer()
{
    // 青タクトスイッチが押されたら SAKURA をリセット
    if (!digitalRead(PIN_SW)) {
        system_reboot(REBOOT_USERAPP);
    }
}

void setup()
{
    // デバグ用標準出力初期化
#if UseDebugStdout
    Serial.begin(115200, SCI_SCI1JTAG);
    Serial.setDefault();
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\x1b[H\x1b[2J");
#endif

    // PC 接続ポート設定
    PcSerial.begin(DefaultBaudRate);
    sci_convert_crlf_ex(PcSerial.get_handle(), CRLF_NONE, CRLF_NONE);    // バイナリを通せるようにする

#if UseStatusDisplay
    // ステータス表示 LED 初期化
    for (int i = 0; i < 4; i++) {
        pinMode(PIN_LED0 + i, OUTPUT);
        digitalWrite(PIN_LED0 + i, LOW);
    }
#endif

    // RESET 出力ポート設定
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, HIGH);

    // SAKURA の青タクトスイッチを SAKURA のリセットに使用
    pinMode(PIN_SW, INPUT);

    // リセット処理の監視処理をタイマー割り込みに登録
    timer_regist_userfunc(timer);
}

void loop()
{
    // 状態
    static enum {SETUP, WAIT, PROGRAM} status = SETUP;

    // 通信タイムアウト計測用変数
    static long lastTimer;

    switch (status) {
    case SETUP:
        // TXD 出力ポート設定
        pinMode(PIN_TXD, OUTPUT);
        digitalWrite(PIN_TXD, HIGH);

        // KURUMI をリセット
        digitalWrite(PIN_RESET, LOW);
        delayMicroseconds(10);
        digitalWrite(PIN_RESET, HIGH);

#if UseStatusDisplay
        // ステータス表示 LED0 のみ点灯(書き込み待ち)
        for (int i = 0; i < 4; i++) {
            digitalWrite(PIN_LED0 + i, (i == 0) ? HIGH : LOW);
        }
#endif
        // WAIT に移行
        status = WAIT;
        break;

    case WAIT:
        // モード引き込みの際に PC から 0x00 が送信されてくる筈なので待つ
        if (PcSerial.available() && PcSerial.read() == 0x00) {
            // フラッシュ・メモリ・プログラミング・モードへの引き込み
            const int tRT = 723;
            const int tTM = 16;
            const int tMB = 62;
            digitalWrite(PIN_TXD, LOW);
            digitalWrite(PIN_RESET, LOW);
            delayMicroseconds(10);
            digitalWrite(PIN_RESET, HIGH);
            delayMicroseconds(tRT);
            digitalWrite(PIN_TXD, HIGH);
            delayMicroseconds(tTM);
            digitalWrite(PIN_TXD, LOW);
            delayMicroseconds((1 + 8) * 1000000 / DefaultBaudRate);
            delayMicroseconds(tMB);

            // KURUMI 接続ポート設定
            KurumiSerial.begin(DefaultBaudRate, SCI_KURUMI);
            KurumiSerial.setStopBit(2);         // ストップビット長を 2 に変更
            sci_convert_crlf_ex(KurumiSerial.get_handle(), CRLF_NONE, CRLF_NONE);    // バイナリを通せるようにする

            // PROGRAM に移行
            status = PROGRAM;
            lastTimer = timer_get_ms();
        }
        break;

    case PROGRAM:
        // PC から来たコマンド/データ・フレームを KURUMI に送る
        if (PcSerial.available()) {
            // コマンド/データ・フレームを格納するバッファ
            byte buf[1000];

            // コマンド/データ・フレームを読み込む
            for (int i = 0; i <= 1 || i < (byte(buf[1] - 1) + 1 + 4); i++) {
                while (!PcSerial.available()) {
                    ;
                }
                buf[i] = PcSerial.read();
            }

#if MainBaudRate != DefaultBaudRate
            // Baud Rate Set コマンドだったのなら、ボー・レート設定データ(D01)を変更する
            if (buf[0] == 0x01 && buf[1] == 0x03 && buf[2] == 0x9a) {
                buf[5] += buf[3];
#if MainBaudRate == 250000
                buf[3] = 0x01;
#elif MainBaudRate == 500000
                buf[3] = 0x02;
#elif MainBaudRate == 1000000
                buf[3] = 0x03;
#else
#error
#endif
                buf[5] -= buf[3];
            }
#endif

            // コマンド/データ・フレームを KURUMI に送る
            for (int i = 0; i < (byte(buf[1] - 1) + 1 + 4); i++) {
                KurumiSerial.write(buf[i]);
            }

            // KURUMI から来たステータス・フレームを PC に送る
            int len = 0;
            for (int i = 0; i <= 1 || i < len; i++) {
                while (!KurumiSerial.available()) {
                    ;
                }
                byte d = KurumiSerial.read();
                PcSerial.write(d);
                if (i == 1) {
                    len = d + 4;
                }
            }

            // PC から来たコマンド・フレームが Baud Rate Set コマンドだったのなら、パラメータに応じてボーレートを変更する
            if (buf[0] == 0x01 && buf[1] == 0x03 && buf[2] == 0x9a && (buf[3] >= 0x00 && buf[3] <= 0x03)) {
                static const unsigned long baudRateTable[] = {
                    115200, 250000, 500000, 1000000,
                };
                KurumiSerial.setBaudRate(baudRateTable[buf[3]]);
            }

#if UseStatusDisplay
            // ステータス表示 LED アニメーション(転送中)
            int i = (timer_get_ms() / 100) & 7;
            for (int j = 0; j < 4; j++) {
                digitalWrite(PIN_LED0 + j, j == (i & 4 ? 7 - i : i & 3) ? HIGH : LOW);
            }
#endif

            // 通信時刻更新
            lastTimer = timer_get_ms();
        }

        // KURUMI からデータ・フレームが来たら PC に送る
        if (KurumiSerial.available()) {
            int len = 0;
            for (int i = 0; i <= 1 || i < len; i++) {
                while (!KurumiSerial.available()) {
                    ;
                }
                byte d = KurumiSerial.read();
                PcSerial.write(d);
                if (i == 1) {
                    len = d + 4;
                }
            }

            // 通信時刻更新
            lastTimer = timer_get_ms();
        }

        // 最後の通信から 500m 秒経過したら SETUP に移行
        if ((timer_get_ms() - lastTimer) >= 500) {
            // KurumiSerial を閉じる
            KurumiSerial.end();

            // SCI 切り離し
            MPC.PFS_RXD.BIT.PSEL = 0;
            MPC.PFS_TXD.BIT.PSEL = 0;

            // SETUP に移行
            status = SETUP;
        }
        break;
    }
}

