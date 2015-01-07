# DRCでのネットワークについて
本ドキュメントは[DRC Finals Rules](http://www.theroboticschallenge.org/sites/default/files/2014_12_3_Comms_Doc_For_Teams_DISTAR_Case_23912.pdf),
および[DRC Q&A](http://www.theroboticschallenge.org/files/2015_01_05_DRC_QA.pdf)に基づいたものである。

![](images/drc_network_overview_by_darpa.png)

(This image is provided by darpa in [this document](http://www.theroboticschallenge.org/sites/default/files/2014_12_3_Comms_Doc_For_Teams_DISTAR_Case_23912.pdf))

## ルール概要
![](images/network_overview.png)

大きく分けて5つの領域に計算機を置くことが可能。その間の通信は制限される。
1. Robot
2. Field Computer
3. Operator Computor
4. Operator Cloud

特に2と3の間で提供される通信は特殊である。
![](images/network_fc_ocs.png)
* (0~1023 port) 常時2と3へ10kbpsから2kbpsの間の通信, 遅延なしの通信。
* (16384~24575 port) 3から4へ、1秒間の300Mbps, 遅延なしの通信。
ただしこれは300Mbitの"burst"通信が1秒から30秒の間で
ランダムに通信可能になる。

この２つはポートによって切り分けられる。また、ICMP(ping)は上の常時接続された
経路を通る.


# ネットワーク通信量
## プロトコル
![OSI model](images/osi_layer.png)

### TCP/IP
TCP/IPはハンドシェイクが必要となるプロトコルである。
つまり計算機AからBへパケットPを通信したい時、AからBへパケットPを送るだけではなく、
BからACKと呼ばれる返答パケットが返ってこなくてはいけない。

この場合、通信遅延は双方向の和となる。

例えばAからBへの経路に遅延が1秒、BからAへの経路に遅延が1秒であった場合、遅延は2秒となる.

#### パケット構造
TCP/IPのヘッダは20byteである。さらにIP層も含めると、58byteとなる。
さらに8byteのパディングを入れることが多いため、実際のデータに加え66byteを利用すると考えて良い。

これはACKの最小サイズは66byteであると考えてよく、1秒間に1回データをやりとりするとして、
2000bit/secのレギュレーションの場合はおよそ1/4がACKで埋まるという事である。

### UDP
UDPはハンドシェイクを必要としない。ただし、信頼できる通信を実装するには、自分で
ACKのような確認パケットを送る仕組みを用意しなくてはいけない。

#### パケット構造
UDPのヘッダは8byteである。IP層も含めると36byteである。

### ROS
ROSのパケットはかなり軽量である。

簡単なメッセージは以下のようなサイズである.
8 byteのオフセットがヘッダとして追加される。

---
    Message Type                | Data Size (bit)
--------------------------------|---------------
`std_msgs/Int32`                | 64
`std_msgs/Int64`                | 96
`std_msgs/String` (0 character) | 64
`std_msgs/String` (8 character) | 128
`std_msgs/Header` (empty frame) | 160

---
```python
In [14]: b = StringIO.StringIO()
In [15]: rospy.msg.serialize_message(b, 0, String())
In [16]: b.len
Out[16]: 8

In [18]: b = StringIO.StringIO()
In [19]: rospy.msg.serialize_message(b, 0, String(data='hogehoge'))
In [20]: b.len
Out[20]: 16

In [7]: b = StringIO.StringIO()
In [8]: rospy.msg.serialize_message(b, 0, Int32(data=10))
In [9]: b.len
Out[9]: 8

In [10]: b = StringIO.StringIO()
In [11]: rospy.msg.serialize_message(b, 0, Int64(data=10))
In [12]: b.len
Out[12]: 12
```

## 通信量の観点から
10kbpsは厳しい帯域制限である。

---
以下に各種データに関してそのサイズの大きさを示す。これらは理論値であり、実際に通信するときは
各パケットに対しヘッダが付与される。

* データ量比較

Data Type                      | Data Size(bit)     | Num per sec in 10kbps | Num per sec in 2kbps
-------------------------------|--------------------|-----------------------|---------------------
Image (RGB,VGA)                | `640*480*3*8=7.2M` | 0.001                 | 0.0003
Image (RGB,QVGA)               | `320*240*3*8=1.8M` | 0.006                 | 0.001
Image (Mono,VGA)               | `640*480*1*8=2.4M` | 0.004                 | 0.0008
Image (Mono,QVGA)              | `320*240*1*8=0.6M` | 0.02                  | 0.003
Image (Mono,20x20)             | `20*20*8=0.4K`     | 3                     | 0.625
Image (Mono,10x10)             | `10*10*8=0.1K`     | 12                    | 2.5
Float                          | `32`               | 300                   | 62.5
Angle Vector (32 Float array)  | `1024`             | 9                     | 2
Bool                           | `1`                | 9600                  | 2000
Int                            | `64`               | 150                   | 31
OK-Warn-Error Status           | `2`                | 4800                  | 1000
Character                      | `8`                | 1200                  | 250
