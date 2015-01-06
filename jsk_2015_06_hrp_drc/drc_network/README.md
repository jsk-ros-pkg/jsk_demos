# DRCでのネットワークについて
本ドキュメントは[DRC Finals Rules](http://www.theroboticschallenge.org/files/2014_11_02_DRC_Finals_Rules_Release_2_DISTAR_23753.pdf)に基づいたものである。

## ルール概要
![](images/network_overview.png)

大きく分けて5つの領域に計算機を置くことが可能。その間の通信は制限される。
1. Robot
2. Field Computer
3. Field Cloud
4. Operator Computor
5. Operator Cloud

特に3と4の間で提供される通信は特殊である。
![](images/network_fc_ocs.png)
* 常時3から4へ10kbps, 遅延なしの通信
* 常時4から3へ10kbps, 遅延1秒の通信
* 3から4へ、1Gbps, 遅延なしの通信。ただしこれは300Mbitの"burst"通信が1秒から60秒の間で
ランダムに通信可能になる。

# ネットワーク通信量
## プロトコル
### TCP/IP
TCP/IPはハンドシェイクが必要となるプロトコルである。
つまり計算機AからBへパケットPを通信したい時、AからBへパケットPを送るだけではなく、
BからACKと呼ばれる返答パケットが返ってこなくてはいけない。

この場合、通信遅延は双方向の和となる。

例えばAからBへの経路に遅延が1秒、BからAへの経路に遅延が1秒であった場合、遅延は2秒となる.

### UDP
UDPはハンドシェイクを必要としない。ただし、信頼できる通信を実装するには、自分で
ACKのような確認パケットを送る仕組みを用意しなくてはいけない。

## 通信量の観点から
10kbpsは厳しい帯域制限である。

---
以下に各種データに関してそのサイズの大きさを示す。これらは理論値であり、実際に通信するときは
各パケットに対しヘッダが付与される。

* データ量比較

Data Type                      | Data Size(bit)     | Num per sec in 10kbps
-------------------------------|--------------------|----------------------
Image (RGB,VGA)                | `640*480*3*8=7.2M` | 0.0001
Image (RGB,QVGA)               | `320*240*3*8=1.8M` | 0.0006
Image (Mono,VGA)               | `640*480*1*8=2.4M` | 0.0004
Image (Mono,QVGA)              | `320*240*1*8=0.6M` | 0.002
Image (Mono,20x20)             | `20*20*8=0.4K`     | 0.32
Image (Mono,10x10)             | `10*10*8=0.1K`     | 1.28
Float                          | `32`               | 32
Angle Vector (32 Float array)  | `1024`             | 1
Bool                           | `1`                | 1024
Int                            | `64`               | 16
OK-Warn-Error Status           | `2`                | 512
Character                      | `8`                | 128
