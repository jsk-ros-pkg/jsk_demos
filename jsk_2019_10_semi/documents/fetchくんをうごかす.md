# fetchを動かす for 2019_semi
### fetchに接続

```bash
$ rossetip
$ rossetmaster fetch15
$ source ~/semi_ws/devel/setup.bash 
$ roslaunch jsk_fetch_startup rviz.launch #rvizを起動したいとき 
```

### fetchを操作
`rlwrap roseus`
```lisp
(fetch-init)
(objects (list *fetch*))
(send *ri* :state :potentio-vector) ;;現在の実機の状態を取得
(send *fetch* :angle-vector (send *ri* :state :potentio-vector)) ;;IRT viewerに適用
```

### fetchのポーズ一覧
fetch1  
#f(5.52373 52.5326 3.4729 77.8638 -59.6255 10.2535 -78.2225 74.2234 0.022649 2.23)
<img src="./images/fetch_pose1.png">

fetch2  
#f(5.56187 88.5458 -44.9768 -1.1279 -58.6368 -7.30264 -35.244 74.1355 0.294559 2.44973)
<img src="./images/fetch_pose2.png">

fetch3  
#f(5.56952 88.5897 -42.0105 0.87161 -59.8672 3.90341 -76.157 7.51439 0.27191 2.42776)
<img src="./images/fetch_pose3.png">