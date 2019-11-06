### GitHubマニュアル
例：new.lを作った時</br>
初めにやること</br>
`source ~/semi_ws/devel/setup.bash `</br>
`roscd jsk_2019_10_semi`</br>

ブランチをを移動<br>
`git checkout add_jsk_2019_10_semi`</br>

大元の内容を自分のところにもってくる</br>
`git pull origin add_jsk_2019_10_semi`

ブランチを作って移動 (ブランチが存在しない時は -bをつける)</br>
`git checkout (-b) new_branch`</br>

自分のGitを更新（addは初回のみ）</br>
`git add new.l`</br>
`git commit -m "コメント" new.l`</br>
`git push アカウント名 new_branch`

最後に自分のGitのページを開いて自分のアカウントのnew_branchからk-okada/jsk-demosのadd_jsk_2019_10_semiブランチにPullreqを送る。</br>
</br>
### ロボットシミュレーションの基本
#### pepperを使った場合
* viewerのpepperを動かす：\*pepper*<br>
* シミュレーターor実機を動かす：\*ri*</br>

✱roscoreを忘れずに！</br>
<br>
どんなjointがあるのかを調べる</br>
`(send *pepper* :methods :joint)`</br>
全てのパラメータを指定する時</br>
`(send *pepper* :angle-vector #f(...))`</br>
viewerのpepperの今の状態のパラメータを知る</br>
`(send *pepper* :angle-vector)`</br>
上記のパラメータをシミュレーターに送る。</br>
`(send *ri* :angle-vector (send *pepper* :angle-vector))`</br>
実機を動かす場合は何秒かけて行うかも指定</br>
` (send *ri* :angle-vector (send *pepper* :angle-vector) 5000)`</br>
実機の今の状態のパラメータを知る</br>
`(send *ri* :state :potentio-vector)`</br>
逆運動学</br>
` (send *pepper* :rarm :inverse-kinematics (make-coords :pos #f(1000 0 1000)) :revert-if-fail nil)`</br>
✱revert-if-fail nil : 逆運動学に失敗しても解けたところまでで中断する。</br>
<br>
fetchについては以下を参照</br>
[fetchの詳細](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_fetch_robot)


### Emacs豆知識
C-x C-f : ファイル作成</br>
C-x 2 : 画面上下２分割</br>
C-x b \*shell*: shellをLispにする。</br>
C-x o : 画面移動</br>
