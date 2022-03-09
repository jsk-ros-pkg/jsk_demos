## euslisp directory structure

- Programs in `demo` directory is for PR2 to do demos. These programs load only programs in `motion` directory.

- Programs in `motion` directory is PR2's primitives for cooking. These programs load only programs in `util` and `model` directory.
  - プログラムの命名規則の説明
  - arrangement-at-arrange.l
    - arrangement motions at arrange position
    - For example, `(scoop-up-curry)`
  - ih-at-arrange.l
    - IH manipulation motions at arrange position
    - For example, `(push-knob)`
  - tool-at-arrange.l
    - tool manipulation motions at arrange position
    - For example, `(open-shelf)`
  - move-to-kitchen-with-map.l
    - navigation motions in the kitchen
    - If you go to arrange position, call `(move-to-arrange-ri)`
  - interaction.l
    - speech interaction with the robot.
    - 「OKと合図をしてください」と言われる -> OKと答えるまでPR2は待つ
    - それ以外の疑問形 -> 「はい」 or 「いいえ」で答える -> 「いいえ」の場合、動作をやり直す

- Programs in `util` directory is PR2's general functions. For example, touch sensing and human interaction.
- Programs in `model` directory is for euslisp modelsfor cooking.
