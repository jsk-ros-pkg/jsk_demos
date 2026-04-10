import time
import launchpad_py as launchpad

def connect_launchpad_x():
    lp = launchpad.LaunchpadLPX()
    # まずはデフォルト
    if lp.Open():
        return lp
    # X は 1番スロットで開く場合がある
    if lp.Open(1):
        return lp
    # 名前指定のフォールバック
    if lp.Open(0, "Launchpad X") or lp.Open(0, "LPX"):
        return lp
    raise RuntimeError("Launchpad X を開けませんでした。USB/MIDI接続を確認してください。")

def led_on(lp, x, y, rgb=(63,0,0)):
    r,g,b = rgb
    lp.LedCtrlXY(x, y, r, g, b)

def led_off(lp, x, y):
    lp.LedCtrlXY(x, y, 0, 0, 0)

if __name__ == "__main__":
    x, y = 3, 5  # 左下(0,0)〜右上(7,7)
    lp = None
    try:
        lp = connect_launchpad_x()
        lp.LedAllOn(0)          # 念のため全消灯
        led_on(lp, x, y, (63,0,0))  # 赤で点灯
        time.sleep(1.0)
        led_off(lp, x, y)
    finally:
        if lp:
            lp.Close()
