# 全面を赤くする最小スクリプト（Launchpad X）
# pip install mido python-rtmidi
import mido

def find_launchpad_port():
    """LPX MIDI Out を優先して探す"""
    names = mido.get_output_names()
    # DAWポートは避ける
    preferred = [n for n in names if 'Launchpad' in n and 'DAW' not in n and ('LPX MIDI' in n or 'MIDI' in n)]
    if preferred:
        return preferred[0]
    # フォールバック
    for n in names:
        if 'Launchpad' in n and 'DAW' not in n:
            return n
    raise RuntimeError(f'Launchpad X の出力ポートが見つかりません。候補: {names}')

# パレット番号で「赤」は 5（明るい赤）
RED_PALETTE = 5  # 公式カラーパレットの5番が赤。ノート/CCの値にそのまま使う。  # :contentReference[oaicite:1]{index=1}

with mido.open_output(find_launchpad_port()) as out:
    # ① Programmer Mode に入る（0x0E, 0x01）
    #   F0 00 20 29 02 0C 0E 01 F7
    out.send(mido.Message('sysex', data=[0x00, 0x20, 0x29, 0x02, 0x0C, 0x0E, 0x01]))  # :contentReference[oaicite:2]{index=2}

    # ② 8x8パッド（11〜88）を全部 赤（Ch.1=静止色）で点灯
    #    ノート番号=パッド位置、ベロシティ=カラーパレット番号  # :contentReference[oaicite:3]{index=3}
    for row in range(1, 9):         # 下→上
        for col in range(1, 9):     # 左→右
            note = row * 10 + col   # 例: 左下=11
            out.send(mido.Message('note_on', channel=0, note=note, velocity=RED_PALETTE))

    # ③ 追加: 上列/右列/下の機能ボタンも赤に（任意）
    #    Programmerレイアウトの CC 番号にパレット値を送る  # :contentReference[oaicite:4]{index=4}
    top_cc    = [91,92,93,94,95,96,97,98,99]      # 上ボタン列
    right_cc  = [19,29,39,49,59,69,79,89]         # 右側ボタン列
    bottom_cc = [8,12,13,14,15,16,17,18]          # 下の機能ボタン
    for cc in top_cc + right_cc + bottom_cc:
        out.send(mido.Message('control_change', channel=0, control=cc, value=RED_PALETTE))
