"""受信した画像の時刻に合わせて、動画に何フレーム書き込むかを決める(ROS 非依存).

カメラ画像は、PC が遅いと 30 fps ではなく数 fps しか届かないことがある。受信したフレームを
1 枚ずつ書き込むだけだと、動画が実際より極端に短く(早送りに)なる。そこで画像の時刻
(シミュレーション時間)を基準に、足りないフレームは直前の画像を繰り返して埋め、
動画の長さ = 撮影していた時間 になるようにする。
"""


class FramePacer:

    def __init__(self, fps, max_repeat_seconds=2.0):
        self.fps = float(fps)
        self.max_repeat = max(1, int(round(self.fps * max_repeat_seconds)))
        self.t0 = None
        self.written = 0

    def frames_for(self, stamp_s):
        """時刻 stamp_s [s] の画像を何回書き込むか(0 なら書かない).

        - 最初の画像は 1 回
        - 以降は、その時刻までに必要なフレーム数との差だけ書く
          (時刻が戻った・同じ時刻の画像は 0 回)
        - 長く途切れた場合(ポーズなど)は max_repeat_seconds 分で打ち切り、基準時刻をずらす
        """
        if self.t0 is None:
            self.t0 = stamp_s
            self.written = 1
            return 1
        target = int(round((stamp_s - self.t0) * self.fps)) + 1
        n = target - self.written
        if n <= 0:
            return 0
        if n > self.max_repeat:
            # 途切れた分を詰める(以降の時刻の基準をずらす)
            self.t0 += (n - self.max_repeat) / self.fps
            n = self.max_repeat
        self.written += n
        return n
