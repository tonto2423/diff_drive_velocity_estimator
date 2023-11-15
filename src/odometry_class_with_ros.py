# オドメトリを/odomフレームのtfとして出力するクラス
# 2023/11/15
import numpy as np
import tf
import tf.transformations

class Odometry:
    def __init__(self):
        # ブロードキャスターのインスタンスを生成
        self.br = tf.TransformBroadcaster()        
        # ロボットのパラメータ
        self.radius = 0.02504               # 車輪半径[m]
        self.resolution = 400               # エンコーダ分解能[カウント/回転]
        self.pi = 3.1415926535              # 円周率
        self.length = 0.16218               # タイヤの配置半径[m]
        self.x = 0.0                        # ロボットのx座標[m]
        self.y = 0.0                        # ロボットのy座標[m]
        self.theta = 0.0                    # ロボットの向き[rad]
        self.counts = np.array([0, 0, 0])   # エンコーダのカウント値

    # エンコーダのパルスカウントからロボットの変位を計算する関数
    def calculate_velocity(self, count1, count2, count3):
        Translate_Matrix = self.radius * 2.0 * self.pi / 3.0 / self.resolution * np.array([
            [-1, 0.5, 0.5],
            [0, -0.866025, 0.866025],
            [1.0 / self.length, 1.0 / self.length, 1.0 / self.length]
        ])
        delta_x, delta_y, delta_theta = np.dot(Translate_Matrix, np.array([count1, count2, count3]) - self.counts)
        # ひとつ前のthetaだけ回転する
        Rotate_Matrix = np.array([[np.cos(self.theta), -np.sin(self.theta)],
                                  [np.sin(self.theta), np.cos(self.theta)]])
        delta_x, delta_y = 2 * np.dot(Rotate_Matrix, np.array([delta_x, delta_y])) # 2倍の補正をかける
        # 前回変数の更新
        self.update_counts(count1, count2, count3)
        return delta_x, delta_y, delta_theta

    def calculate_pose(self, delta_x, delta_y, delta_theta):
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = (self.theta + self.pi) % (2 * self.pi) - self.pi
        return self.x, self.y, self.theta

    # エンコーダのカウント値を更新する関数
    def update_counts(self, count1, count2, count3):
        self.counts = np.array([count1, count2, count3])
    
    # オドメトリのtfを送信する関数
    def tf_broadcast(self):
        # Transformを送信
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, self.theta),
            rospy.Time.now(),
            "base_link",
            "odom"
        )