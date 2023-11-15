# COBSデコード関数をクラスにまとめる
from cobs import cobs
import struct

class CobsClass:
    def __init__(self):
        pass

    # データを分割する関数
    def split_data(self, data):
        messages = []
        start = 0

        for i in range(len(data)):
            if data[i] == 0x00:
                messages.append(data[start:i])  # 0x00 までのデータを抽出
                start = i + 1

        return messages

    # COBSデコード関数
    def decode_cobs_data(self, encoded_data):
        try:
            # COBSデコード
            decoded_data = cobs.decode(encoded_data)

            # デコードされたデータからカウント値と速度値を抽出
            # 先頭の識別子バイト0x01を除外
            decoded_data = decoded_data[1:]
            # print(decoded_data)

            # カウント値の抽出：3つの符号付き32ビット整数
            counts = struct.unpack('<3i', decoded_data[:12])

            # 速度値の抽出：3つの符号付き16ビット固定小数点
            speed_bytes = decoded_data[12:]
            speeds = [struct.unpack('<h', speed_bytes[i:i+2])[0] / 16.0 for i in range(0, len(speed_bytes), 2)]

            return counts, speeds
        except cobs.DecodeError:
            print("COBSデコードエラーが発生しました。")
            return None, None
        except struct.error as e:
            print("データのアンパック中にエラーが発生しました:", e)
            return None, None