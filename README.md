# 9DOFセンサとRaspberry Pi Picoを用いたクォータニオンを用いた拡張カルマンフィルタによる姿勢とジャイロバイアスの推定
An Extended Kalman Filter sample for Quaternion-Based Orientation Estimation Using 9DOF Sensors and Raspberry Pi Pico

### 進捗
Raspberry Pi Pico とLSM9DS1を用いた拡張カルマンフィルタによるクォータニオンとジャイロバイアスの推定をするコードを置いておきます。姿勢の推定については遅れやオーバーシュートの改善など見直し中。ジャイロバイアスの推定はまだ十分に検証できていません。（2021/10/17時点）

### サンプルデータ
data_sample.txtは取得したデータのサンプルです。CSV形式になっています。
１列目から以下の様にデータが並んでいます。
1. 時刻(s)
2. クォータニオン推定値 q0
3. クォータニオン推定値 q1
4. クォータニオン推定値 q2
5. クォータニオン推定値 q3
6. ジャイロバイアス推定値(ロール) p(rad/s)
7. ジャイロバイアス推定値(ピッチ) q(rad/s)
8. ジャイロバイアス推定値(ヨー) r(rad/s)
9. Raspberry Pi Pico EKF計算時間(us)(ログ表示時間含んでいます。ログ表示時間を引くとほぼEKF計算時間です。)
10. Raspberry Pi Pico ログ表示時間(us)
11. 重力加速度計測値(x)(m/s^2)
12. 重力加速度計測値(y)(m/s^2)
13. 重力加速度計測値(z)(m/s^2)
14. 角速度計測値（ロール）(rad/s)
15. 角速度計測値（ピッチ）(rad/s)
16. 角速度計測値（ヨー）(rad/s)
17. 地磁気計測値（x）（校正済）(ノルムを１に正規化)
18. 地磁気計測値（y）（校正済）(ノルムを１に正規化)
19. 地磁気計測値（z）（校正済）(ノルムを１に正規化)

### ブログ 
関連ブログを書いています

- https://rikei-tawamure.com/entry/2021/09/19/153553
- https://rikei-tawamure.com/entry/2021/09/27/111205
- https://rikei-tawamure.com/entry/2021/10/07/211725
- https://rikei-tawamure.com/entry/2021/08/28/172556
