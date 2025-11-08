# motion_primitives
Lifecycle Nodes which simply publish /cmd_vel of stop or spin turn etc.




## Test
```
cd ~/ros2_ws
colcon build --packages-select motion_primitives --symlink-install
source install/setup.bash
ros2 launch motion_primitives test_stop_turn.launch.py
```

いいね、2ノードとも起動しています。次は状態確認→`configure/activate`→出力確認の順で動作をチェックしましょう。

まず現在のライフサイクル状態を確認します。

```
ros2 lifecycle get /in_place_turn_node
ros2 lifecycle get /stop_motion_node
```

両方を `inactive`（= configured）にしてから `active` に上げます。いまの launch は configure まで自動ですが、手動でも実行できます。

```
# 必要なら（未configureの場合のみ）
ros2 lifecycle set /in_place_turn_node configure
ros2 lifecycle set /stop_motion_node configure

# アクティブ化
ros2 lifecycle set /in_place_turn_node activate
ros2 lifecycle set /stop_motion_node activate
```

`/cmd_vel` の出力を確認します（別ターミナル）。

```
ros2 topic echo /cmd_vel
```

角速度パラメータの確認と変更（`in_place_turn_node`のみ）:

```
ros2 param get /in_place_turn_node angular_speed
ros2 param set /in_place_turn_node angular_speed 1.2
```

変更後はすぐに `/cmd_vel.angular.z` に反映されます（`active`中でもOK）。

停止を確認したいときは `deactivate` に落とします。

```
ros2 lifecycle set /in_place_turn_node deactivate
ros2 lifecycle set /stop_motion_node deactivate
```

最後に後片付け（任意）。

```
ros2 lifecycle set /in_place_turn_node cleanup
ros2 lifecycle set /stop_motion_node cleanup
```

もし `/cmd_vel` が見えない場合は、`activate` を忘れていないか、あるいは可視化側の QoS が合っているかを確認してください（こちらはデフォルトの `QoS(10)` で publish しています）。
