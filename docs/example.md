# Example

<!--
# ----------------------------------------------------------------------------------------------------------------------
#
#    Gazebo 起動
#
# ----------------------------------------------------------------------------------------------------------------------
--->
## Fetch
Terminatorなどを使って複数のターミナルをターミナル毎に各コマンドを実行する．

### Gazebo 起動
```
roslaunch fetch_gazebo playground.launch 
```

### Rviz表示
```
roslaunch em_fetch_rviz rviz.launch
```


### 自己位置推定
```
roslaunch fetch_gazebo_demo fetch_nav.launch
```