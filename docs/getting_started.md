# Getting Started

<!--
# ----------------------------------------------------------------------------------------------------------------------
#
#   Setup
#
# ----------------------------------------------------------------------------------------------------------------------
--->
## 開発環境のセットアップ <a id="Setup"></a>

1. Gitのインストール.
```
sudo apt-get update && sudo apt-get install -y git 
```

2. レポジトリをローカルにクローン.
```
git clone https://github.com/Nenetti/emlab_ros_tutorial_2022.git
```

3. 作業ディレクトリをリポジトリに変更
```
cd emlab_ros_tutorial_2022
```

4. 必要なパッケージのインストールを実行 

(docker, docker-compose, nvidia-docker2, Terminator)
```
bash ./SETUP-DEVELOPMENT-ENVIRONMENT.sh
```

5. Dockerイメージをdocker-composeを利用して`./docker/Dockerfile`を基にビルド.
```
bash ./BUILD-DOCKER-IMAGE.sh
```

エラーなくビルド出来ていれば`docker images`コマンドで以下のような出力が得られる．
(REPOSITORYとTAGが合っていれば問題なし)
```
docker images

REPOSITORY           TAG    IMAGE ID       CREATED        SIZE
emlab/tutorial/fetch noetic 742c38df2548   10 minutes ago 15.4GB
```

<!--
# ----------------------------------------------------------------------------------------------------------------------
#
#    Run container
#
# ----------------------------------------------------------------------------------------------------------------------
--->
## コンテナ起動 <a id="Setup"></a>

1. コンテナ起動
```
bash ./RUN-DOCKER-CONTAINER.sh
```

エラーなくコンテナが立ち上がっていれば`docker ps -a`コマンドで以下のような出力が得られる．
(IMAGEとNAMESが合っていれば問題なし)
```
docker ps -a

CONTAINER ID   IMAGE                         COMMAND                  CREATED          STATUS          PORTS    NAMES
f09784f66cda   emlab/tutorial/fetch:noetic   "/usr/local/bin/entr…"   11 minutes ago   Up 11 minutes            emlab-tutorial-client

```

2. コンテナにアクセス
```
docker exec -it -u docker emlab-tutorial-client bash
```

Terminatorを使ってコンテナに複数アクセス
```
bash ./RUN-TERMINATOR.sh
```