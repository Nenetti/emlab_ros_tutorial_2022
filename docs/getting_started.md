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

5. Dockerイメージを`./docker/Dockerfile`からビルド.
```
bash ./BUILD-DOCKER-IMAGE.sh
```

エラーなくビルド出来ていれば`docker images`コマンドで以下の出力が得られる．
```
docker images

REPOSITORY              TAG       IMAGE ID       CREATED          SIZE
emlab/tutorial/fetch    noetic    a378cbe7a2f8   XX minutes ago   15GB
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

2. コンテナが立ち上がっていれば，`docker ps -a`コマンドで以下の出力が得られる．
```
CONTAINER ID   IMAGE          COMMAND                  CREATED         STATUS    PORTS     NAMES
xxxxxxxxxxxx   fetch:noetic   "/usr/local/bin/entr…"   7 minutes ago   Created             fetch-client
yyyyyyyyyyyy   fetch:noetic   "/usr/local/bin/entr…"   7 minutes ago   Created             fetch-simulator
```

3. コンテナにアクセス
```
bash ./RUN-TERMINATOR.sh
```