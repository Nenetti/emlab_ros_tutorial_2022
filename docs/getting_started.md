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

コマンドは以下のどれでも同じ結果になる．(カレントディレクトリは`emlab_ros_tutorial_2022`とする)
```
1. docker-compose -f ./docker/docker-compose.yml build
2. docker build ./docker -t emlab/tutorial/fetch:noetic
2. docker build ./docker -f ./docker/Dockerfile -t emlab/tutorial/fetch:noetic

4. cd ./docker && docker-compose build
5. cd ./docker && docker build ./ -t emlab/tutorial/fetch:noetic
6. cd ./docker && docker build ./ -f Dockerfile -t emlab/tutorial/fetch:noetic

7. bash ./BUILD-DOCKER-IMAGE.sh
```

エラーなくビルド出来ていれば`docker ps -a`コマンドで以下の出力が得られる．
```
docker ps -a

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
コマンドは以下のどれでも同じ結果になる．(カレントディレクトリは`emlab_ros_tutorial_2022`とする)
```
bash ./RUN-DOCKER-CONTAINER.sh
```

2. コンテナにアクセス
コマンドは以下のどれでも同じ結果になる．(カレントディレクトリは`emlab_ros_tutorial_2022`とする)
```
bash ./RUN-DOCKER-CONTAINER.sh
```