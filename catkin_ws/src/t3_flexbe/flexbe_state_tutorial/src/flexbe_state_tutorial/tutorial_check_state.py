#!/usr/bin/env python
import numpy as np
import rospy

from flexbe_core import EventState
from flexbe_core.core import UserData


class TutorialInputState(EventState):
    """
    ここにこのステートの説明を書く．
    ここに書いたものはFlexbe AppのGUIにて表示される．日本語も対応している．
    （なお上記の説明もGUI上に載るので注意．）

    This is sample state for the beginner.

    以下はこのステートへの引数，出力，出力後の経路に関する情報を記載しておく．
    ちなみにこの部分はGUIでも表示されないし，開発の際のコメント以上の役割はなし．

    -- init_arg     int 		__init__ への引数
    -- init_arg2 	string 		こんな引数は__init__で定義していない．GUIでも表示されないことを確認するようで，
                                開発時はこういったものを残さないように．引き継ぎの人の勘違いの元になる．

    以下はenter, execute, exit関数から他ステート向けに設定する変数の出力．(ちなみに #>が入力を表す．)
    GUI上からこのステートで用いたい変数を指定すると，他のステートなどから出力された同名の変数を利用出来る．
    (もちろん，このステートを通る前に，参照する変数を出力するステートを通るとnot definedでエラーになる．)

    #> input_str_data 	str 	enter, execute, exit関数で使用できる外部ステートからの文字列の入力．
    #> input_int_data 	int 	enter, execute, exit関数で使用できる外部ステートからの整数の入力．
    #> input_dummy   	str 	これも確認用のダミー

    以下はenter, execute, exit関数から他ステート向けに設定する変数の出力．(ちなみに >#が出力を表す．)
    GUI上で他ステートからの引数にこの変数名を指定すると，他のステートがこの変数を利用出来る．
    (もちろん，このステートを通る前に，参照するステートを通るとnot definedでエラーになる．)

    ># output_list      List            リスト型の出力．
    ># output_ndarray   np.ndarray      numpyのndarray型の出力
    #> output_dummy   	str 	        これも確認用のダミー

    以下はこのステートの結果を書いておく(ちなみに<=で結果を表す)．
    execute関数がreturnする文字列であり，このステートの結果を示す．
    よくあるのは，doneとfailedの2つ．
    doneだったら次にどのステートに移動する，failedだったらこっちのステート．のようにこのステートの次のステートが経路になる．

    <= route1           経路1
    <= route2 			経路2
    <= ROUTE3 		    経路3
    <= ROUTE4 			経路4

    References:
        http://wiki.ros.org/flexbe/Tutorials/Developing%20Basic%20States
        http://wiki.ros.org/flexbe/Tutorials/The%20State%20Lifecycle
    """

    def __init__(self, init_arg):
        """
        Constructor
        ここでは以下の3つの要素を基底クラスの初期化引数として定義する．
        1. このステートから派生する経路 (outcomes)
        2. このステートへの入力変数 (input_keys)
        3. このステートからの出力変数 (output_keys)

        これらはすべて文字列型が要素のListで定義する．
        ちなみにinput_keysとoutput_keysはUserDataというクラスによって管理されるが，変数名通り辞書型とほぼ同じ挙動をする．

        ちなみに，__init__には，普通にインスタンス変数などを定義してもOK．cv_bridgeやROSのpub, subもここに定義してOK．
        ただし ActionClientなどのwait_for_serverなどはon_enterで行うことを推奨．

        """
        super(TutorialInputState, self).__init__(
            outcomes=["route1", "route2", "ROUTE3", "ROUTE4"],
            input_keys=["input_str_data", "input_int_data"],
            output_keys=["output_list", "output_ndarray"]
        )
        rospy.loginfo(f"__init__: {init_arg}")

    # ==================================================================================================
    #
    #   Flexbe Methods
    #
    # ==================================================================================================
    def on_start(self) -> None:
        """
        プログラム開始時に1度だけ呼ばれる関数．
        すべてのステートの __init__ が終わった後に呼ばれると思って相違なし．
        一般的にはActionClientのwait_for_serverなどの接続待機確認などで用いられる．

        """
        rospy.loginfo(f"on_start")

    def on_enter(self, userdata: UserData) -> None:
        """
        ステートに入った最初の1回目に呼び出される．
        なお，他ステートから呼び出されるときにだけ呼び出される．

        """
        rospy.loginfo(f"on_enter")

    def execute(self, userdata: UserData) -> str:
        """
        このステートでのメイン処理．
        ここで以降の経路を決定する．

        このプログラムでは，他ステートから得た，input_int_data の値を元に次の経路を決定している．

        """
        rospy.loginfo(f"execute")
        input_int_data: int = userdata.input_int_data
        if input_int_data == 1:
            userdata.output_list = [1, 1]
            return "route1"

        if input_int_data == 2:
            userdata.output_list = [1, 2]
            return "route2"

        if input_int_data == 3:
            userdata.output_list = [1, 3]
            return "ROUTE3"

        if input_int_data == 4:
            userdata.output_list = [1, 4]
            return "ROUTE4"

        raise ValueError(f"Got unexpected value {input_int_data}")

    def on_exit(self, userdata: UserData) -> None:
        """
        このステートの後処理．(終了処理に近い)
        ステート自体は終了しない．(組み方次第で再び呼ばれるため．)
        on_enterでpublisherやsubscriberを定義する場合はここで，unregisterなどの処理を行ったりする．

        """
        userdata.output_ndarray = np.array([1, 2])
        rospy.loginfo(f"on_exit")

    # 以下はそこまで重要ではないので説明は省く．
    # 気になる場合はWikiを参照すべし
    def on_pause(self) -> None:
        rospy.loginfo(f"on_pause")

    def on_resume(self, userdata: UserData) -> None:
        rospy.loginfo(f"on_resume")

    def on_stop(self):
        rospy.loginfo(f"on_stop")
