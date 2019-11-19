#Motorの構成
##したい事
    *とりあえず最低限ラジコンのごとく前後左右に動かしたい
    *モータの速度を前方の物体との距離に応じて決定させたい
    *ロボットが目標物に対しまっすぐになるように前進しつつ方向を変えたい
##解決策
    *モータに入力する速度の組み合わせで前後左右に動けるようにする
    *ロボットの位置，ここまで進んだ距離の積分値，対象までの距離の変化量に応じて速度を決定[^1]
    [^1]:いわゆるPID制御~~の真似事~~
    *ロボットの角度についても，座標からずれ角&theta;を求めて距離と同様PID制御
##実装
    ###Motor.py
    ここでは主にラズパイとモータの直接的なやり取りを担当  
    できるだけいじりたくないパラメータについてもここに置いている．~~いじる気を無くすのが目的~~  
    ```Python:Motor.py
    def Run_setting(self,tmpspd,spi_id):
        #print('runboth {}'.format(tmpspd))
        # 方向検出。
        if (tmpspd < 0):
            dir = 0x50
            setspd = -1 * tmpspd
        else:
            dir = 0x51
            setspd = tmpspd

        # 送信バイトデータ生成。
        spd_h   =  (0x0F0000 & setspd) >> 16
        spd_m   =  (0x00FF00 & setspd) >> 8
        spd_l   =  (0x00FF & setspd)

        # コマンド（レジスタアドレス）送信。
        self.Write(dir, spi_id)
        # データ送信。
        self.Write(spd_h, spi_id)
        self.Write(spd_m, spi_id)
        self.Write(spd_l, spi_id)
    ```
    上のメソッドは速度とモータのid(左右)を渡すことでモータにその速度を渡してくれます．方向検出とか何とかは，速度が負だった時にモータを逆向きに回しています．モータを回したいときは速度を書くメソッドで決定し，その値をこのメソッドに投げる，という形式をとっています．

    ```Python:Motor.py
    def Run_forward(self):
        #print('runboth {}'.format(self.speed))
        if self.id==0:
            self.Run_setting(self.speed,0)
        else:
            tmpspd=(-1)*self.speed
            self.Run_setting(tmpspd, 1)
    ```
    ラジコン用メソッド．id==1のとき-1をかけているのは向きの調整．呼び出されると速度をモータに投げる．何秒間持続とかは一切なく，ただ一度だけ速度を入力する．ただレジスタに値が保存されるらしくて，他の入力をしない限り止まらないっぽい(要検証)~~本当はMotor_controlに置いた方がいいと思う~~  
    softstop()は停止命令，softhiz()は正直謎．softstop()の後に使うものらしい．(調べたら停止後にトルクをハイインピーダンス状態にするそうです，なんじゃそら　[ステッピングモータ](http://www.ne.jp/asahi/o-family/extdisk/L6470/L6470_Rev7_DSJP.pdf))

    ###Motor_move.py
    Motor.pyの子クラス．ここは開発者とラズパイの橋渡し的な感じです．ここに動かすためのメソッドを書きます．他の人から隠しておきたい変数もここに格納するのがよさそうです．
    ```Python:Motor_move.py
    #PID制御
    #目標までの距離を受け取り速度を出力する
    def PID(self):
        KP=200
        KI=10
        KD=10
        self.diff.insert(0,self.diff(1))
        self.diff.insert(1,self.dis)
        self.integrald+=(self.diff(0)+self.diff(1))/2.0*self.delta
        p=KP*self.diff(1)
        i=KI*self.integrald
        d=KD*(self.diff(1)-self.diff(0))/self.delta
        if p+i+d>30000:
            return 30000
        elif p+i+d<-30000:
            return -30000
        else:
            return p+i+d
    ```
    このメソッドは距離によって速度を決定するメソッドです．今は距離受け取ってないけど距離を受け取ることによって速度を計算，返します．returnは出力確認のためだけにやっているので後でRun_setting()に変更予定です．pは距離，iは距離の積分結果，dは距離の微分結果です．これらにパラメータをかけることによっていい感じの値をゲットします．いまいちこれらを同時に使う意味が分かっていません．一応手元で調節済みですが実機では未確認．

    ```Python:Motor_move.py
    def Angle(self,x,y):
        ox=50
        oy=50
        KP=500
        KI=10
        KD=10
        sinx=(x-ox)/math.sqrt((x-ox)*(x-ox)+(y-oy)*(y-oy))
        cosx=(x-ox)/math.sqrt((x-ox)*(x-ox)+(y-oy)*(y-oy))
        self.diff.insert(0,self.diff(1))
        self.diff.insert(1,cosx)
        self.integrald+=sinx
        p=KP*self.diff(1)
        i=KI*self.integrald
        d=KD*sinx
        ans=20*(-p-i+d)
        if p+i+d>30000:
            return 30000
        elif p+i+d<-30000:
            return -30000
        else:
            if self.id==0:
                return ans
            else:
                return -1*ans
    ```
    さっきのやつの角度バージョン．x座標とy座標を受け取ると速度を返します．説明含めさっきのとほぼ同じ．カメラ中央の座標が分からないため適当に(50,50)にしています．絶対違うのでそこだけ確認が必要かな，という状況．

    ###その他のファイル
    実行用ファイルです．使う時にこんな感じかなという．

##課題
PID制御ちゃんとやらねば

