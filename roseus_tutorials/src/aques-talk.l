#!/usr/bin/env roseus
#|

http://www.ros.org/wiki/sound_playを使ってサウンド出力するサンプルプログラム


|#

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "sound_play")
;;;
(setq ros::*debug* t)
;;;
;;;
(ros::roseus "aques_talk")

;; メッセージをアドバタイズする
(ros::advertise "robotsound" sound_play::SoundRequest 5)
(unix:sleep 1) ;; this is VERY important

;; メッセージのインスタンスを作りパブリッシュする
(setq msg (instance sound_play::SoundRequest :init))

;; 組み込みサウンド
(send msg :sound sound_play::*needs_plugging*) ;; need_plugging sound
(send msg :command sound_play::*play_once*) ;; once
(ros::publish "robotsound" msg)
(unix:sleep 2)

;; サウンドファイルの再生 (skype をインストールしておくこと）
(send msg :sound sound_play::*play_file*) ;; need_plugging sound
(send msg :command sound_play::*play_once*) ;; once
(send msg :arg "/usr/share/skype/sounds/CallRingingIn.wav")
(ros::publish "robotsound" msg)
(unix:sleep 4)

;; 音声合成を利用する
(send msg :sound sound_play::*say*) ;; say
(send msg :command sound_play::*play_once*) ;; once
(send msg :arg "おはようございま'す．きょ'うわ/い'い天気ですか？")
(ros::publish "robotsound" msg)
(unix:sleep 5)

(exit)
