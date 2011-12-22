#!/usr/bin/env roseus
#|

http://www.ros.org/wiki/sound_playを使ってサウンド出力するサンプルプログラム


|#

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "sound_play")
(load "package://pr2eus/speak.l")
;;;
(setq ros::*debug* t)
;;;
;;;
(ros::roseus "aques_talk")

;; メッセージをアドバタイズする
(ros::advertise "robotsound_jp" sound_play::SoundRequest 5)
(unix:sleep 1) ;; this is VERY important

;; メッセージのインスタンスを作りパブリッシュする
(setq msg (instance sound_play::SoundRequest :init))

;; 組み込みサウンド
(send msg :sound sound_play::SoundRequest::*needs_plugging*) ;; need_plugging sound
(send msg :command sound_play::SoundRequest::*play_once*) ;; once
(ros::publish "robotsound_jp" msg)
(unix:sleep 2)

;; サウンドファイルの再生 (skype をインストールしておくこと）
(send msg :sound sound_play::SoundRequest::*play_file*) ;; need_plugging sound
(send msg :command sound_play::SoundRequest::*play_once*) ;; once
(send msg :arg "/usr/share/skype/sounds/CallRingingIn.wav")
(ros::publish "robotsound_jp" msg)
(unix:sleep 4)

;; 音声合成を利用する
;;(send msg :sound sound_play::SoundRequest::*say*) ;; say
;;(send msg :command sound_play::SoundRequest::*play_once*) ;; once
;;(send msg :arg "おはようございま'す．きょ'うわ/い'い天気ですか？")
;;(send msg :arg2 "aq_f1c.phont")
;;(send msg :arg2 "aq_m4b.phont")
;;(ros::publish "robotsound" msg)

(speak-jp "こんにちわ。")
(unix:sleep 3)

(speak-jp "おはようございま'す．きょ'うわ/い'い天気ですか？")
(unix:sleep 5)

(exit)
