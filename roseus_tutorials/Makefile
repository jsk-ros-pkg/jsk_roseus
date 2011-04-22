include $(shell rospack find mk)/cmake.mk


# download images from http://www.boj.or.jp/note_tfjgs/note/valid/issue.htm/#p04
download: img/1000yen.jpg img/2000yen.jpg img/5000yen.jpg img/10000yen.jpg
img/1000yen.jpg:
	wget http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_1000f_e.jpg -O $@
img/2000yen.jpg:
	wget http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_2000f_d.jpg -O $@
img/5000yen.jpg:
	wget http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_5000f_e.jpg -O $@
img/10000yen.jpg:
	wget http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_10000f_e.jpg -O $@
