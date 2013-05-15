leatra: obj/kinematics.o obj/lfd.o obj/dtw_leatra.o obj/dtw2.o obj/leatra.o obj/leatramain.o obj/litera.o obj/draw.o obj/stringhelp.o
	g++ obj/kinematics.o obj/lfd.o obj/dtw_leatra.o obj/dtw2.o obj/leatra.o obj/leatramain.o obj/litera.o obj/draw.o obj/stringhelp.o -o leatra ; cp leatra ../leatra
obj/dtw_leatra.o: dtw_leatra.cc dtw_leatra.hh
	g++ -c dtw_leatra.cc -o obj/dtw_leatra.o
obj/dtw2.o: dtw2.cc dtw2.hh
	g++ -c dtw2.cc -o obj/dtw2.o
obj/dtw.o: dtw.cc dtw.hh
	g++ -c dtw.cc -o obj/dtw.o
obj/leatra.o: leatra.cc leatra.hh stringhelp.hh
	g++ -c leatra.cc -o obj/leatra.o
obj/leatramain.o: leatramain.cc leatra.hh litera.hh
	g++ -c leatramain.cc -o obj/leatramain.o
obj/draw.o: draw.cc draw.hh leatra.hh stringhelp.hh
	g++ -c draw.cc -o obj/draw.o
obj/litera.o: litera.cc litera.hh leatra.hh stringhelp.hh
	g++ -c litera.cc -o obj/litera.o
obj/stringhelp.o: stringhelp.cc stringhelp.hh
	g++ -c stringhelp.cc -o obj/stringhelp.o
obj/lfd.o: lfd.cc lfd.hh
		g++ -c lfd.cc -o obj/lfd.o
obj/kinematics.o: kinematics.cc kinematics.hh
	g++ -c kinematics.cc -o obj/kinematics.o
