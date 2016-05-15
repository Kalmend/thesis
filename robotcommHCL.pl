:- dynamic cc0/1,cc1/1,cc2/1,cc2TXT/1,cc3/1,cc3TXT/1,ccx/1.
:- dynamic world/5.

%---------------- Tests: -------------------
	command([juhata, gert,koridori]).
	command([juhata, gert,tuppa,nelisada,kakskümmend]).
	command([ütle,kus,asub,tass]).
	command([ütle,kus,tass, asub]).
	command([ole,toas,nelisada,kakskümmend]).
	command([ole,kohvitoas,kell,viis,viiskümmend,viis]).
	command([tule,kohe,siia]).
	command([tule,ukse,juurde]).
	command([leia, kohe, koridorist, gert,üles]).
	command([anna,see, raamat,evelinile]).
	command([võta, evelinilt, see, raamat]).
	command([pane, tass, laua, peale]).
	command([too,see,tass,laualt,evelinile]).
	command([too,see,tass,evelinile,laualt]).
	command([too,evelinile,laualt,tass]).
	command([too,laualt,evelinile,tass]).
	command([vii,see,tass,evelini,kätte]).      % failed
	command([vii,see,tass,kohvituppa]).
	command([mine,kiiresti,evelini,juurde]).
test1:-
	command(Txt), write('KÄSK: '),write(Txt),nl,write('ROBOTI TEGEVUS:'),nl,
	käsk(Txt,_),nl,fail.
test1.

%------------------------------------------
%----------------------------------------------- DCG Interpreteerimisreeglid -----------------------------------------------------
käsk([juhata|_G2210],_G2221):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(isik_1(_G2210,_G2312),update(cc0,_G2210);_G2210 =_G2312),
	update(cc2,_G2312),
	(koht1_4(_G2312,_G2221);_G2312=_G2221),
	reaction(juhata),!.
käsk([ütle, kus|_G2202],_G2213):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	verb2(_G2202,_G2250),
	(objekt1(_G2250,_G2213),update(cc1,_G2250),update(ccx,_G2250);
	objekt21(_G2250,_G2325),update(cc2,_G2250),update(ccx,_G2250)),
	reaction(ütle_kus1),!.
käsk([ütle,kus|_G2192],_G2203):-	% isiku puhul
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(verb2(_G2192,_G2240),isik_1(_G2240,_G2203),update(cc0, _G2240);
	isik_1(_G2192,_G2240),verb2(_G2240,_G2203),update(cc0, _G2192)),
	reaction(ütle_kus3),!.
käsk([ütle, kus|_G2206],_G2217):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(objekt1(_G2206,_G2325), update(cc1, _G2206),update(ccx,_G2206);
	objekt21(_G2206,_G2325),update(cc2,_G2206),update(ccx,_G2206)),
	verb2(_G2325,_G2217),
	reaction(ütle_kus1),!.
käsk([ole|_G2220],_G2231):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(koht1_50(_G2220,_G2342);
	((_G2220=[siin|_G2322];_G2220=_G2322),koht1_5(_G2322,_G2342),update(ccx, _G2322))),
	(ajamäärus(_G2342,_G2231);_G2342=_G2231),
	reaction(ole),!.
käsk([tule|_G2210],_G2221):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(viisimäärus1(_G2210,_G2312);ajamäärus(_G2210,_G2312);_G2210=_G2312),
	(kohamäärus1(_G2312,_G2221);isik_2(_G2312,_G2223),update(cc0,_G2312),kohamäärus3(_G2223,_G2221)),
	(ajamäärus(_G2221,_G2222);_G2221=_G2222),
	reaction(tule1),!.
käsk([tule|_G2223],_G2234):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(viisimäärus1(_G2223,_G2396);ajamäärus(_G2223,_G2396);_G2223=_G2396),
	(objekt2(_G2396,_G2467),update(cc2,_G2396),
	kohamäärus3(_G2467,_G2234);_G2396=_G2234),
	reaction(tule2),!.
käsk(_G2271,_G2282):-					% Leia|otsi
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(_G2271=[leia|_G2373];_G2271=[otsi|_G2373]),	%
	(isik_1(_G2373,_G2464),update(cc0,_G2373);	% kes
	objekt1(_G2373,_G2464), update(cc1,_G2373)),	% või mis
	update(ccx,_G2373),
	(viisimäärus1(_G2464,_G2555);_G2464=_G2555),	% kuidas
	update(cc2,_G2555),
	(koht1_6(_G2555,_G2717);koht1_9(_G2555,_G2717);_G2555=_G2717), % mis kohast
	(_G2717=[üles|_G2282];_G2717=_G2282),
	reaction(otsi_leia),!.
käsk(_G2271,_G2282):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(_G2271=[leia|_G2373];_G2271=[otsi|_G2373]),	% Leia|otsi
	(viisimäärus1(_G2373,_G2555);_G2373=_G2555),	% kuidas
	update(cc2,_G2555),
	(koht1_6(_G2555,_G2717);koht1_9(_G2555,_G2717);_G2555=_G2717), update(ccx,_G2717),% mis kohast
	(isik_1(_G2717,_G2464), update(cc0,_G2717);	% kes
	objekt1(_G2717,_G2464), update(cc1,_G2717)),	% või mis
	(_G2464=[üles|_G2282];_G2464=_G2282),
	reaction(otsi_leia),!.
%käsk([tõuse|_G2201],_G2212):- (_G2201=[üles|_G2303];_G2201=_G2303), (ajamäärus(_G2303,_G2212);_G2303=_G2212).
%käsk([tõuse|_G2225],_G2236):- (ajamäärus(_G2225,_G2398);viisimäärus1(_G2225,_G2398);_G2225=_G2398), (_G2398=_G2236;_G2398=[üles|_G2236];_G2398=[püsti|_G2236]).
%käsk([vaata|_G2168],_G2179):- kohamäärus1(_G2168,_G2179).
%käsk([hakka|_G2187],_G2198):- verb1(_G2187,_G2198);roll(_G2187,_G2198).
käsk(_G2268,_G2279):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(_G2268=[anna|_G2370];_G2268=[ulata|_G2370]),
	(_G2370=[see|_G2461];_G2370=_G2461),
	(objekt1(_G2461,_G2552),update(cc1,_G2461);_G2461=_G2552),
	(_G2552=[siia|_G2279];
	asesõna_7(_G2552,_G2279);
	isik_7(_G2552,_G2279),update(cc0,_G2552);
	_G2552=_G2279),
	reaction(anna_ulata),!.
käsk(_G2270,_G2281):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(_G2270=[võta|_G2372];_G2270=[haara|_G2372]),
	(isik_9(_G2372,_G2230);_G2372=_G2230),
	(_G2230=[see|_G2463];_G2230=_G2463),
	(objekt1(_G2463,_G2554),update(cc1,_G2463);_G2463=_G2554),
	((_G2554=[oma|_G2622];_G2554=[enda|_G2622]),_G2622=[kätte|_G2281];_G2554=[endale|_G2281];_G2554=_G2281),
	reaction(võta_haara),!.
käsk([A|_G2321],_G2332):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(A=tõsta;A=pane;A=aseta),
	(_G2321=[see|_G2526];_G2321=[see|_G2477],objekt1(_G2477,_G2526),update(cc1,_G2477);objekt1(_G2321,_G2526),update(cc1,_G2321)),
	(_G2526=[üles|_G2332];_G2526=[sinna|_G2332];_G2526=[lauale|_G2332];objekt2(_G2526,_G2747),update(cc2,_G2526),
	(_G2747=[peale|_G2332];_G2747=[kõrvale|_G2332];_G2747=[juurde|_G2332];_G2747=[alla|_G2332])),
	reaction(tõsta_pane),!.
käsk([too|_G2261],_G2272):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(_G2261=[see|_G2466];_G2261=_G2466),
	(objekt1(_G2466,_G2631),update(cc1,_G2466);_G2466=_G2631),
	(koht1_6(_G2631,_G2272);koht1_9(_G2631,_G2272),update(cc2,_G2631);_G2631=_G2272),
	(asesõna_7(G2272,_G2632);isik_7(_G2272,_G2632),update(cc0,_G2272);_G2272=_G2632),
	(koht1_6(_G2632,_G2273);koht1_9(_G2632,_G2273),update(cc2,_G2632);_G2632=_G2273),
	reaction(too),!.
käsk([too|_G2246],_G2257):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(_G2246=[siia|_G2490]; asesõna_7(_G2246,_G2490),update(cc0,_G2246); isik_7(_G2246,_G2490),update(cc0,_G2246);_G2246=_G2490),
	(koht1_6(_G2490,_G2658); koht1_9(_G2490,_G2658), update(cc2,_G2490); _G2490=_G2658),
	(objekt1(_G2658,_G2257),update(cc1,_G2658); _G2658=_G2257),
	reaction(too),!.
käsk([too|_G2231],_G2242):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(koht1_6(_G2231,_G2404);koht1_9(_G2231,_G2404),update(cc2,_G2231);_G2231=_G2404),
	(asesõna_7(_G2404,_G2569);isik_7(_G2404,_G2569),update(cc0,_G2404);_G2404=_G2569),
	(objekt1(_G2569,_G2242),update(cc1,_G2569);_G2569=_G2242),
	reaction(too),!.
käsk([vii|_G2295],_G2306):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(_G2295=[see|_G2397];_G2295=_G2397),
	(objekt1(_G2397,_G2488),update(cc1,_G2397);_G2397=_G2488),
	(_G2488=[ära|_G2306],update(cc2,[ära]);
	 asesõna_7(_G2488,_G2306),update(cc0,_G2488);
	 isik_7(_G2488,_G2306),update(cc0,_G2488),update(ccx,_G2488);
	 isik_2(_G2488,_G2754),_G2754=[kätte|_G2306],update(cc0,_G2488),update(ccx,_G2488);
	 koht1_4(_G2488,_G2306);
	 koht1_11(_G2488,_G2306);
	 _G2488=_G2306),
	update(cc3,[now]),
	reaction(vii),!.
% käsk([jää|_G2237],_G2248):-_G2237=[siia|_G2248]; (viisimäärus1(_G2237,_G2453);_G2237=_G2453), (_G2453=[paigale|_G2248];_G2453=[seisma|_G2248]);_G2237=[otsima|_G2248].
%käsk([peatu|_G2156],_G2156):-true.
%käsk(_G2272,_G2283):- (_G2272=[keera|_G2374];_G2272=[pööra|_G2374]), (_G2374=[ümber|_G2283];_G2374=[tagasi|_G2283]; (arvsõna1(_G2374,_G2586),_G2586=[kraadi|_G2635];_G2374=_G2635), (_G2635=[paremale|_G2283];_G2635=[vasakule|_G2283])).
%käsk(_G2276,_G2287):- (_G2276=[keera|_G2378];_G2276=[pööra|_G2378]), (_G2378=[ümber|_G2287];_G2378=[tagasi|_G2287]; (_G2378=[paremale|_G2738];_G2378=[vasakule|_G2738];_G2378=_G2738),arvsõna1(_G2738,_G2761),_G2761=[kraadi|_G2287];_G2378=_G2287).
käsk(_G2267,_G2278):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(_G2267=[liigu|_G2369];_G2267=[mine|_G2369]),
	(viisimäärus1(_G2369,_G2531);ajamäärus(_G2369,_G2531);_G2369=_G2531),
	(kohamäärus1(_G2531,_G2278); (objekt2(_G2531,_G2739),update(cc2,_G2531);isik_2(_G2531,_G2739),update(cc0,_G2531)), update(ccx,_G2531),
	(_G2739=[juurde|_G2278];_G2739=[poole|_G2278]);_G2531=_G2278),reaction(liigu_mine),!.


verb1(_G2219,_G2230):-_G2219=[sõitma|_G2230];_G2219=[liikuma|_G2230];_G2219=[minema|_G2230];_G2219=[tulema|_G2230];_G2219=[otsima|_G2230].
verb2(_G2219,_G2230):-_G2219=[asub|_G2230];_G2219=[seisab|_G2230];_G2219=[asetseb|_G2230];_G2219=[paikneb|_G2230];_G2219=[viibib|_G2230];_G2219=[on|_G2230].

objekt1(_G2249,_G2260):-_G2249=[raamat|_G2260];_G2249=[pall|_G2260];_G2249=[asi|_G2260];_G2249=[tass|_G2260];_G2249=[pliiats|_G2260];_G2249=[õlu|_G2260];_G2249=[kohv|_G2260];_G2249=[tee|_G2260].
objekt12(_G2209,_G2220):-_G2209=[palli|_G2220];_G2209=[asja|_G2220];_G2209=[tassi|_G2220];_G2209=[pliiatsi|_G2220].
objekt21(_G2204,_G2215):-_G2204=[laud|_G2215];_G2204=[tool|_G2215];_G2204=[uks|_G2215];_G2204=[aken|_G2215].
objekt2(_G2204,_G2215):-_G2204=[laua|_G2215];_G2204=[tooli|_G2215];_G2204=[ukse|_G2215];_G2204=[akna|_G2215].

isik_1(_G2219,_G2230):-_G2219=[evelin|_G2230];_G2219=[gert|_G2230];_G2219=[marko|_G2230];_G2219=[oliver|_G2230];_G2219=[külaline|_G2230];_G2219=[teadur|_G2230].
isik_2(_G2264,_G2275):-_G2264=[evelini|_G2275];_G2264=[gerdi|_G2275];_G2264=[marko|_G2275];_G2264=[oliveri|_G2275];_G2264=[külalise|_G2275];_G2264=[minu|_G2275];_G2264=[tema|_G2275];_G2264=[nende|_G2275].
isik_7(_G2219,_G2230):-_G2219=[evelinile|_G2230];_G2219=[gerdile|_G2230];_G2219=[markole|_G2230];_G2219=[oliverile|_G2230];_G2219=[minule|_G2230].
isik_9(_G2219,_G2230):-_G2219=[evelinilt|_G2230];_G2219=[gerdilt|_G2230];_G2219=[markolt|_G2230];_G2219=[oliverilt|_G2230];_G2219=[minult|_G2230];_G2219=[talt|_G2230].

asesõna_7(_G2234,_G2245):-
	_G2234=[mulle|_G2245];
	_G2234=[talle|_G2245];
	_G2234=[meile|_G2245];
	_G2234=[teile|_G2245];
	_G2234=[neile|_G2245];
	_G2234=[endale|_G2245].

koht1_4(_G2209,_G2220):-
	update(cc2TXT,_G2209),
	(_G2209=[kohvituppa|_G2220],update(cc2,[kohvituba]),update(ccx,[kohvituba]);
	_G2209=[koridori|_G2220],update(cc2,[koridor]),update(ccx,[koridor]);
	_G2209=[tuppa,number|_G2372],room_number(_G2372,_G2220),retract(cc2TXT(T)),asserta(cc2TXT([tuppa|T]));
	_G2209=[tuppa|_G2372],room_number(_G2372,_G2220),retract(cc2TXT(T)),asserta(cc2TXT([tuppa|T]))).
koht1_5(_G2176,_G2187):-
	(objekt12(_G2176,_G2278);objekt2(_G2176,_G2278)), kohamäärus2(_G2278,_G2187).
koht1_50(_G2209,_G2220):-
	_G2209=[kohvitoas|_G2220],update(cc2,[kohvituba]),update(ccx,[kohvituba]),update(cc2TXT,[kohvitoas]);
	_G2209=[koridoris|_G2220],update(cc2,[koridor]),update(ccx,[koridor]),update(cc2TXT,[koridoris]);
	_G2209=[toas,number|_G2372],room_number(_G2372,_G2220),retract(cc2TXT(T)),asserta(cc2TXT([toas|T]));
	_G2209=[toas|_G2372],room_number(_G2372,_G2220),retract(cc2TXT(T)),asserta(cc2TXT([toas|T])).
koht1_6(_G2213,_G2224):-
	update(cc2TXT,_G2213),
	(_G2213=[kohvitoast|_G2224],update(cc2,[kohvituba]);
	_G2213=[koridorist|_G2224],update(cc2,[koridor]);
	_G2213=[toast,number|_G2405],room_number(_G2405,_G2224),retract(cc2TXT(T)),asserta(cc2TXT([toast|T]));
	_G2213=[toast|_G2405],room_number(_G2405,_G2224)),retract(cc2TXT(T)),asserta(cc2TXT([toast|T])).
koht1_9(_G2249,_G2260):-
	_G2249=[põrandalt|_G2260];
	_G2249=[laualt|_G2260];
	_G2249=[seinalt|_G2260];
	_G2249=[ukselt|_G2260];
	_G2249=[aknalt|_G2260];
	_G2249=[toolilt|_G2260];
	_G2249=[diivanilt|_G2260].
koht1_11(_G2219,_G2230):-
	update(cc2TXT,_G2219),
	_G2219=[kohvitoani|_G2230],update(cc2,[kohvituba]);
	_G2219=[koridorini|_G2230],update(cc2,[koridor]);
	_G2219=[ukseni|_G2230];
	_G2219=[toani,number|_G2448],room_number(_G2448,_G2230),retract(cc2TXT(T)),asserta(cc2TXT([toani|T]));
	_G2219=[toani|_G2448],room_number(_G2448,_G2230),retract(cc2TXT(T)),asserta(cc2TXT([toani|T])).

room_number(_G2192,_G2203):-
	_G2192=[nelisada,üheksateist|_G2203], update(cc2,[ruum_419]),update(ccx,[ruum_419]), update(cc2TXT,[nelisada,üheksateist]);	% jüri
	_G2192=[nelisada,kakskümmend|_G2203], update(cc2,[ruum_420]),update(ccx,[ruum_420]), update(cc2TXT,[nelisada,kakskümmend]);		% olaf
	_G2192=[nelisada,kakskümmend,üks|_G2203], update(cc2,[ruum_421]),update(ccx,[ruum_421]), update(cc2TXT,[nelisada,kakskümmend,üks]);	% gert
	_G2192=[nelisada,kakskümmend,kaks|_G2203], update(cc2,[ruum_422]),update(ccx,[ruum_422]), update(cc2TXT,[nelisada,kakskümmend,kaks]);	% tarmo
	_G2192=[nelisada,kakskümmend,kolm|_G2203], update(cc2,[ruum_423]),update(ccx,[ruum_423]), update(cc2TXT,[nelisada,kakskümmend,kolm]);	% evelin
	_G2192=[nelisada,kakskümmend,neli|_G2203], update(cc2,[ruum_424]),update(ccx,[ruum_424]), update(cc2TXT,[nelisada,kakskümmend,neli]);	% sven
	_G2192=[nelisada,kakskümmend,viis|_G2203], update(cc2,[ruum_425]),update(ccx,[ruum_425]), update(cc2TXT,[nelisada,kakskümmend,viis]);	% jaagup
	_G2192=[nelisada,kakskümmend,kuus|_G2203], update(cc2,[ruum_426]),update(ccx,[ruum_426]), update(cc2TXT,[nelisada,kakskümmend,kuus]);	% tanel
	_G2192=[nelisada,kakskümmend,seitse|_G2203], update(cc2,[ruum_427]),update(ccx,[ruum_427]), update(cc2TXT,[nelisada,kakskümmend,seitse]);	% marko
	_G2192=[nelisada,kakskümmend,kaheksa|_G2203], update(cc2,[ruum_428]),update(ccx,[ruum_428]), update(cc2TXT,[nelisada,kakskümmend,kaheksa]);	% juhan
	_G2192=_G2203.

room([nelisada,üheksateist], ruum_419).		% jüri
room([nelisada,kakskümmend], ruum_420).		% olaf
room([nelisada,kakskümmend,üks], ruum_421).	% gert
room([nelisada,kakskümmend,kaks], ruum_422).	% tarmo
room([nelisada,kakskümmend,kolm], ruum_423).	% evelin
room([nelisada,kakskümmend,neli], ruum_424).	% sven
room([nelisada,kakskümmend,viis], ruum_425).	% jaagup
room([nelisada,kakskümmend,kuus], ruum_426).	% tanel
room([nelisada,kakskümmend,seitse], ruum_427).	% marko
room([nelisada,kakskümmend,kaheksa],ruum_428).	% juhan


roll([teejuhiks|_G2156],_G2156):-true.

viisimäärus1(_G2249,_G2260):-
	_G2249=[kohe|_G2260];
	_G2249=[ruttu|_G2260];
	_G2249=[viivitamata|_G2260];
	_G2249=[aeglasemalt|_G2260];
	_G2249=[kiiremini|_G2260];
	_G2249=[kiiresti|_G2260];
	_G2249=[aegalselt|_G2260].
ajamäärus(_G2168,_G2179):-
	_G2168=[varsti|_G2179];
	aeg(_G2168,_G2179).
kohamäärus1(_G2294,_G2305):-
	_G2294=[siia|_G2305];
	_G2294=[edasi|_G2305];
	_G2294=[otse|_G2305];
	_G2294=[sinna|_G2305];
	_G2294=[paremale|_G2305];
	_G2294=[vasakule|_G2305];
	_G2294=[tagasi|_G2305];
	_G2294=[kõrvale|_G2305];
	_G2294=[kaugemale|_G2305];
	_G2294=[lähemale|_G2305].
kohamäärus2(_G2204,_G2215):-_G2204=[juures|_G2215];_G2204=[kõrval|_G2215];_G2204=[lähedal|_G2215];_G2204=[ees|_G2215].
kohamäärus3(_G2204,_G2215):-_G2204=[juurde|_G2215];_G2204=[kõrvale|_G2215];_G2204=[lähedale|_G2215];_G2204=[ette|_G2215].
%-------------------AEG-------------------------
aeg(_G2172,_G2184):-
	(_G2172=[kell|_G2173];_G2172=_G2173),
	kella_näit(_G2173,_G2184),
	retract(tundTXT(H)),retract(minutTXT(M)),append(H,M,HM),
	retract(cc3TXT(_)),asserta(cc3TXT(HM)),
	retract(tund(HD)),retract(minut(MD)),
	retract(cc3(_)),asserta(cc3([HD,MD])).

kella_näit(_G2172,_G2183):-tund(_G2172,_G2203),minut(_G2203,_G2183).

test:- kella_näit([null,null,viiskümmend,seitse],_),tundTXT(H),minutTXT(M),append(H,M,HM),write(HM),tund(HD),minut(MD),write([HD,MD]).

tund([null,null|_G2504],_G2504):-	% 00:00-00:59
	asserta(tund(0)),asserta(tundTXT([null,null])),!.
tund([null,H2|_G2504],_G2504):-		% 01:00-09:59
	arv(H2,D), D > 0, D =<9,
	asserta(tund(D)),asserta(tundTXT([null,H2])),!.
tund([H1,H2|_G2504],_G2504):-		% 21:00-23:59
	arv(H1,D1), D1 = 20,
	arv(H2,D2), D2 >= 1, D2 =< 4,
	D is D1 + D2,
	asserta(tund(D)),asserta(tundTXT([H1,H2])),!.
tund([H|_G2504],_G2504):-		% 1:00-20:59
	arv(H,D), D >= 1, D =< 20,
	asserta(tund(D)),asserta(tundTXT([H])),!.

minut([null,null|_G2504],_G2504):-
	asserta(minut(0)),asserta(minutTXT([null,null])),!.
minut([null,M|_G2504],_G2504):-
	arv(M,D), D>=0, D=<9,
	asserta(minut(D)),asserta(minutTXT([null,M])),!.
minut([M1,M2|_G2504],_G2504):-
	arv(M1,D1), (D1=20;D1=30;D1=40;D1=50),
	arv(M2,D2),  D2>=1, D2=<9,
	D is D1 + D2,
	asserta(minut(D)),asserta(minutTXT([M1,M2])),!.
minut([M|_G2504],_G2504):-		% 1:00-20:59
	arv(M,D), (D>=1, D=<20; D=30; D=40; D=50),
	asserta(minut(D)),asserta(minutTXT([M])),!.

arvsõna1(_G2189,_G2200):-_G2189=[üheksamkümmend|_G2200];_G2189=[sada|_G2286],_G2286=[kaheksakümmend|_G2200].

%------Arvud------------
arv(üks, 1). arv(kaks, 2). arv(kolm, 3). arv(neli, 4). arv(viis, 5). arv(kuus, 6). arv(seitse, 7). arv(kaheksa, 8). arv(üheksa, 9). arv(kümme, 10).
arv(üksteist, 11). arv(kaksteist, 12). arv(komteist, 13). arv(neliteist, 14). arv(viisteist, 15). arv(kuusteist, 16). arv(seitseteist, 17). arv(kaheksateist, 18). arv(üheksateist, 19).
arv(kakskümmend, 20). arv(kolmkümmend, 30). arv(nelikümmend, 40). arv(viiskümmend, 50).
%------------------- Robot's reactions -----------------------------

reaction(juhata):-
	cc0(CC0),
	retract(world(CC0, Idx0, _, XYZQ0,_)),	% keda juhatada
	goto(CC0,XYZQ0), wait(done),		% mine juhatava juurde
	retract(world(me,Idx,_,_,_)),
	get_time(T0),
	asserta(world(me,Idx,Idx0,XYZQ0,T0)),
	cc2(CC2),cc2TXT(CC2P),
	append([CC0,palun, järgnege, mulle, juhatan, teid],CC2P,Text1),
	respond(Text1),
	käände_vorm(CC2Nim,_,CC2),
	world(CC2Nim, Idx2, _, XYZQ2,_),
	goto(CC2Nim,XYZQ2),  wait(done),		% käsk navisüsteemile
	respond(['oleme kohal']),
	retract(world(me,Idx,_, _,_)),
	get_time(T1),
	asserta(world(me,Idx,Idx2, XYZQ2,T1)),
	asserta(world(CC0, Idx0, Idx2, XYZQ2,T1)),!.
reaction(ütle_kus1):-
	ccx(CCx),
	world(CCx,Idx,EIdx,_,Time),
	world(Koht,EIdx,_,XYZQ,_),
	(Koht=kohvituba,KohtX=[kohvitoas];
	Koht=koridor,KohtX=[koridoris];
	room(KohtLong,Koht),KohtX=[toas|KohtLong];
	käände_vorm(Koht,2, KohtOm),KohtX=[KohtOm,läheduses]),
	append([CCx,asus,viimati],KohtX,Text2),
	respond(Text2),!.
reaction(ütle_kus2):-
	ccx(CCx),
	world(CCx,Idx,EIdx, XYZQ,_),
	world(Koht, EIdx, _, _, _),
	käänded(Koht,_,Vormid),
	nth1(8, Vormid, Kohal),			% leia alalütlev kääne
	respond([CCx, asub, Kohal]),!.
reaction(ütle_kus3):-
	cc0(CC0),
	world(CC0,Idx,EIdx,_,Time),
	world(Koht,EIdx,_,XYZQ,_),
	(Koht=kohvituba,KohtX=[kohvitoas];
	Koht=koridor,KohtX=[koridoris];
	room(KohtLong,Koht),KohtX=[toas|KohtLong];
	käände_vorm(Koht,2, KohtOm),KohtX=[KohtOm,juures]),
	append([CC0,asus,viimati],KohtX,Text),
	respond(Text),!.
reaction(ole):-
	ccx(CCx),
	käände_vorm(Koht,_,CCx),
	käände_vorm(Koht,2,KohtOm),
	world(Koht, Idx, EIdx, XYZQ, TS),
	(Koht=kohvituba,KohtX=[kohvitoas];
	Koht=koridor,KohtX=[koridoris];
	room(KohtLong,Koht),KohtX=[toas|KohtLong];
	käände_vorm(Koht,2, KohtOm),KohtX=[KohtOm,juures]),
	append([hästi, olen],KohtX,Text1),
	cc3TXT(HM),
	append(Text1,HM,Text),
	respond(Text),
	goto(Koht, XYZQ), wait(done), !.
reaction(tule1):-
	cc0(CC0), cc3(CC3),cc3TXT(CC3TXT),
	käände_vorm(CC0Nim,_,CC0),
	käände_vorm(CC0Nim,2,CC0Om),
	world(CC0Nim,Idx0,_,XYZQ0,_),
	retract(world(me,Idxme,_,_,_)),
	append([sain,aru,tulen],CC3TXT,Answer),
	respond(Answer),
	asserta(world(me,Idxme,Idx0,XYZQ0,CC3)),
	goto(CC0Nim,XYZQ0), wait(done),!.
reaction(tule2):-
	cc2(CC2), cc3(CC3),
	käände_vorm(CC2Nim,_,CC2),
	world(CC2Nim,Idx2,_,XYZQ2,_),
	retract(world(me,Idx0,PIdx,XYZQ0,_)),
	asserta(world(me,Idx0,Idx2,XYZQ2,CC3)),
	respond([sain,aru,tulen,kohe,CC2,juurde]),
	goto(CC2Nim,XYZQ2), !.
reaction(otsi_leia):-
	ccx(CCx), cc2(CC2),
	käände_vorm(CC2Nim,Nr,CC2),
	käände_vorm(CCxNim,_,CCx),
	käände_vorm(CCxNim,3,CCxOs),
	respond([hästi,lähen,CCxOs,CC2, otsima]),
	world(CC2Nim,Idx2,PIdx2,XYZQ2,TS2),
	goto(CC2Nim, XYZQ2), wait(done),
	world(CCx,Idx,PIdx,XYZQx,TS),
	goto(CCx,XYZQx), wait(done),
	respond([näe,leidsingi]),!.
reaction(anna_ulata):-
	cc0(CC0), cc1(CC1),
	käände_vorm(CC0Nim,Index,CC0),
	world(CC0Nim, Idx0, _, XYZQ0, _),
	world(CC1,Idx,_,_,_),
	respond([CC0Nim, palun, võta, see, CC1]),
	place_to(CC1, XYZQ0), wait(done),
	retract(world(CC1,Idx,PIdx,_,_)),
	get_time(Time),
	asserta(world(CC1,Idx,Idx0,XYZQ0,Time)),!.
reaction(võta_haara):-
	cc0(CC0),
	käände_vorm(CC0Nim,Index,CC0),
	world(CC0Nim, Idx0, _, XYZQ0, _),
	cc1(CC1),
	retract(world(CC1,Idx1,_,_,_)),
	pick(CC1, XYZQ0),
	world(me,Idxme,_,XYZQme,_),
	get_time(Time),
	asserta(world(CC1,Idx1,Idxme,XYZQme,Time)),
	respond([tänan]),!.
reaction(tõsta_pane):-
	cc1(CC1),
	cc2(CC2),
	käände_vorm(CC2Nim,Index,CC2),
	world(CC2Nim,Idx2,_,XYZQ2,_),
	respond([hästi, panen]),
	place_to(CC1, XYZQ2),
	wait(done),
	retract(world(CC1,Idx1,_,_,_)),
	get_time(TS),
	asserta(world(CC1,Idx1,Idx2,XYZQ2,TS)),!.
reaction(too):-
	cc0(CC0),			% Kellele tuua
	cc1(CC1),			% Mida tuua
	cc2(CC2),			% Kust tuua
	käände_vorm(CC2Nim,_,CC2),
	respond([hästi, lähen, tooma]),
	world(CC2Nim,_,_, XYZQ2,_),	% Leia koha koordinaadid kust tuua
	goto(CC2Nim,XYZQ2),	wait(done),
	world(CC1,_,_,XYZQ1,_),
	pick(CC1,XYZQ1), wait(done),
	käände_vorm(CC0Nim,_,CC0),
	world(CC0Nim,Idx0,_,XYZQ0,_),
	goto(CC0Nim,XYZQ0),
	respond([palun, võta,CC1]),
	place_to(CC1, XYZQ0),
	retract(world(CC1,Idx1,_,_,_)),
	get_time(TS),
	asserta(world(CC1,Idx1,Idx0,XYZQ0,TS)),!.
reaction(vii):-
	cc1(CC1), ccx(CCx),
	world(me,_,_,XYZQme,_),
	world(CC1,_,_,XYZQ1,_),
	käände_vorm(CCxNim,_,CCx),
	world(CCxNim,Idxx,_,XYZQx,_),
	käände_vorm(CC1,2,CC1Om),
	respond([hästi, viin, CC1Om,ära]),
	pick(CC1, XYZQ1),
	goto(CCxNim,XYZQx), wait(done),
	respond([palun,võta,CC1]),
	place_to(CC1, XYZQx),
	retract(world(CC1,Idx1,_,_,_)),
	get_time(TS),
	asserta(world(CC1,Idx1,Idxx,XYZQx,TS)),
	goto(speaker,XYZQme), wait(done), % R tuleb tagasi
	käände_vorm(CCxNim,7,CCx1),
	respond([CC1,on,viidud]),!.
reaction(liigu_mine):-
	ccx(CCx),
	käände_vorm(CCxNim,_,CCx),
	world(CCxNim,Idx,PIdx,XYZQ,_),
	respond([sain,aru,hakkan,minema]),
	goto(CCxNim,XYZQ),
	retract(world(me,Idxme,_,_,_)),
	get_time(TS), wait(done),
	respond([jõudsin,kohale]),
	asserta(world(me,Idxme,Idx,XYZQ,TS)),!.
reaction(A):-
	respond([tegevus,A,ebaõnnestus,ootan, uut, korraldust]),nl.
%---------------------------------- Liitsõnade moodustamine eraldi sõnadest--------------------------------------------
filtreeri_liitsõnad([],[]):- !.
filtreeri_liitsõnad(A,[S|B1]):-
	tee_liitsõna(A,[S|B]),
	filtreeri_liitsõnad(B,B1).

tee_liitsõna([kohvi,A|Tail],[AA|Tail]):- käände_vorm(ANim,Index,A),( ANim=masin; ANim=tass; ANim=tuba), atom_concat(kohvi,A,AA),!.
tee_liitsõna([tee,A|Tail], [AA|Tail]):- käände_vorm(ANim,Index,A),( ANim=pakk; ANim=tass; ANim=lusikas), atom_concat(tee,A,AA),!.
tee_liitsõna([paberi,A|Tail], [AA|Tail]):- käände_vorm(ANim,Index,A),( ANim=pakk; ANim=hunt; ANim=leht; ANim=korv), atom_concat(paberi,A,AA),!.
tee_liitsõna([aja,A|Tail], [AA|Tail]):- käände_vorm(ANim,Index,A),( ANim=kiri; ANim=leht), atom_concat(aja,A,AA),!.
tee_liitsõna([X,A|Tail], [AA|Tail]):- käände_vorm(ANim,Index,A), ( ANim=kümmend; ANim=teist; ANim=sada), atom_concat(X,A,AA),!.
tee_liitsõna([välja,trükk|Tail],[ väljatrükk|Tail]):- !.
tee_liitsõna(A,A).

% ------------------ Current Context variables -------------------
cc0(oliver).
cc1(tass).
cc2(ruum_411).
cc2TXT([ruum,nelisada,üksteist]).
cc3(now).
cc3TXT([esimesel,võimalusel]).
ccx(koridor).
%-----------------------------------------------------------------
käände_vorm(Nim,Index,Käändes):-
	käänded(Nim,_,Käänded),
	nth1(Index, Käänded, Käändes),!.
käände_vorm(Käändes,_,Käändes):-
%	write([sõnal, Käändes,puuduva,käänded]),
	nl.

update(cc3TXT,CC3TXT):-
	retract(cc3TXT(_)),
	asserta(cc3TXT(CC3TXT)),!.
update(cc2TXT,CC2P):-
	retract(cc2TXT(_)),
	asserta(cc2TXT(CC2P)),!.
update(Context,[Entity|_]):-		% Kontekstimuutuja värskendamine
	TermO=.. [Context,_],
	TermN=.. [Context, Entity],
	retract(TermO),
	asserta(TermN),!.
%--------------
update_W(Entity, AttrNr, Val):-		% world-fakti välja uuendamine
	retract(world(Entity,Idx,Pidx,XYZQ,TS)),
	substitute([Entity,Idx,Pidx,XYZQ,TS],AttrNr,Val,List1),
	Term=.. [world|List1],
	asserta(Term),!.
update_W(Entity, AttrNr, Val):-
	write('Incorrext world update'),nl,!.

substitute([El|Rest],1,El1,[El1|Rest]):- !.
substitute([El|Rest],N,El1,[El|Rest1]):-
	N1 is N-1,
	substitute(Rest,N1,El1,Rest1).

%------- Roboti käske imiteerivad predikaadid ----------------
wait(done).				% ootamine kuni robot täidab eelneva käsu
%--------------
goto( Place,XYZQ):- cc3(CC3),write('Going to: '), write([Place,with,coordinates,XYZQ,CC3]),nl,!.	% robot liigub koordinaadile XYZQ
pick( Object,XYZQ):- write('Picking: '), write([Object,from, XYZQ]),nl,!.		% objekti haaramine roboti poolt
place_to( Object, XYZQ):- write('Placing: '), write([Object,to, XYZQ]),nl,!.		% aseta objekt positsioonile XYZQ
respond(Text):- write('Telling: '), write(Text),nl,!.					% robot vastab lausega

%----------------  Entities of context cc0 --------------------------
%
world(me,	id0,id11, (23,34,2,0,0,0,1),  0).	% roboti asendit kirjeldav fakt
world(külaline, id1,id11, (1,14,0,0,0,0,1),   0).
world(oliver,	id2,id20, (41,14,2,0,0,0,1),  0).
world(jüri,	id3,id12, (41,14,2,0,0,0,1),  0).
world(gert,	id4,id13, (41,14,2,0,0,0,1),  0).
world(evelin,	id5,id14, (41,14,2,0,0,0,1),  0).
world(sven,	id6,id15, (41,14,2,0,0,0,1),  0).
world(jaagup,	id7,id16, (5,14,0,0,0,0,1),   0).
world(tanel,	id8,id17, (5,14,0,0,0,0,1),   0).
world(marko,	id9,id18, (41,14,2,0,0,0,1),  0).
world(juhan,	id10,id19, (41,14,2,0,0,0,1), 0).

% Entities of context cc1
world(tass,     id31,id28, (18,45,5,0,0,0,1), 0).
world(kauss,	id32,id27, (1,14,0,0,0,0,1),  0).
world(lusikas,	id33,id43, (1,14,0,0,0,0,1),  0).
world(nuga,     id34,id43, (1,14,0,0,0,0,1),  0).
world(pall,     id35,id27, (1,14,0,0,0,0,1),  0).
world(raamat,   id36,id29, (1,14,0,0,0,0,1),  0).
world(suss,   	id37,id12, (1,14,0,0,0,0,1),  0).
world(pastakas,	id38,id12, (1,14,0,0,0,0,1),  0).
world(väljatrükk,id39,id23, (1,14,0,0,0,0,1), 0).
world(paber,	id40, id23, (1,14,0,0,0,0,1), 0).
world(ajaleht,	id41, id29, (1,14,0,0,0,0,1), 0).
world(ajakiri,	id42, id29, (1,14,0,0,0,0,1), 0).

% Entities of context cc2
world(koridor,  id11,[],  (11,26,3,0,0,0,1),  0).
world(ruum_419, id12,[],  (3,4,0,0,0,0,1),    0).
world(ruum_420, id43,[],  (3,4,0,0,0,0,1),    0).
world(ruum_421, id13,[],  (43,14,0,0,0,0,1),  0).
world(ruum_423, id14,[],  (3,4,0,0,0,0,1),    0).
world(ruum_424, id15,[],  (43,14,0,0,0,0,1),  0).
world(ruum_425, id16,[],  (43,14,0,0,0,0,1),  0).
world(ruum_426, id17,[],  (43,14,0,0,0,0,1),  0).
world(ruum_427, id18,[],  (43,14,0,0,0,0,1),  0).
world(ruum_428, id19,[],  (43,14,0,0,0,0,1),  0).
world(kohvituba,id20,[],  (43,14,0,0,0,0,1),  0).

world(uks,	 id21, id11, (5,4,2,0,0,0,1), 0).
world(uks,	 id22, id11, (5,4,2,0,0,0,1), 0).
world(printer1,  id23, id11, (5,4,2,0,0,0,1), 0).
world(printer2,  id24, id11, (5,4,2,0,0,0,1), 0).
world(diivan1,   id25, id11, (1,14,0,0,0,0,1),0).
world(diivan2,   id26, id11, (1,14,0,0,0,0,1),0).

world(tool,	 id27, id20, (1,14,0,0,0,0,1), 0).
world(kohvimasin,id28, id20, (1,14,0,0,0,0,1), 0).
world(laud,	 id29, id20, (1,14,0,0,0,0,1), 0).
world(paberihunt,id30, id11, (1,14,0,0,0,0,1), 0).
world(uks,	 id31, id20, (1,14,0,0,0,0,1), 0).
world(kapp,	 id43, id20, (1,14,0,0,0,0,1), 0).

%------------------------Käändsõnade grammatika -----------------------------
käänded(inimene, [ns, _, elus], [inimene, inimese, inimest, inimesse, inimeses, inimesest, inimesele, inimesel, inimeselt, inimeseks, inimeseni, inimesena, inimeseta, inimesega, inimesed, inimeste, inimesi, inimestesse, inimestes, inimestest, inimestele, inimestel, inimestelt, inimesteks, inimesteni, inimestena, inimesteta, inimestega]).
käänded(mees,  [ns, _, elus],  [mees, mehe, meest, mehesse, mehes, mehest, mehele, mehel, mehelt, meheks,  meheni,  mehena,  meheta, mehega,  mehed, meeste, mehi, meestesse, meestes, meestest, meestele, meestel, meestelt, meesteks, meesteni, meestena, meesteta, meestega]).
käänded(naine, [ns, ains, elus], [naine, naise, naist, naisesse, naises, naisest, naisele, naisel, naiselt, naiseks, naiseni, naisena, naiseta, naisega, naised, naiste, naisi, naistesse, naistes, naistest, naistele, naistel, naistelt, naisteks, naisteni, naistena, naisteta, naistega]).
käänded(laps, [ns, ains, elus], [laps, lapse, last, lapsesse, lapses, lapsest, lapsele, lapsel, lapselt, lapseks, lapseni, lapsena, lapseta, lapsega, lapsed, laste, lapsi, lastesse, lastes, lastest, lastele, lastel, lastelt, lasteks, lasteni, lastena, lasteta, lastega]).
käänded(poiss, [ns, ains, elus], [poiss, poisi, poissi, poissi, poisis, poisist, poisile, poisil, poisilt, poisiks, poisini, poisina, poisita, poisiga, poisid, poiste, poisse, poistesse, poistess, poistesst, poistele, poistel, poistelt, poisteks, poisteni, poistena, poisteta, poistega]).
käänded(tüdruk, [ns, ains, elus], [tüdruk, tüdruku, tüdrukut, tüdrukusse, tüdrukus, tüdrukust, tüdrukule, tüdrukul, tüdrukult, tüdrukuks, tüdrukuni, tüdrukuna, tüdrukuta, tüdrukuga, tüdrukud, tüdrukute, tüdrukuid, tüdrukutesse, tüdrukutes, tüdrukutest, tüdrukutele, tüdrukutel, tüdrukutelt, tüdrukuteks, tüdrukuteni, tüdrukutena, tüdrukuteta, tüdrukutega]).
käänded(külaline, [ns, ains, elus], [külaline, külalise, külalist, külalisse, külalises, külalisest, külalisele, külalisel, külaliselt,külaliseks, külaliseni,külalisena, külaliseta, külalisega, külalised, külaliste, külalisi, külalistesse, külalistes, evelinidest,külalistele, külalistel, külalistelt, külalisteks, külalisteni, külalistena, külalisteta, külalistega]).

% ASESÕNAD -----------------
käänded(mina, [ns, ains, elus], [mina, minu, mind, minusse, minus, minust, minule, minul, minult, minuks, minuni, minuna,  minuta, minuga, minad, minade, minasid,  minadesse, minades, minadest, minadele, minadel, minadelt, minadeks, minadeni, minadena, minadeta, minadega]).
käänded(ma, [ns, ains, elus], [ma, mu, mind, musse, mus, must, mule, mul, mult, muks, muni, muna,  muta, muga, mud, mude, musid,  mudesse, mudes, mudest, mudele, mudel, mudelt, mudeks, mudeni, mudena, mudeta, mudega]).
käänded(sina, [ns, ains, elus], [sina, sinu, sind, sinusse, sinus, sinust, sinule, sinul, sinult, sinuks, sinuni, sinuna,  sinuta, sinuga, sinad, sinade, sinasid,  sinadesse, sinades, sinadest, sinadele, sinadel, sinadelt, sinadeks, sinadeni, sinadena, sinadeta, sinadega]).
käänded(sa, [ns, ains, elus], [sa, su, sind, susse, sus, sust, sule, sul, sult, suks, suni, suna,  suta, suga, sud, sude, susid,  sudesse, sudes, sudest, sudele, sudel, sudelt, sudeks, sudeni, sudena, sudeta, sudega]).
käänded(tema, [ns, ains, elus], [tema, tema, teda, temasse, temas, temast, temale, temal, temalt, temaks, temani, temana,  temata, temaga, temad, temade, sinasid,  temadesse, temades, temadest, temadele, temadel, temadelt, temadeks, temadeni, temadena, temadeta, temadega]).
käänded(ta, [ns, ains, elus], [ta, tema, teda, tasse, tas, tast, talle, tal, talt, taks, tani, tana,  tata, taga, tad, tade, tasid,  tadesse, tades, tadest, tadele, tadel, tadelt, tadeks, tadeni, tadena, tadeta, tadega]).
käänded(meie, [ns, mitm, elus], [meie, meie, meid, meiesse, meis, meist, meile, meil, meilt, meieks, meieni, meiena, meieta, meiega, meied, meiede, meiesid, meiedesse, meiedes, meiedest, meiedele, meiedel, meiedelt, meiedeks, meiedeni, meiedena, meiedeta, meiedega]).
käänded(me, [ns, mitm, elus], [me, me, meid, meisse, meis, mest, meile, meil, meilt, meiks, meni, mena, meta, mega, med, meiede, mesid, medesse, medes, medest, medele, medel, medelt, medeks, medeni, medena, medeta, medega]).
käänded(teie, [ns, mitm, elus], [teie, teie, teid, teiesse, teies, teiest, teile, teil, teilt, teieks, teieni, teiena, teieta, teiega, teied, teiede, teiesid, teiedesse, teiedes, teiedest, teiedele, teiedel, teiedelt, teiedeks, teiedeni, teiedena, teiedeta, teiedega]).
käänded(te, [ns, mitm, elus], [te, teie, teid, teisse, teis, teist, teile, teil, teilt, teiks, teini, teina, teita, teiga, teid, teite, teisid, teidesse, teites, teitest, teitele, teitel, teitelt, teiteks, teiteni, teiedena, teiteta, teitega]).
käänded(nemad, [ns, mitm, elus], [nemad, nende, neid, nendesse, nendes, nendest, nendele, nendel, nendelt, nendeks, nendeni, nendena, nendeta, nendega, nemad, nemade, nemasid, nemadesse, nemades, nemadest, nemadele, nemadel, nemadelt, nemadeks, nemadeni, nemadena, nemadeta, nemadega]).
käänded(nad, [ns, mitm, elus], [nad, nende, neid, nendesse, nendes, nendest, nendele, nendel, nendelt, nendeks, nendeni, nendena, nendeta, nendega, nemad, nemade, nemasid, nemadesse, nemades, nemadest, nemadele, nemadel, nemadelt, nemadeks, nemadeni, nemadena, nemadeta, nemadega]).

% NIMED ------------------
käänded(evelin, [ns, ains, elus], [evelin, evelini, evelini, evelinisse, evelinis, evelinist, evelinile, evelinil, evelinilt, eveliniks, evelinini, evelinina, evelinita, eveliniga,evelinid, evelinide, eveline, evelinidesse, evelinides, evelinidest, evelinidele, evelinidel, evelinidelt, evelinideks, evelinideni, evelinidena, evelinideta, evelinidega]).
käänded(oliver, [ns, ains, elus], [oliver, oliveri, oliveri, oliverisse, oliveris, oliverist, oliverile, oliveril, oliverilt, oliveriks, oliverini, oliverina, oliverita, oliveriga,oliverid, oliveride, olivere,oliveridesse, oliverides, oliveridest, oliveridele, oliveridel, oliveridelt,oliverideks, oliverideni, oliveridena, oliverideta, oliveridega]).
käänded(jüri, [ns, ains, elus], [jüri, jüri, jüri, jürisse, jüris, jürist, jürile, jüril, jürilt, jüriks, jürini, jürina, jürita, jüriga,jürid, jüride, jürisid, jüridesse, jürides, jüridest, jüridele, jüridel, jüridelt,jürideks, jürideni, jüridena, jürideta, jüridega]).
käänded(gert, [ns, ains, elus], [gert, gerdi, gerti, gerdisse, gerdis, gerdist, gerdile, gerdil, gerdilt, gerdiks, gerdini, gerdina, gerdita, gerdiga,gerdid, gertide, gertisid, gertidesse, gertides, gertidest, gertidele, gertidel, gertidelt,gertideks, gertideni, gertidena, gertideta, gertidega]).
käänded(sven, [ns, ains, elus], [sven, sveni, svenni, svenisse, svenis, svenist, svenile, svenil, svenilt, sveniks, svenini, svenina, svenita, sveniga,svenid, svennide, svenisid, svenidesse, svenides, svenidest, svenidele, svenidel, svenidelt,svenideks, svenideni, svenidena, svenideta, svenidega]).
käänded(jaagup, [ns, ains, elus], [jaagup, jaagupi, jaagupit, jaagupisse,jaagupis, jaagupist, jaagupile, jaagupil, jaagupilt, jaagupiks, jaagupini, jaagupina, jaagupita, jaagupiga,jaagupid, jaagupite, jaagupeid, jaagupitesse, jaagupites, jaagupitest, jaagupitele, jaagupitel, jaagupitelt,jaagupiteks, jaagupiteni, jaagupitena, jaagupiteta, jaagupitega]).
käänded(tanel, [ns, ains, elus], [tanel, taneli, tanelit, tanelisse,tanelis, tanelist, tanelile, tanelil, tanelilt, taneliks, tanelini, tanelina, tanelita, taneliga,tanelid, tanelite, taneleid, tanelitesse, tanelites, tanelitest, tanelitele, tanelitel, tanelitelt,taneliteks, taneliteni, tanelitena, taneliteta, tanelitega]).
käänded(marko, [ns, ains, elus], [marko, marko, markot, markosse,markos, markost, markole, markol, markolt, markoks, markoni, markona, markota, markoga,markod, markode, markosid, markodesse, markodes, markodest, markodele, markodel, markodelt,markodeks,markodeni, markodena, markodeta, markodega]).
käänded(juhan, [ns, ains, elus], [juhan, juhani, juhanit, juhanisse,juhanis, juhanist, juhanile, juhanil, juhanilt,juhaniks, juhanini, juhanina, juhanita, juhaniga,juhanid, juhanite, juhaneid, juhanitesse, juhanites, juhanitest, juhanitele, juhanitel, juhanitelt,juhaniteks,juhaniteni, juhanitena, juhaniteta,juhanitega]).

% ESEMED -----------------
käänded(asi, [ns, ains, eluta], [asi, asja, asja, asjasse, asjas, asjast, asjale, asjal, asjalt, asjaks, asjani, asjana, asjata, asjaga, asjad, asjade, asju, asjadesse, asjades, asjadest, asjadele,	asjadel, asjadelt, asjadeks, asjadeni, asjadena, asjadeta, asjadega]).
käänded(tass,  [ns, ains, eluta], [tass, tassi, tassi,  tassi,  tassis,  tassist,  tassile,  tassil,  tassilt,  tassiks,  tassini,  tassina,  tassita, tassiga, tassid,  tasside,  tassisid,  tassidesse,  tassides,  tassidest,  tassidele,  tassidel,  tassidelt,  tassideks,  tassideni,  tassidena,  tassideta, tassidega]).
käänded(lusikas,[ns, ains, eluta], [lusikas, lusika, lusikat,  lusikasse, lusikas, lusikast, lusikale, lusikal,lusikalt, lusikaks,lusikani, lusikana, lusikata, lusikaga, lusikad, lusikate,  lusikaid, lusikatesse, lusikates, lusikatest,lusikatele, lusikatel, lusikatelt, lusikateks, lusikateni, lusikatena, lusikateta, lusikatega]).
käänded(nuga, [ns, ains, eluta], [nuga, noa, nuga, noasse, noas, noast, noale, noal, noalt, noaks, noani, noana, noata, noaga, noad, nugade, nuge, nugadesse, nugadedes, nugadedest, nugadedele, nugadel, nugadelt, nugadeks, nugadeni, nugadena, nugadeta, nugadega]).
käänded(kahvel, [ns, ains, eluta], [kahvel, kahvli, kahvlit, kahvlisse, kahvlis, kahvlist, kahvlile, kahvlil, kahvlilt, kahvliks, kahvlini, kahvlina, kahvlita, kahvliga, kahvlid, kahvlite,kahvleid, kahvlitesse, kahvlites, kahvlitest, kahvlitele, kahvlitel, kahvlitelt, kahvliteks,kahvliteni,kahvlitena,kahvliteta, kahvlitega]).
käänded(raamat,  [ns, ains, eluta], [raamat, raamatu, raamatut,  raamatusse, raamatus, raamatust, raamatule, raamatul,raamatult, raamatuks, raamatuni, raamatuna, raamatuta, raamatuga, raamatud, raamatute,  raamatuid, raamatutesse, raamatutes, raamatutest, raamatutele, raamatutel, raamatutelt, raamatuteks, raamatuteni, raamatutena, raamatuteta, raamatutega]).
käänded(pall,  [ns, ains, eluta], [pall, palli, palli,  palli,  pallis,  pallist,  pallile,  pallil,  pallilt,  palliks,  pallini,  pallina,  pallita, palliga, pallid,  pallide,  pallisid,  pallidesse,  pallides,  pallidest,  pallidele,  pallidel,  pallidelt,  pallideks,  pallideni,  pallidena,  pallideta, pallidega]).


%RUUMID-------------------
käänded(ruum, [ns, ains, eluta], [ruum, ruumi, ruumi, ruumi, ruumis, ruumist, ruumile, ruumil, ruumilt, ruumiks, ruumini, ruumina, ruumita, ruumiga, ruumid, ruumide, ruume, ruumidesse,ruumides, ruumidest, ruumidele, ruumidel, ruumidelt, ruumideks, ruumideni, ruumidena, ruumideta, ruumidega]).
käänded(tuba, [ns, ains, eluta], [tuba, toa, tuba, tuppa, toas, toast, toale, toal, toalt, toaks, toani, toana, toata, toaga, toad, tubade, tubasid, tubadesse, tubades, tubadest, tubadele,tubadel, tubadelt, tubadeks, tubadeni, tubadena, tubadeta, tubadega]).
käänded(koridor, [ns, ains, eluta], [koridor, koridori, koridori, koridori, koridoris, koridorist, koridorile, koridoril, koridorilt, koridoriks, koridorini, koridorina, koridorita,	koridoriga, koridorid, koridoride, koridore, koridoridesse, koridorides, koridoridest, koridoridele, koridoridel, koridoridelt, koridorideks, koridorideni,	koridoridena, koridorideta, koridoridega]).
käänded(kabinet, [ns, ains, eluta], [kabinet, kabineti, kabinetti, kabinetti, kabinetis, kabinetist, kabinetile, kabinetil, kabinetilt, kabinetiks, kabinetini, kabinetina, kabinetita, kabinetiga,kabinetid, kabinettide, kabinette, kabinettidesse, kabinettides, kabinettidest, kabinettidele, kabinettidel, kabinettidelt, kabinettideks, kabinettideni, kabinettidena, kabinettideta,	kabinettidega]).
käänded(kohvituba, [ns, ains, eluta], [kohvituba, kohvitoa, kohvituba, kohvituppa, kohvitoas, kohvitoast, kohvitoale, kohvitoal, kohvitoalt, kohvitoaks, kohvitoani, kohvitoana, kohvitoata, kohvitoaga,kohvitoad, kohvitubade, kohvitube, kohvitubadesse, kohvitubades, kohvitubadest, kohvitubadele, kohvitubadel, kohvitubadelt, kohvitubadeks, kohvitubadeni, kohvitubadena, kohvitubadeta,	kohvitubadega]).

%RUUMI SISUSTUS
käänded(laud, [ns, ains, eluta], [laud, laua, lauda, lauasse, lauas, lauast, lauale, laual, laualt, lauaks, lauani, lauana, lauata, lauaga, lauad, laudade, laudu, laudadesse,	laudades, laudadest, laudadele, laudadel, laudadelt, laudadeks, laudadeni, laudadena, laudadeta, laudadega]).
käänded(tool, [ns, ains, eluta], [tool, tooli, tooli, tooli, toolis, toolist, toolile, toolil, toolilt, tooliks, toolini, toolina, toolita, tooliga, toolid, toolide, toole, toolidesse,toolides, toolidest, toolidele,toolidel,toolidelt, toolideks,toolideni, toolidena, toolideta, toolidega]).
käänded(aken, [ns, ains, eluta], [aken, akna, akent, aknasse, aknas, aknast, aknale, aknal, aknalt, aknaks, aknani, aknana, aknata, aknaga, aknad, aknate, aknaid, akendesse, akendes, aknendest, aknendele, aknendel, aknendelt, aknendeks, aknendeni, aknendena, aknendeta, aknendega]).
käänded(uks,  [ns, ains, eluta], [uks, ukse, ust, uksesse, ukses, uksest, uksele, uksel, ukselt, ukseks, ukseni, uksena, ukseta, uksega, uksed, uste, uksi, ustesse, ustes, ustest, ustele, ustel, ustelt, usteks, usteni, ustena, usteta, ustega]).
käänded(riiul,[ns, ains, eluta], [riiul, riiuli, riiulit, riiulisse, riiulis, riiulist, riiulile, riiulil, riiulilt, riiuliks, riiulini, riiulina, riiulita, riiuliga, riiulid, riiulite, riiuleid, riiulitesse, riiulites, riiulitest, riiulitele, riiulitel,riiulitelt, riiuliteks, riiuliteni, riiulitena, riiuliteta, riiulitega]).
käänded(kapp, [ns, ains, eluta], [kapp, kapi, kappi, kappi, kapis, kapist, kapile, kapil, kapilt, kapiks, kapini, kapina,kapita,kapiga, kapid,kappide, kappe, kappidesse, kappides, kappidest, kappidele, kappidel,kappidelt, kappideks, kappideni, kappidena, kappideta, kappidega]).
käänded(kohvimasin, [ns, ains, eluta], [kohvimasin,kohvimasina,kohvimasinat,kohvimasinasse, kohvimasinas, kohvimasinast, kohvimasinale,kohvimasinal,kohvimasinalt, kohvimasinaks, kohvimasinani,kohvimasinana,kohvimasinata,kohvimasinaga, kohvimasinad,kohvimasinate, kohvimasinaid, kohvimasinatesse,kohvimasinates, kohvimasinatest, kohvimasinatele, kohvimasinatel, kohvimasinatelt, kohvimasinateks, kohvimasinateni, kohvimasinatena, kohvimasinateta, kohvimasinatega]).
käänded(masin, [ns, ains, eluta], [masin,masina,masinat,masinasse, masinas, masinast,masinale,masinal,masinalt, masinaks,masinani,masinana,masinata,masinaga, masinad,masinate, masinaid, masinatesse,masinates, masinatest, masinatele, masinatel, masinatelt, masinateks, masinateni, masinatena,masinateta, masinatega]).
käänded(põrand, [ns, ains, eluta], [põrand,põranda,põrandat,põrandasse, põrandas, põrandast,põrandale,põrandal,põrandalt, põrandaks,põrandani,põrandana,põrandata,põrandaga, põrandad,põrandate, põrandaid, põrandatesse,põrandates, põrandatest, põrandatele, põrandatel, põrandatelt, põrandateks, põrandateni, põrandatena,põrandateta, põrandatega]).


% Arvud
käänded(teistkümmend,  [arvsonajärk,  ains, _], [teistkümmend, teistkümne, teistkümmet, teistkümnesse, teistkümnes, teistkümnest, teistkümnele,  teistkümnel,  teistkümnelt,  teistkümneks, teistkümneni, teistkümnena, teistkümneta, teistkümnega, teistkümned, teistkümnete, teistkümneid, teistkümnetesse, teistkümnetes, teistkümnetes, teistkümnetele,  teistkümnetel,  teistkümnetelt, teistkümneteks,  teistkümneteni,  teistkümnetena,  teistkümneteta,  teistkümnetega]).
käänded(teist,  [arvsonajärk, ains, _], [teist, teistme, teistmet, teistmesse, teistmes, teistmest, teistmele, teistmel, teistmelt, teistmeks, teistmeni, teistmena, teistmeta, teistmega,
teistmed, teistmete, teistmeid, teistmetesse, teistmetes, teistmetest, teistmetele, teistmetel, teistmetelt, teistmeteks, teistmeteni, teistmetena, teistmeteta,teistmetega]).
käänded(kümmend, [arvsonajärk, ains, _], [kümmend, kümne, kümmet, kümnesse, kümnes, kümnest, kümnele, kümnel, kümnelt,  kümneks, kümneni, kümnena, kümneta, kümnega]).
käänded(sada,  [arvsonajärk, ains, _], [sada, saja, sadat, sajasse, sajas, sajast, sajale, sajal, sajalt, sajaks, sajani, sajana, sajata, sajaga, sajad, sadade, sadu, sadadesse, sadades, sadadest, sadadele, sadadel, sadadelt, sadadeks, sadadeni, sadadena, sadadeta, sadadega]).
käänded(tuhat,  [arvsonajärk1, ains, _], [tuhat, tuhande, tuhandet, tuhandesse, tuhandes, tuhandest, tuhandele, tuhandel, tuhandelt, tuhandeks, tuhandeni, tuhandena, tuhandeta, tuhandega, tuhanded, tuhandete, tuhandeid, tuhandetesse, tuhandetes, tuhandetest,  tuhandetele, tuhandetel, tuhandetelt,  tuhandeteks,  tuhandeteni, tuhandetena, tuhandeteta, tuhandetega]).
käänded(miljon,  [arvsonajärk1, ains, _], [miljon, miljoni, miljonit, miljonisse, miljonis, miljonist, miljonile, miljonil, miljonilt, miljoniks, miljonini, miljonina, miljonita, miljoniga, miljonid, miljonite, miljoneid, miljonitesse, miljonites, miljonitest, miljonitele, miljonitel, miljonitelt, miljoniteks, miljoniteni, miljonitena, miljoniteta, miljonitega]).
käänded(paar,  [arvsonajärk1, ains, _], [paar, paari, paari, paarisse, paaris, paarist, paarile, paaril, paarilt, paariks, paarini, paarina, paarita, paariga, paarid, paaride, paarisid, paaridesse, paarides, paaridest, paarildele, paaridel, paaridelt, paarideks, paarideni, paaridena, paarideta, paaridega]).
käänded(tosin,  [arvsonajärk1, ains, _], [tosin, tosina, tosinat, tosinasse, tosinas, tosinast, tosinale, tosinal, tosinalt, tosinaks, tosinani, tosinana, tosinata, tosinaga, tosinad, tosinate, tosinaid, tosinatesse, tosinates, tosinatest, tosinatele, tosinatel, tosinatelt, tosinateks, tosinateni, tosinatena, tosinateta, tosinatega]).

