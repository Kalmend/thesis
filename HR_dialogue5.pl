:- dynamic best_match/2.	% Dünaamiline fakt sisendsõna teadaoleva parima vaste kohta
:- dynamic cc0/1,cc1/1,cc2/1,cc2TXT/1,cc3/1,cc3TXT/1,ccx/1.
:- dynamic world/5.

% Abifakt sisendi parandamisel:
similarity_treshold(0.7).	% Lexical similarity >= treshold
%=================================================================
:- dynamic input_filename/1.
:- dynamic output_filename/1.
input_filename('/home/oliver/thesis/catkin_ws/input.txt').
output_filename('/home/oliver/thesis/catkin_ws/output.txt').

do_change_input(Name) :-
    retractall(input_filename(_)),
    assert(input_filename(Name)).
    
do_change_output(Name) :-
    retractall(output_filename(_)),
    assert(output_filename(Name)).
    
change_input_filename(Name) :- 
    not(is_task_alive()) -> do_change_input(Name) ; false.

change_output_filename(Name) :- 
    not(is_task_alive()) -> do_change_output(Name) ; false.
    
%=========================== TEST ================================
t:-
	input_filename(From),
	output_filename(To),
	dialogue_ctrl(From, To).
t.
%========================== END TESTS ============================

%======================== SYNCHRONIZATION =====================================
%--- incoming sync commands
:- message_queue_create(wait_for_signal).
:- mutex_create(sync_mutex).

%--- sync API
is_task_in_progress() :- with_mutex(sync_mutex, do_is_task_in_progress()).
is_task_alive() :- with_mutex(sync_mutex, do_is_task_alive()).
cleanup_if_done() :- with_mutex(sync_mutex, do_cleanup_if_done()).
kill_current_task() :- with_mutex(sync_mutex, do_kill_current_task()).
signal_done() :- with_mutex(sync_mutex, do_signal_done()).
are_files_free() :- with_mutex(sync_mutex, not(is_stream(input_file);is_stream(output_file))).

%--- sync implementation
do_is_task_in_progress() :- catch(thread_property(task_thread, status(running)), _, fail).
do_is_task_alive() :- catch(thread_property(task_thread, _), _, fail).
do_cleanup_if_done() :- (not(is_task_in_progress()),is_task_alive()) -> thread_join(task_thread,_); false.

do_kill_current_task() :-
    is_task_in_progress() -> (thread_signal(task_thread, abort), do_cleanup_if_done()); 
    do_cleanup_if_done().
do_signal_done() :- is_task_in_progress() -> thread_send_message(wait_for_signal, gotit) ; false.

%--- outgoing sync commands
wait(_):- mutex_unlock(sync_mutex), thread_get_message(wait_for_signal, gotit),mutex_lock(sync_mutex),!.

%======================== MAIN CALLABLE =====================================
% To - variable showing output action file for commands to robot
% From - variable showing input text file from speech recognition
%----------------------------------------------------------------------------

dialogue_ctrl(From, To) :- thread_create(with_mutex(sync_mutex,dialogue_ctrl_task(From, To)),_,[alias(task_thread)]).

dialogue_ctrl_task(From, To):-
	get_command(From, Text_in),							% read input command from file
	write('KÄSK KÕNETUVASTUSEST: '), write(Text_in),nl,
	filter_compound_words(Text_in,CorrectedCommand), % moodusta sisendfraasis vajalikud liitsõnad
	interpret_command(To,From,CorrectedCommand,_),!.	% if CorrectedCommand is recognized by command grammar, then continue with asked action


% ==================== READING SPEECH RECOGNITION OUTPUT ====================
% reads a phrase from speech recognition output file (file name in variable From)
% and converts it to the list of words (the value of variable Text_in)
%-------------------------------------------------------------------------------
get_command(From,Text_in):-
		open( From, read, H1, [alias(input_file)]),
		read_line_to_codes(H1, Codes),
		convert_C(Codes, Text),
		convert_D([' '|Text], _,Text_in),
		close(H1),!.

%--- Transforming the list of codes to the list of symbols -----------
convert_C([],[]):- !.
convert_C([Code|Tail],[Atom| Atoms]):-
	convert_C(Tail, Atoms),
	char_code(Atom, Code), !.

convert_D([], [], []):- !.
convert_D([' '|Text], [], [Atom| Text_in]):-
	convert_D(Text, Text_sym, Text_in),
	atomic_list_concat(Text_sym, Atom).
convert_D([Sym| Text], [Sym| Text_sym], Text_in):-
	convert_D(Text, Text_sym, Text_in).
% ================= END READING SPEECH RECOGNITION OUTPUT ====================


%-======================== SIMILARITY COEFFICIENT CALCULATION ================================
% Sõnade leksikalise sarnasuse koefitsendi arvutamine tähtede listidel
% Sisend: võrreldavate sõnade tähtede listid
% Väljund: Sarnasus koefitsient 0-1
%---------------------------------------------------------------------------------------------

%--------------
% Algorithm 2: % võtab arvesse tähtede järjestust sõnas ja nihet üksteise suhtes
%--------------
calculate_similarity1(Word,Word, 1):- !.
calculate_similarity1(Word1,Word2, Similarity):-
	count_similars(Word1,Word2,SimilarCount),
	length(Word1,Length1),
	length(Word2,Length2),
	Similarity is 2*SimilarCount/(Length1+Length2), !.
%-------------
count_similars(Word,Word, Length):- length(Word,Length),!.
count_similars([],_, 0):- !.
count_similars(_,[], 0):- !.
count_similars([X|Word1],[X|Word2],SimilarCount):-
    count_similars(Word1,Word2,SimilarCount1),
    SimilarCount is SimilarCount1 + 1,!.
count_similars([_|Word],[_|Word], Length):- length(Word,Length),!.
count_similars([_,_|Word],[_,_|Word], Length):- length(Word,Length),!.
count_similars([_,_,_|Word],[_,_,_|Word], Length):- length(Word,Length),!.
count_similars([_,X|Word1],[X|Word2],SimilarCount):-
        count_similars([X|Word1],[X|Word2],SimilarCount),!.
count_similars([X|Word1],[_,X|Word2],SimilarCount):-
        count_similars([X|Word1],[X|Word2],SimilarCount),!.
count_similars([_,_,X|Word1],[X|Word2],SimilarCount):-
        count_similars([X|Word1],[X|Word2],SimilarCount),!.
count_similars([X|Word1],[_,_,X|Word2],SimilarCount):-
        count_similars([X|Word1],[X|Word2],SimilarCount),!.
count_similars([_|Word1],[_|Word2],SimilarCount):-
    count_similars(Word1,Word2,SimilarCount),!.
%========================== END SIMILARITY COEFFICIENT CALCULATION ====================


%============================ Liitsõnade moodustamine =================================
% SISEND: list, kus on lihtsõnad
% VÄLJUND: list, kus lihtsõnad on liidetud vastavalt sõnastiku liitsõnadele
%--------------------------------------------------------------------------------------
filter_compound_words([],[]):- !.
filter_compound_words(A,[S|B1]):-
	make_compound_word(A,[S|B]),
	filter_compound_words(B,B1).
%--------------------------------------------------------------------------------------
% JUHIS: uue liitsõna lisamisel sõnastikku tuleb see defineerida make_compound_word/2 ühe alternatiivina
%--------------------------------------------------------------------------------------
make_compound_word([kohvi,A|Tail],[AA|Tail]):- find_case(ANim,_,A),(ANim=masin; ANim=tass; ANim=tuba), atom_concat(kohvi,A,AA),!.
make_compound_word([tee,A|Tail], [AA|Tail]):- find_case(ANim,_,A),(ANim=pakk; ANim=tass; ANim=lusikas), atom_concat(tee,A,AA),!.
make_compound_word([paberi,A|Tail], [AA|Tail]):- find_case(ANim,_,A),(ANim=pakk; ANim=hunt; ANim=leht; ANim=korv), atom_concat(paberi,A,AA),!.
make_compound_word([aja,A|Tail], [AA|Tail]):- find_case(ANim,_,A),(ANim=kiri; ANim=leht), atom_concat(aja,A,AA),!.
make_compound_word([X,A|Tail], [AA|Tail]):- find_case(ANim,_,A), (ANim=kümmend; ANim=teist; ANim=sada), atom_concat(X,A,AA),!.
make_compound_word([välja,trükk|Tail],[ väljatrükk|Tail]):- !.
make_compound_word(A,A).
%------------------------------
find_case(Nim,Index,Käändes):-
	cases(Nim,_,Käänded),
	nth1(Index, Käänded, Käändes),!.
find_case(Käändes,_,Käändes):-
%	write([sõnal, Käändes,puuduva,käänded]),
	nl.
%================================ END Liitsõnade moodustamine ===========================



% ======================== ROBOTILE ANTAVATE KÄSKUDE INTERPRETAATOR ========================
% Need alternatiivid implementeerivad DCG reegleid
% Otsesele interpretatsioonile on lisatud täiendusi
%-------------------------------------------------------------------------------------------
interpret_command(ToR, FromR,Phrase,Rest):-						% juhata Kes Kuhu
	in_context(Phrase,juhata,_,Tail0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(in_context(Tail0,isik_1,Isik,Tail1),update(cc0,[Isik]); Tail0 =Tail1),
	update(cc2,Tail1),
	(koht1_4(Tail1,Rest); Rest=Tail1),
	reaction(ToR, FromR,juhata),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% ütle kus on Asi
	in_context(Phrase,ütle,_,Phrase1), in_context(Phrase1,kus,_,Tail0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	in_context(Tail0,verb2,_,Tail1),
	(in_context(Tail1,objekt1, Word1,Rest), update(cc1,[Word1]),update(ccx,[Word1]);
	 in_context(Tail1,objekt21,Word1,Rest), update(cc2,[Word1]),update(ccx,[Word1])),
	reaction(ToR, FromR,ütle_kus1),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% ütle kus on Isik
	in_context(Phrase,ütle,_,Phrase1), in_context(Phrase1,kus,_,Tail0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(in_context(Tail0,verb2,_,Tail1), in_context(Tail1,isik_1,Isik,Rest);
	in_context(Tail0,isik_1,Isik,Tail1),in_context(Tail1,verb2,_,Rest)),
	update(cc0, [Isik]),
	reaction(ToR, FromR,ütle_kus3),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% ütle kus Asi on
	in_context(Phrase,ütle,_,Phrase1), in_context(Phrase1,kus,_,Tail0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(in_context(Tail0,objekt1, Word,Tail1), update(cc1,[Word]),update(ccx,[Word]);
	 in_context(Tail0,objekt21,Word,Tail1), update(cc2,[Word]),update(ccx,[Word])),
	in_context(Tail1,verb2,_,Rest),
	reaction(ToR, FromR,ütle_kus1),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% ole
	in_context(Phrase,ole,_,Phrase1),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(koht1_50(Phrase1,Phrase2);
	 ((in_context(Phrase1,siin,_,Phrase3);
	   Phrase3=Phrase1),
	  (in_context(Phrase3,objekt12,Word,Phrase2);in_context(Phrase3,objekt2,Word,Phrase2)),
	update(ccx, [Word]))),
	(ajamäärus(Phrase2,Rest); Rest=Phrase2),
	reaction(ToR, FromR,ole),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% tule
	in_context(Phrase,tule,_,Phrase0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(in_context(Phrase0,viisimäärus1,_,Tail1);ajamäärus(Phrase0,Tail1);Tail1=Phrase0),
	(in_context(Tail1,kohamäärus1,_,Rest);
	   in_context(Tail1,isik_2,Word,Tail2),update(cc0,[Word]),in_context(Tail2,kohamäärus3,_,Tail3)),
	   (ajamäärus(Tail3,Rest); Rest=Tail3),
	reaction(ToR, FromR,tule1),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% tule
	in_context(Phrase,tule,_,Phrase0), update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(in_context(Phrase0, viisimäärus1,_,Phrase1);ajamäärus(Phrase0,Phrase1);Phrase0=Phrase1),
	(in_context(Phrase1,objekt2,Object,Phrase2),update(cc2,[Object]),in_context(Phrase2,kohamäärus3,_,Rest);Phrase1=Rest),
	reaction(ToR, FromR,tule2),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% Leia|otsi
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	in_context(Phrase,leia,_,Phrase0),
	(in_context(Phrase0,isik_1,Word,Phrase1), update(cc0,[Word]);		% kes
	in_context(Phrase0,objekt1,Word, Phrase1), update(cc1,[Word])),		% või mis
	update(ccx,[Word]),
	(in_context(Phrase1,viisimäärus1,_,Phrase2);Phrase2=Phrase1),		% kuidas
	in_context(Phrase2, koht1_6_1_9,Koht,Phrase21),
	update(cc2,[Koht,Phrase21]),
	(koht1_6(Phrase2,Phrase3);koht1_9(Phrase2,Phrase3);Phrase2=Phrase3), % mis kohast
	(Phrase3=[Word1|Rest],search_match(Word1,_,[üles]);Phrase3=Rest),
	reaction(ToR, FromR,otsi_leia),!.
interpret_command(ToR, FromR,Phrase,Rest):-
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	in_context(Phrase,leia,_,Phrase0),		% Leia|otsi
	(in_context(Phrase0,viisimäärus1,_,Phrase2);Phrase0=Phrase2),		% kuidas
	in_context(Phrase2, koht1_6_1_9,Koht,Phrase21),
	update(cc2,[Koht,Phrase21]),
	(koht1_6(Phrase2,Phrase3);koht1_9(Phrase2,Phrase3);Phrase2=Phrase3), % mis kohast
	(in_context(Phrase3,isik_1,Word,Phrase4), update(cc0,[Word]);			% kes
	in_context(Phrase3,objekt1,Word,Phrase4), update(cc1,[Word])),			% või mis
	update(ccx,[Word]),
	(in_context(Phrase4,üles,_,Rest);Rest=Phrase4),
	reaction(ToR, FromR,otsi_leia),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% ulata
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	in_context(Phrase,ulata,_,Phrase0),
	(in_context(Phrase0,see,_,Phrase1);Phrase0=Phrase1),
	(in_context(Phrase1,objekt1,Object,Phrase2),update(cc1,[Object]);Phrase2=Phrase1),
	(in_context(Phrase2,siia,_,Rest); in_context(Phrase2,asesõna_7,_,Rest);
	in_context(Phrase2,isik_7,Isik,Rest),update(cc0,[Isik]); Rest=Phrase2),
	reaction(ToR, FromR,anna_ulata),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% võta
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	in_context(Phrase,haara,_,Phrase0),(in_context(Phrase0,isik_9,_,Phrase1);Phrase0=Phrase1),
	(in_context(Phrase1,see,_,Phrase2);Phrase1=Phrase2),
	(in_context(Phrase2,objekt1,Object,Phrase3),update(cc1,[Object]);Phrase2=Phrase3),
	(in_context(Phrase3,oma,_,Phrase4),in_context(Phrase4,kätte,_,Rest);
	 in_context(Phrase3,endale,_,Rest);Rest=Phrase3),
	reaction(ToR, FromR,võta_haara),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% pane
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	in_context(Phrase,pane,_,Phrase0),
	((in_context(Phrase0,see,_,Phrase1);Phrase1=Phrase0),
	(in_context(Phrase1,objekt1,Object,Phrase2),update(cc1,[Object]))
	;
	(in_context(Phrase0,see,_,Phrase2);	Phrase2=Phrase0)),
	(in_context(Phrase2,peale,_,Rest);
	 in_context(Phrase2,objekt2,Object1,Phrase3),update(cc2,[Object1]),
	 in_context(Phrase3,peale,_,Rest)),
	reaction(ToR, FromR,tõsta_pane),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% too
	in_context(Phrase,too,_,Phrase0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(in_context(Phrase0,see,_,Phrase1);Phrase1=Phrase0),
	(in_context(Phrase1,objekt1,Object,Phrase2),update(cc1,[Object]); Phrase2=Phrase1),
	(koht1_6(Phrase2,Phrase3);koht1_9(Phrase2,Phrase3),in_context(Phrase2,koht1_6_1_9,Koht,_),update(cc2,[Koht]); Phrase3=Phrase2),
	(in_context(Phrase3,asesõna_7,_,Phrase4);in_context(Phrase3,isik_7,Isik,Phrase4),update(cc0,[Isik]);Phrase4=Phrase3),
	(koht1_6(Phrase4,Rest);koht1_9(Phrase4,Rest),in_context(Phrase4,koht1_6_1_9,Koht,_), update(cc2,[Koht]);Rest=Phrase4),
	reaction(ToR, FromR,too),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% too
	in_context(Phrase,too,_,Phrase0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(in_context(Phrase0,siia,_,Phrase1); in_context(Phrase0,asesõna_7,Keegi,Phrase1),update(cc0,[Keegi]);
	in_context(Phrase0,isik_7,Keegi,Phrase1),update(cc0,[Keegi]);Phrase1=Phrase0),
	(koht1_6(Phrase1,Phrase2); koht1_9(Phrase1,Phrase2),in_context(Phrase1,koht1_6_1_9,Koht,_), update(cc2,[Koht]); Phrase2=Phrase1),
	(in_context(Phrase2,objekt1,Object,Rest),update(cc1,[Object]); Rest=Phrase2),
	reaction(ToR, FromR,too),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% too
	in_context(Phrase,too,_,Phrase0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(koht1_6(Phrase0,Phrase1);koht1_9(Phrase0,Phrase1),in_context(Phrase0,koht1_6_1_9,Koht,_),update(cc2,[Koht]);Phrase1=Phrase0),
	(in_context(Phrase1,asesõna_7,_,Phrase2);in_context(Phrase1,isik_7,Isik,_),update(cc0,[Isik]);Phrase2=Phrase1),
	(in_context(Phrase2,objekt1,Object,Rest),update(cc1,[Object]);Rest=Phrase2),
	reaction(ToR, FromR,too),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% vii
	in_context(Phrase,vii,_,Phrase0),
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),
	(in_context(Phrase0,see,_,Phrase1);Phrase1=Phrase0),
	(in_context(Phrase1,objekt1,Object,Phrase2),update(cc1,[Object]);Phrase2=Phrase1),
	(in_context(Phrase2,ära,Word,Rest),update(cc2,[Word]);
	in_context(Phrase2,asesõna_7,_,Rest);
	 in_context(Phrase2,isik_7,Isik,Rest),update(cc0,[Isik]),update(ccx,[Isik]);
	 in_context(Phrase2,isik_2,Isik,Phrase3),in_context(Phrase3,kätte,_,Rest),
	 update(cc0,[Isik]),update(ccx,[Isik]);
	 koht1_4(Phrase2,Rest);
	 koht1_11(Phrase2,Rest);
	 Rest=Phrase2),
	update(cc3,[now]),
	reaction(ToR, FromR,vii),!.
interpret_command(ToR, FromR,Phrase,Rest):-								% liigu, mine
	update(cc3,[now]),update(cc3TXT,[esimesel,võimalusel]),				% -----ajamääruse blokk
	in_context(Phrase,mine,_,Phrase0),
	  (in_context(Phrase0,viisimäärus1,_,Phrase1);
	   in_context(Phrase0,ajamäärus,_,Phrase1);
	   Phrase1=Phrase0),
	(in_context(Phrase1,kohamäärus1,_,Rest),update(cc2,Phrase1); 							%------kohamääruse blokk
	 koht1_4(Phrase1,Rest);
	 (in_context(Phrase1,objekt2,Object,Phrase2),update(cc2,[Object]);
	 in_context(Phrase1,isik_2,Keegi,Phrase2),update(cc0,[Keegi])), update(ccx,[Keegi]),
	 (in_context(Phrase2,juurde,_,Rest));
	 Rest=Phrase1),
	reaction(ToR, FromR,liigu_mine),!.
interpret_command(_, _,_,_):-
	write('Ei saanud sellest lausest aru'), nl,
	fail.

% ==================================== END ROBOTILE ANTAVATE KÄSKUDE INTERPRETAATOR ===============================

in_context([Word|Tail],Context_name,Context_obj,Tail):-
	context(Context_name,Context_List),
	search_match(Word,Context_obj,Context_List),!.


%==================================== SISENDKÄSKUDE INTERPRETEERIMISEKS VAJALIK TERMINALIDE GRAMMATIKA ========================
context(verb1,    [sõitma, liikuma, minema, tulema, otsima]).
context(verb2,    [asub,seisab,asetseb,paikneb,viibib,on]).
context(objekt1,  [raamat,pall,asi,tass,pliiats,õlu,kohv,tee]).
context(objekt12, [palli,asja,tassi,pliiatsi]).
context(objekt21, [laud,tool,uks,aken]).
context(objekt2,  [laua, tooli, ukse, akna]).
context(isik_1,   [evelin, gert, marko, oliver, külaline, teadur]).
context(isik_2,   [evelini, gerdi, marko, oliveri, külalise, minu, tema, nende]).
context(isik_7,   [evelinile, gerdile, markole, oliverile, minule]).
context(isik_9,   [evelinilt,gerdilt,markolt,oliverilt,minult,talt]).
context(asesõna_7,[mulle,talle,meile,teile,neile,endale]).
context(koht1_4,  [kohvituppa, koridori, tuppa]).
context(juhata,   [juhata]).
context(ütle,	  [ütle]).
context(kus,	  [kus]).
context(ole,	  [ole]).
context(tule,	  [tule]).
context(leia,	  [leia, otsi]).
context(too,	  [too,tari, tiri, vea]).
context(viisimäärus1,[kohe, ruttu, viivitamata, aeglasemalt, kiiremini, kiiresti, aegalselt]).
context(kohamäärus1, [siia, edasi, otse, sinna, paremale, vasakule, tagasi, kõrvale, kaugemale, lähemale]).
context(kohamäärus2, [juures, kõrval, lähedal, ees]).
context(kohamäärus3, [juurde, kõrvale, lähedale, ette]).
context(siin,	[siin]).
context(roll,	[teejuhiks]).
context(koht1_6_1_9, [põrandalt,laualt,seinalt,ukselt,aknalt,toolilt,diivanilt, kohvitoast, koridorist, toast]).
context(üles,   [üles]).
context(ulata,	[anna, ulata]).
context(see,	[see]).
context(siia,	[siia]).
context(haara,	[haara,võta]).
context(oma,	[oma,enda]).
context(kätte,	[kätte]).
context(endale, [endale, omale]).
context(pane,	[tõsta, pane, aseta]).
context(peale,	[peale,kõrvale, juurde,üles,sinna]).
context(ära,	[ära]).
context(mine,	[mine, liigu]).
context(juurde, [juurde,poole]).
context(toas,	[toas]).
context(vii,	[vii, toimeta, kanna]).

koht1_4(Phrase, FraasN):-
	in_context(Phrase,koht1_4,Word1,Fraas),
	update(cc2TXT,[Word1|Fraas]),
	(Word1=kohvituppa,  update(cc2,[kohvituba]),update(ccx,[kohvituba]),FraasN=Fraas;
	 Word1=koridori,    update(cc2,[koridor]),  update(ccx,[koridor]),  FraasN=Fraas;
	 Word1=tuppa, (Fraas=[Word2|Fraas1], search_match(Word2,_,[number]);Fraas1=Fraas),
	    room_number(Fraas1,FraasN),	retract(cc2TXT(T)),asserta(cc2TXT([tuppa|T]))).
koht1_5(Phrase,FraasN):-
	(in_context(Phrase,objekt12,_,Rest);in_context(Phrase,objekt2,_,Rest)),
	in_context(Rest,kohamäärus2,_,FraasN).
koht1_50([Word|Fraas], FraasN):-
	search_match(Word,W0,[kohvitoas,koridoris,toas]),
	(W0=kohvitoas, update(cc2,[kohvituba]),update(ccx,[kohvituba]),update(cc2TXT,[kohvitoas]),FraasN=Fraas;
	W0=koridoris, update(cc2,[koridor]),update(ccx,[koridor]),update(cc2TXT,[koridoris]),FraasN=Fraas;
	W0=toas,Fraas=[Word1|Fraas1], search_match(Word1,_,[number]), room_number(Fraas1,FraasN),retract(cc2TXT(T)),asserta(cc2TXT([toas|T]));
	W0=toas, room_number(Fraas,FraasN),retract(cc2TXT(T)),asserta(cc2TXT([toas|T]))).
koht1_6([Word|Fraas], FraasN):-
	search_match(Word,W0,[kohvitoast,koridorist,toast]),
	update(cc2TXT,[W0|Fraas]),
	(W0=kohvitoast, update(cc2,[kohvituba]),FraasN=Fraas;
	 W0=koridorist, update(cc2,[koridor]),  FraasN=Fraas;
	 W0=toast, Fraas=[Word1|Fraas1], search_match(Word1,_,[number]), room_number(Fraas1,FraasN),retract(cc2TXT(T)),asserta(cc2TXT([toast|T]));
	 W0=toast, room_number(Fraas,FraasN)),
	retract(cc2TXT(T)),
	asserta(cc2TXT([toast|T])).
koht1_9([Word|Fraas], Fraas):-
	search_match(Word,_,[põrandalt,laualt,seinalt,ukselt,aknalt,toolilt,diivanilt]).
koht1_11([Word|Fraas], FraasN):-
	search_match(Word,W0,[kohvitoani,koridorini,ukseni,toani]),
	update(cc2TXT,[W0|Fraas]),
	(W0=kohvitoani, update(cc2,[kohvituba]),FraasN=Fraas;
	W0=koridorini, update(cc2,[koridor]),  FraasN=Fraas;
	W0=ukseni, FraasN=Fraas;
	W0=toani, Fraas=[Word1|Fraas1], search_match(Word1,_,[number]), room_number(Fraas1,FraasN),retract(cc2TXT(T)),asserta(cc2TXT([toani|T]));
	W0=toani,room_number(Fraas,FraasN),retract(cc2TXT(T)),asserta(cc2TXT([toani|T]))).

room_number(N,Rest):-
	N=[N1,N2|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[üheksateist]),
		update(cc2,[ruum_419]),update(ccx,[ruum_419]), update(cc2TXT,[nelisada,üheksateist]);	% jüri
	N=[N1,N2|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),
		update(cc2,[ruum_420]),update(ccx,[ruum_420]), update(cc2TXT,[nelisada,kakskümmend]);		% olaf
	N=[N1,N2,N3|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),search_match(N3,_,[üks]),
		update(cc2,[ruum_421]),update(ccx,[ruum_421]), update(cc2TXT,[nelisada,kakskümmend,üks]);	% gert
	N=[N1,N2,N3|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),search_match(N3,_,[kaks]),
		update(cc2,[ruum_422]),update(ccx,[ruum_422]), update(cc2TXT,[nelisada,kakskümmend,kaks]);	% tarmo
	N=[N1,N2,N3|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),search_match(N3,_,[kolm]),
		update(cc2,[ruum_423]),update(ccx,[ruum_423]), update(cc2TXT,[nelisada,kakskümmend,kolm]);	% evelin
	N=[N1,N2,N3|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),search_match(N3,_,[neli]),
		update(cc2,[ruum_424]),update(ccx,[ruum_424]), update(cc2TXT,[nelisada,kakskümmend,neli]);	% sven
	N=[N1,N2,N3|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),search_match(N3,_,[viis]),
		update(cc2,[ruum_425]),update(ccx,[ruum_425]), update(cc2TXT,[nelisada,kakskümmend,viis]);	% jaagup
	N=[N1,N2,N3|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),search_match(N3,_,[kuus]),
		update(cc2,[ruum_426]),update(ccx,[ruum_426]), update(cc2TXT,[nelisada,kakskümmend,kuus]);	% tanel
	N=[N1,N2,N3|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),search_match(N3,_,[seitse]),
		update(cc2,[ruum_427]),update(ccx,[ruum_427]), update(cc2TXT,[nelisada,kakskümmend,seitse]); % marko
	N=[N1,N2,N3|Rest], search_match(N1,_,[nelisada]),search_match(N2,_,[kakskümmend]),search_match(N3,_,[kaheksa]),
		update(cc2,[ruum_428]),update(ccx,[ruum_428]), update(cc2TXT,[nelisada,kakskümmend,kaheksa]); % juhan
	N=Rest.

room([nelisada,üheksateist], ruum_419).			% jüri
room([nelisada,kakskümmend], ruum_420).			% olaf
room([nelisada,kakskümmend,üks], ruum_421).		% gert
room([nelisada,kakskümmend,kaks], ruum_422).	% tarmo
room([nelisada,kakskümmend,kolm], ruum_423).	% evelin
room([nelisada,kakskümmend,neli], ruum_424).	% sven
room([nelisada,kakskümmend,viis], ruum_425).	% jaagup
room([nelisada,kakskümmend,kuus], ruum_426).	% tanel
room([nelisada,kakskümmend,seitse], ruum_427).	% marko
room([nelisada,kakskümmend,kaheksa],ruum_428).	% juhan

ajamäärus([Word|Fraas], Fraas1):-
	search_match(Word,_,[varsti]);
	aeg([_|Fraas], Fraas1).

%-------------------AEG-------------------------
aeg(Aeg,Rest):-
	(Aeg=[kell|Näit];Aeg=Näit),
	kella_näit(Näit,Rest),
	retract(tundTXT(H)),retract(minutTXT(M)),append(H,M,HM),
	retract(cc3TXT(_)),asserta(cc3TXT(HM)),
	retract(tund(HD)),retract(minut(MD)),
	retract(cc3(_)),asserta(cc3([HD,MD])).

kella_näit(H,Rest):- tund(H,M),minut(M,Rest).

% test:-
% kella_näit([null,null,viiskümmend,seitse],_),tundTXT(H),minutTXT(M),append(H,M,HM),write(HM),tund(HD),minut(MD),write([HD,MD]).
%

tund([null,null|Rest],Rest):-	% 00:00-00:59
	asserta(tund(0)),asserta(tundTXT([null,null])),!.
tund([null,H2|Rest],Rest):-		% 01:00-09:59
	arv(H2,D), D > 0, D =<9,
	asserta(tund(D)),asserta(tundTXT([null,H2])),!.
tund([H1,H2|Rest],Rest):-		% 21:00-23:59
	arv(H1,D1), D1 = 20,
	arv(H2,D2), D2 >= 1, D2 =< 4,
	D is D1 + D2,
	asserta(tund(D)),asserta(tundTXT([H1,H2])),!.
tund([H|Rest],Rest):-			% 1:00-20:59
	arv(H,D), D >= 1, D =< 20,
	asserta(tund(D)),asserta(tundTXT([H])),!.

minut([null,null|Rest],Rest):-
	asserta(minut(0)),asserta(minutTXT([null,null])),!.
minut([null,M|Rest],Rest):-
	arv(M,D), D>=0, D=<9,
	asserta(minut(D)),asserta(minutTXT([null,M])),!.
minut([M1,M2|Rest],Rest):-
	arv(M1,D1), (D1=20;D1=30;D1=40;D1=50),
	arv(M2,D2),  D2>=1, D2=<9,
	D is D1 + D2,
	asserta(minut(D)),asserta(minutTXT([M1,M2])),!.
minut([M|Rest],Rest):-			% 1:00-20:59
	arv(M,D), (D>=1, D=<20; D=30; D=40; D=50),
	asserta(minut(D)),asserta(minutTXT([M])),!.

arvsõna1(Arv,Rest):- Arv=[üheksamkümmend|Rest]; Arv=[sada|Kümnelised], Kümnelised=[kaheksakümmend|Rest].

%------Arvud------------
arv(üks, 1). arv(kaks, 2). arv(kolm, 3). arv(neli, 4). arv(viis, 5). arv(kuus, 6). arv(seitse, 7). arv(kaheksa, 8). arv(üheksa, 9). arv(kümme, 10).
arv(üksteist, 11). arv(kaksteist, 12). arv(komteist, 13). arv(neliteist, 14). arv(viisteist, 15). arv(kuusteist, 16). arv(seitseteist, 17). arv(kaheksateist, 18). arv(üheksateist, 19).
arv(kakskümmend, 20). arv(kolmkümmend, 30). arv(nelikümmend, 40). arv(viiskümmend, 50).
%======================== END SISENDKÄSKUDE INTERPRETEERIMISEKS VAJALIK GRAMMATIKA ========================

% Test: search_match(ukset,[laua, tooli, ukse, akna]).

search_match(Word,Word,Vocabulary):-
	member(Word,Vocabulary), !.
search_match(Word,Best_match, Vocabulary):-
	atom_string(Word, WordString),									% siis hakka otsima leksikaliselt lähimat
	string_chars(WordString,WordChars),								% muuda sisendi sõna tähtede stringiks
	search_match1(Vocabulary,WordChars,Best_match, Similarity),
	similarity_treshold(Treshold),
	Similarity >= Treshold,
	write('CONTEXT-BASED CORRECTION: '),write(Word), write('-->'), write(Best_match),nl,!.
%search_match(Word, _,_):-
%	nl, write(['Sõna ', Word,'on mulle selles kontekstis täiesti tundmatu']),fail.

search_match1([], _,xxx,0):- !.										% kõik antud sõna vormid vaadatud
search_match1([Vorm|VormideL],WordChars,Best_match, Similarity):-
	search_match1(VormideL, WordChars,Best_match1, Similarity1),	% võta jooksva sõnastiku sõna järgmine vorm
		atom_string(Vorm, VormString),								% sõna stringiks 1
		string_chars(VormString,VormChars),							% String tähtede listiks 2
		calculate_similarity1(WordChars,VormChars,SimilarityX),
		((SimilarityX > Similarity1,
		Similarity = SimilarityX,									% uuenda parimat, kui vaja
		Best_match = Vorm);
		(Similarity =Similarity1,
		Best_match =Best_match1)),!.

%======================== ROBOT's REACTION DESCRIPTIONS ====================
reaction(ToR, FromR,juhata):-
	cc0(CC0),
	retract(world(CC0, Idx0, _, XYZQ0,_)),	% keda juhatada
	goto(ToR,CC0,XYZQ0), wait(FromR),		% mine juhatava juurde
	retract(world(me,Idx,_,_,_)),
	get_time(T0),
	asserta(world(me,Idx,Idx0,XYZQ0,T0)),
	cc2(CC2),cc2TXT(CC2P),
	append([CC0,palun, järgnege, mulle, juhatan, teid],CC2P,Text1),
	respond(ToR,Text1),
	find_case(CC2Nim,_,CC2),
	world(CC2Nim, Idx2, _, XYZQ2,_),
	goto(ToR,CC2Nim,XYZQ2),  wait(FromR),		% käsk navisüsteemile
	respond(ToR,[oleme,kohal]),
	retract(world(me,Idx,_, _,_)),
	get_time(T1),
	asserta(world(me,Idx,Idx2, XYZQ2,T1)),
	asserta(world(CC0, Idx0, Idx2, XYZQ2,T1)),!.
reaction(ToR, _,ütle_kus1):-
	ccx(CCx),
	world(CCx,_,EIdx,_,_),
	world(Koht,EIdx,_,_,_),
	(Koht=kohvituba,KohtX=[kohvitoas];
	Koht=koridor,KohtX=[koridoris];
	room(KohtLong,Koht),KohtX=[toas|KohtLong];
	find_case(Koht,2, KohtOm),KohtX=[KohtOm,läheduses]),
	append([CCx,asus,viimati],KohtX,Text2),
	respond(ToR,Text2),!.
reaction(ToR, _,ütle_kus2):-
	ccx(CCx),
	world(CCx,_,EIdx, _,_),
	world(Koht, EIdx, _, _, _),
	cases(Koht,_,Vormid),
	nth1(8, Vormid, Kohal),			% leia alalütlev kääne
	respond(ToR,[CCx, asub, Kohal]),!.
reaction(ToR, _,ütle_kus3):-
	cc0(CC0),
	world(CC0,_,EIdx,_,_),
	world(Koht,EIdx,_,_,_),
	(Koht=kohvituba,KohtX=[kohvitoas];
	Koht=koridor,KohtX=[koridoris];
	room(KohtLong,Koht),KohtX=[toas|KohtLong];
	find_case(Koht,2, KohtOm),KohtX=[KohtOm,juures]),
	append([CC0,asus,viimati],KohtX,Text),
	respond(ToR,Text),!.
reaction(ToR, FromR,ole):-
	ccx(CCx),
	find_case(Koht,_,CCx),
	find_case(Koht,2,KohtOm),
	world(Koht, _, _, XYZQ, _),
	(Koht=kohvituba,KohtX=[kohvitoas];
	Koht=koridor,KohtX=[koridoris];
	room(KohtLong,Koht),KohtX=[toas|KohtLong];
	find_case(Koht,2, KohtOm),KohtX=[KohtOm,juures]),
	append([hästi, olen],KohtX,Text1),
	cc3TXT(HM),
	append(Text1,HM,Text),
	respond(ToR,Text),
	goto(ToR, Koht, XYZQ), wait(FromR), !.
reaction(ToR, FromR,tule1):-
	cc0(CC0), cc3(CC3),cc3TXT(CC3TXT),
	find_case(CC0Nim,_,CC0),
	find_case(CC0Nim,2,_),
	world(CC0Nim,Idx0,_,XYZQ0,_),
	retract(world(me,Idxme,_,_,_)),
	append([sain,aru,tulen],CC3TXT,Answer),
	respond(ToR,Answer),
	asserta(world(me,Idxme,Idx0,XYZQ0,CC3)),
	goto(ToR, CC0Nim,XYZQ0), wait(FromR),!.
reaction(ToR, _,tule2):-
	cc2(CC2), cc3(CC3),
	find_case(CC2Nim,_,CC2),
	world(CC2Nim,Idx2,_,XYZQ2,_),
	retract(world(me,Idx0,_,_,_)),
	asserta(world(me,Idx0,Idx2,XYZQ2,CC3)),
	respond(ToR,[sain,aru,tulen,kohe,CC2,juurde]),
	goto(ToR, CC2Nim,XYZQ2), !.
reaction(ToR, FromR,otsi_leia):-
	ccx(CCx), cc2(CC2),
	find_case(CC2Nim,_,CC2),
	find_case(CCxNim,_,CCx),
	find_case(CCxNim,3,CCxOs),
	respond(ToR,[hästi,lähen,CCxOs,CC2, otsima]),
	world(CC2Nim,_,_,XYZQ2,_),
	goto(ToR,CC2Nim, XYZQ2), wait(FromR),
	world(CCx,_,_,XYZQx,_),
	goto(ToR,CCx,XYZQx), wait(FromR),
	respond(ToR,[näe,leidsingi]),!.
reaction(ToR, FromR,anna_ulata):-
	cc0(CC0), cc1(CC1),
	find_case(CC0Nim,_,CC0),
	world(CC0Nim, Idx0, _, XYZQ0, _),
	world(CC1,Idx,_,_,_),
	respond(ToR,[CC0Nim, palun, võta, see, CC1]),
	place_to(ToR,CC1, XYZQ0), wait(FromR),
	retract(world(CC1,Idx,_,_,_)),
	get_time(Time),
	asserta(world(CC1,Idx,Idx0,XYZQ0,Time)),!.
reaction(ToR, _,võta_haara):-
	cc0(CC0),
	find_case(CC0Nim,_,CC0),
	world(CC0Nim, _, _, XYZQ0, _),
	cc1(CC1),
	retract(world(CC1,Idx1,_,_,_)),
	pick(ToR,CC1, XYZQ0),
	world(me,Idxme,_,XYZQme,_),
	get_time(Time),
	asserta(world(CC1,Idx1,Idxme,XYZQme,Time)),
	respond(ToR,[tänan]),!.
reaction(ToR, FromR,tõsta_pane):-
	cc1(CC1),
	cc2(CC2),
	find_case(CC2Nim,_,CC2),
	world(CC2Nim,Idx2,_,XYZQ2,_),
	respond(ToR,[hästi, panen]),
	place_to(ToR,CC1, XYZQ2),
	wait(FromR),
	retract(world(CC1,Idx1,_,_,_)),
	get_time(TS),
	asserta(world(CC1,Idx1,Idx2,XYZQ2,TS)),!.
reaction(ToR, FromR,too):-
	cc0(CC0),			% Kellele tuua
	cc1(CC1),			% Mida tuua
	cc2(CC2),			% Kust tuua
	find_case(CC2Nim,_,CC2),
	respond(ToR,[hästi, lähen, tooma]),
	world(CC2Nim,_,_, XYZQ2,_),	% Leia koha koordinaadid kust tuua
	goto(ToR,CC2Nim,XYZQ2),	wait(FromR),
	world(CC1,_,_,XYZQ1,_),
	pick(ToR,CC1,XYZQ1), wait(FromR),
	find_case(CC0Nim,_,CC0),
	world(CC0Nim,Idx0,_,XYZQ0,_),
	goto(ToR,CC0Nim,XYZQ0), wait(FromR),
	respond(ToR,[palun, võta,CC1]),
	place_to(ToR,CC1, XYZQ0), wait(FromR),
	retract(world(CC1,Idx1,_,_,_)),
	get_time(TS),
	asserta(world(CC1,Idx1,Idx0,XYZQ0,TS)),!.
reaction(ToR, FromR,vii):-
	cc1(CC1), ccx(CCx),
	world(me,_,_,XYZQme,_),
	world(CC1,_,_,XYZQ1,_),
	find_case(CCxNim,_,CCx),
	world(CCxNim,Idxx,_,XYZQx,_),
	find_case(CC1,2,CC1Om),
	respond(ToR,[hästi, viin, CC1Om,ära]),
	pick(ToR,CC1, XYZQ1), wait(FromR),
	goto(ToR,CCxNim,XYZQx), wait(FromR),
	respond(ToR,[palun,võta,CC1]),
	place_to(ToR,CC1, XYZQx), wait(FromR),
	retract(world(CC1,Idx1,_,_,_)),
	get_time(TS),
	asserta(world(CC1,Idx1,Idxx,XYZQx,TS)),
	goto(ToR,speaker,XYZQme), wait(FromR), % R tuleb tagasi
	find_case(CCxNim,7,_),
	respond(ToR,[CC1,on,viidud]),!.
reaction(ToR, FromR,liigu_mine):-
%	ccx(CCx),
	cc2(CCx),
	find_case(CCxNim,_,CCx),
	world(CCxNim,Idx,_,XYZQ,_),
	respond(ToR,[sain,aru,hakkan,minema]),
	goto(ToR,CCxNim,XYZQ),
	retract(world(me,Idxme,_,_,_)),
	get_time(TS), wait(FromR),
	respond(ToR,[jõudsin,kohale]),
	asserta(world(me,Idxme,Idx,XYZQ,TS)),!.
%reaction(ToR, FromR,A):-
%	respond(ToR,[tegevus,A,ebaõnnestus,ootan, uut, korraldust]),nl.
%======================== END ROBOT's REACTION DESCRIPTIONS ====================


% ======================= UPDATING CONTEXT VARIABLES ===========================
update(cc3TXT,CC3TXT):-
	retract(cc3TXT(_)),
	asserta(cc3TXT(CC3TXT)),!.
update(cc2TXT,CC2P):-
	retract(cc2TXT(_)),
	asserta(cc2TXT(CC2P)),!.
update(Context,[Entity|_]):-		% Kontekstimuutuja värskendamine
	TermO=.. [Context,_],
	find_case(Nim,_,Entity),
	TermN=.. [Context, Nim],
	retract(TermO),
	asserta(TermN),!.
% ======================= END UPDATING CONTEXT VARIABLES =======================


%============================= ROBOT COMMANDS =============================
goto( ToR,Place,XYZQ):-			% mine positsioonile XYZQ
	open( ToR, append, H2, [alias(output_file)]),
	cc3(CC3),
	write(H2,'GOTO: '), write(H2,[Place,with,coordinates,XYZQ,CC3]),nl(H2),
	write('GOTO: '),    write([Place,with,coordinates,XYZQ,CC3]),nl,
	close(H2),!.
pick( ToR,Object,XYZQ):-		% objekti haaramine positsioonilt
	open( ToR, append, H2, [alias(output_file)]),
	write(H2,'PICK: '),  write(H2,[Object,from, XYZQ]), nl(H2),
	write('PICK: '),     write([Object,from, XYZQ]),	nl,
	close(H2),!.
place_to( ToR, Object, XYZQ):-	% objekti asetamine positsioonile XYZQ
	open( ToR, append, H2, [alias(output_file)]),
	write(H2,'PLACE: '), write(H2,[Object,to, XYZQ]),	nl(H2),
	write('PLACE: '),	 write([Object,to, XYZQ]),		nl,
	close(H2),!.
respond( ToR, Text):-
	open( ToR, append, H2, [alias(output_file)]),
	write(H2,'RESPOND: '), write(H2,Text),	nl(H2),
	write('RESPOND: '),	write(Text),	nl,
	close(H2),!.										% robot vastab lausega
%============================= END ROBOT COMMANDS =============================


% ==================== CURRENT CONTEXT VARIABLES ==============================
% Reflect current status of the world as robot knows it (universe of discourse)
% =============================================================================
cc0(oliver).						% dom(cc0): set of subjects
cc1(tass).							% dom(cc1): set of objects
cc2(ruum_411).						% dom(cc2): set of spatial locations
cc2TXT([ruum,nelisada,üksteist]).	% dom(cc2TXT): spatial location in textual form
cc3(now).							% dom(cc3): set of temporal locations
cc3TXT([esimesel,võimalusel]).		% Temporal location in textual form
ccx(koridor).						% auxilliary context variable
%==============================================================================

%----------------  Entities of context cc0 --------------------------
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

% Entities of context cc1 --------------------------
world(tass,     id31,id28, (18,45,5,0,0,0,1), 0).
world(kauss,	id32,id27, (1,14,0,0,0,0,1),  0).
world(lusikas,	id33,id43, (1,14,0,0,0,0,1),  0).
world(nuga,     id34,id43, (1,14,0,0,0,0,1),  0).
world(pall,     id35,id27, (1,14,0,0,0,0,1),  0).
world(raamat,   id36,id29, (1,14,0,0,0,0,1),  0).
world(suss,	id37,id12, (1,14,0,0,0,0,1),  0).
world(pastakas,	id38,id12, (1,14,0,0,0,0,1),  0).
world(väljatrükk,id39,id23, (1,14,0,0,0,0,1), 0).
world(paber,	id40, id23, (1,14,0,0,0,0,1), 0).
world(ajaleht,	id41, id29, (1,14,0,0,0,0,1), 0).
world(ajakiri,	id42, id29, (1,14,0,0,0,0,1), 0).

% Entities of context cc2 --------------------------
world(koridor,  id11,[],  (36,13,3,0,0,0,1),  0).
world(ruum_419, id12,[],  (8,4,0,0,0,0,1),    0).
world(ruum_420, id43,[],  (10,3,0,0,0,0,1),    0).
world(ruum_421, id13,[],  (14,3,0,0,0,0,1),  0).
world(ruum_423, id14,[],  (20,3,0,0,0,0,1),    0).
world(ruum_424, id15,[],  (23,3,0,0,0,0,1),  0).
world(ruum_425, id16,[],  (26,3,0,0,0,0,1),  0).
world(ruum_426, id17,[],  (29,4,0,0,0,0,1),  0).
world(ruum_427, id18,[],  (30,6,0,0,0,0,1),  0).
world(ruum_428, id19,[],  (33,7,0,0,0,0,1),  0).
world(kohvituba,id20,[],  (21,13,0,0,0,0,1),  0).

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
cases(inimene, [ns, _, elus], [inimene, inimese, inimest, inimesse, inimeses, inimesest, inimesele, inimesel, inimeselt, inimeseks, inimeseni, inimesena, inimeseta, inimesega, inimesed, inimeste, inimesi, inimestesse, inimestes, inimestest, inimestele, inimestel, inimestelt, inimesteks, inimesteni, inimestena, inimesteta, inimestega]).
cases(mees,  [ns, _, elus],  [mees, mehe, meest, mehesse, mehes, mehest, mehele, mehel, mehelt, meheks,  meheni,  mehena,  meheta, mehega,  mehed, meeste, mehi, meestesse, meestes, meestest, meestele, meestel, meestelt, meesteks, meesteni, meestena, meesteta, meestega]).
cases(naine, [ns, ains, elus], [naine, naise, naist, naisesse, naises, naisest, naisele, naisel, naiselt, naiseks, naiseni, naisena, naiseta, naisega, naised, naiste, naisi, naistesse, naistes, naistest, naistele, naistel, naistelt, naisteks, naisteni, naistena, naisteta, naistega]).
cases(laps, [ns, ains, elus], [laps, lapse, last, lapsesse, lapses, lapsest, lapsele, lapsel, lapselt, lapseks, lapseni, lapsena, lapseta, lapsega, lapsed, laste, lapsi, lastesse, lastes, lastest, lastele, lastel, lastelt, lasteks, lasteni, lastena, lasteta, lastega]).
cases(poiss, [ns, ains, elus], [poiss, poisi, poissi, poissi, poisis, poisist, poisile, poisil, poisilt, poisiks, poisini, poisina, poisita, poisiga, poisid, poiste, poisse, poistesse, poistess, poistesst, poistele, poistel, poistelt, poisteks, poisteni, poistena, poisteta, poistega]).
cases(tüdruk, [ns, ains, elus], [tüdruk, tüdruku, tüdrukut, tüdrukusse, tüdrukus, tüdrukust, tüdrukule, tüdrukul, tüdrukult, tüdrukuks, tüdrukuni, tüdrukuna, tüdrukuta, tüdrukuga, tüdrukud, tüdrukute, tüdrukuid, tüdrukutesse, tüdrukutes, tüdrukutest, tüdrukutele, tüdrukutel, tüdrukutelt, tüdrukuteks, tüdrukuteni, tüdrukutena, tüdrukuteta, tüdrukutega]).
cases(külaline, [ns, ains, elus], [külaline, külalise, külalist, külalisse, külalises, külalisest, külalisele, külalisel, külaliselt,külaliseks, külaliseni,külalisena, külaliseta, külalisega, külalised, külaliste, külalisi, külalistesse, külalistes, evelinidest,külalistele, külalistel, külalistelt, külalisteks, külalisteni, külalistena, külalisteta, külalistega]).

% ASESÕNAD -----------------
cases(mina, [ns, ains, elus], [mina, minu, mind, minusse, minus, minust, minule, minul, minult, minuks, minuni, minuna,  minuta, minuga, minad, minade, minasid,  minadesse, minades, minadest, minadele, minadel, minadelt, minadeks, minadeni, minadena, minadeta, minadega]).
cases(ma, [ns, ains, elus], [ma, mu, mind, musse, mus, must, mule, mul, mult, muks, muni, muna,  muta, muga, mud, mude, musid,  mudesse, mudes, mudest, mudele, mudel, mudelt, mudeks, mudeni, mudena, mudeta, mudega]).
cases(sina, [ns, ains, elus], [sina, sinu, sind, sinusse, sinus, sinust, sinule, sinul, sinult, sinuks, sinuni, sinuna,  sinuta, sinuga, sinad, sinade, sinasid,  sinadesse, sinades, sinadest, sinadele, sinadel, sinadelt, sinadeks, sinadeni, sinadena, sinadeta, sinadega]).
cases(sa, [ns, ains, elus], [sa, su, sind, susse, sus, sust, sule, sul, sult, suks, suni, suna,  suta, suga, sud, sude, susid,  sudesse, sudes, sudest, sudele, sudel, sudelt, sudeks, sudeni, sudena, sudeta, sudega]).
cases(tema, [ns, ains, elus], [tema, tema, teda, temasse, temas, temast, temale, temal, temalt, temaks, temani, temana,  temata, temaga, temad, temade, sinasid,  temadesse, temades, temadest, temadele, temadel, temadelt, temadeks, temadeni, temadena, temadeta, temadega]).
cases(ta, [ns, ains, elus], [ta, tema, teda, tasse, tas, tast, talle, tal, talt, taks, tani, tana,  tata, taga, tad, tade, tasid,  tadesse, tades, tadest, tadele, tadel, tadelt, tadeks, tadeni, tadena, tadeta, tadega]).
cases(meie, [ns, mitm, elus], [meie, meie, meid, meiesse, meis, meist, meile, meil, meilt, meieks, meieni, meiena, meieta, meiega, meied, meiede, meiesid, meiedesse, meiedes, meiedest, meiedele, meiedel, meiedelt, meiedeks, meiedeni, meiedena, meiedeta, meiedega]).
cases(me, [ns, mitm, elus], [me, me, meid, meisse, meis, mest, meile, meil, meilt, meiks, meni, mena, meta, mega, med, meiede, mesid, medesse, medes, medest, medele, medel, medelt, medeks, medeni, medena, medeta, medega]).
cases(teie, [ns, mitm, elus], [teie, teie, teid, teiesse, teies, teiest, teile, teil, teilt, teieks, teieni, teiena, teieta, teiega, teied, teiede, teiesid, teiedesse, teiedes, teiedest, teiedele, teiedel, teiedelt, teiedeks, teiedeni, teiedena, teiedeta, teiedega]).
cases(te, [ns, mitm, elus], [te, teie, teid, teisse, teis, teist, teile, teil, teilt, teiks, teini, teina, teita, teiga, teid, teite, teisid, teidesse, teites, teitest, teitele, teitel, teitelt, teiteks, teiteni, teiedena, teiteta, teitega]).
cases(nemad, [ns, mitm, elus], [nemad, nende, neid, nendesse, nendes, nendest, nendele, nendel, nendelt, nendeks, nendeni, nendena, nendeta, nendega, nemad, nemade, nemasid, nemadesse, nemades, nemadest, nemadele, nemadel, nemadelt, nemadeks, nemadeni, nemadena, nemadeta, nemadega]).
cases(nad, [ns, mitm, elus], [nad, nende, neid, nendesse, nendes, nendest, nendele, nendel, nendelt, nendeks, nendeni, nendena, nendeta, nendega, nemad, nemade, nemasid, nemadesse, nemades, nemadest, nemadele, nemadel, nemadelt, nemadeks, nemadeni, nemadena, nemadeta, nemadega]).

cases(kus,[ks,ains, eluta],[kus]).

% NIMED ------------------
cases(evelin, [ns, ains, elus], [evelin, evelini, evelini, evelinisse, evelinis, evelinist, evelinile, evelinil, evelinilt, eveliniks, evelinini, evelinina, evelinita, eveliniga,evelinid, evelinide, eveline, evelinidesse, evelinides, evelinidest, evelinidele, evelinidel, evelinidelt, evelinideks, evelinideni, evelinidena, evelinideta, evelinidega]).
cases(oliver, [ns, ains, elus], [oliver, oliveri, oliveri, oliverisse, oliveris, oliverist, oliverile, oliveril, oliverilt, oliveriks, oliverini, oliverina, oliverita, oliveriga,oliverid, oliveride, olivere,oliveridesse, oliverides, oliveridest, oliveridele, oliveridel, oliveridelt,oliverideks, oliverideni, oliveridena, oliverideta, oliveridega]).
cases(jüri, [ns, ains, elus], [jüri, jüri, jüri, jürisse, jüris, jürist, jürile, jüril, jürilt, jüriks, jürini, jürina, jürita, jüriga,jürid, jüride, jürisid, jüridesse, jürides, jüridest, jüridele, jüridel, jüridelt,jürideks, jürideni, jüridena, jürideta, jüridega]).
cases(gert, [ns, ains, elus], [gert, gerdi, gerti, gerdisse, gerdis, gerdist, gerdile, gerdil, gerdilt, gerdiks, gerdini, gerdina, gerdita, gerdiga,gerdid, gertide, gertisid, gertidesse, gertides, gertidest, gertidele, gertidel, gertidelt,gertideks, gertideni, gertidena, gertideta, gertidega]).
cases(sven, [ns, ains, elus], [sven, sveni, svenni, svenisse, svenis, svenist, svenile, svenil, svenilt, sveniks, svenini, svenina, svenita, sveniga,svenid, svennide, svenisid, svenidesse, svenides, svenidest, svenidele, svenidel, svenidelt,svenideks, svenideni, svenidena, svenideta, svenidega]).
cases(jaagup, [ns, ains, elus], [jaagup, jaagupi, jaagupit, jaagupisse,jaagupis, jaagupist, jaagupile, jaagupil, jaagupilt, jaagupiks, jaagupini, jaagupina, jaagupita, jaagupiga,jaagupid, jaagupite, jaagupeid, jaagupitesse, jaagupites, jaagupitest, jaagupitele, jaagupitel, jaagupitelt,jaagupiteks, jaagupiteni, jaagupitena, jaagupiteta, jaagupitega]).
cases(tanel, [ns, ains, elus], [tanel, taneli, tanelit, tanelisse,tanelis, tanelist, tanelile, tanelil, tanelilt, taneliks, tanelini, tanelina, tanelita, taneliga,tanelid, tanelite, taneleid, tanelitesse, tanelites, tanelitest, tanelitele, tanelitel, tanelitelt,taneliteks, taneliteni, tanelitena, taneliteta, tanelitega]).
cases(marko, [ns, ains, elus], [marko, marko, markot, markosse,markos, markost, markole, markol, markolt, markoks, markoni, markona, markota, markoga,markod, markode, markosid, markodesse, markodes, markodest, markodele, markodel, markodelt,markodeks,markodeni, markodena, markodeta, markodega]).
cases(juhan, [ns, ains, elus], [juhan, juhani, juhanit, juhanisse,juhanis, juhanist, juhanile, juhanil, juhanilt,juhaniks, juhanini, juhanina, juhanita, juhaniga,juhanid, juhanite, juhaneid, juhanitesse, juhanites, juhanitest, juhanitele, juhanitel, juhanitelt,juhaniteks,juhaniteni, juhanitena, juhaniteta,juhanitega]).

% ESEMED -----------------
cases(asi, [ns, ains, eluta], [asi, asja, asja, asjasse, asjas, asjast, asjale, asjal, asjalt, asjaks, asjani, asjana, asjata, asjaga, asjad, asjade, asju, asjadesse, asjades, asjadest, asjadele,	asjadel, asjadelt, asjadeks, asjadeni, asjadena, asjadeta, asjadega]).
cases(tass,  [ns, ains, eluta], [tass, tassi, tassi,  tassi,  tassis,  tassist,  tassile,  tassil,  tassilt,  tassiks,  tassini,  tassina,  tassita, tassiga, tassid,  tasside,  tassisid,  tassidesse,  tassides,  tassidest,  tassidele,  tassidel,  tassidelt,  tassideks,  tassideni,  tassidena,  tassideta, tassidega]).
cases(lusikas,[ns, ains, eluta], [lusikas, lusika, lusikat,  lusikasse, lusikas, lusikast, lusikale, lusikal,lusikalt, lusikaks,lusikani, lusikana, lusikata, lusikaga, lusikad, lusikate,  lusikaid, lusikatesse, lusikates, lusikatest,lusikatele, lusikatel, lusikatelt, lusikateks, lusikateni, lusikatena, lusikateta, lusikatega]).
cases(nuga, [ns, ains, eluta], [nuga, noa, nuga, noasse, noas, noast, noale, noal, noalt, noaks, noani, noana, noata, noaga, noad, nugade, nuge, nugadesse, nugadedes, nugadedest, nugadedele, nugadel, nugadelt, nugadeks, nugadeni, nugadena, nugadeta, nugadega]).
cases(kahvel, [ns, ains, eluta], [kahvel, kahvli, kahvlit, kahvlisse, kahvlis, kahvlist, kahvlile, kahvlil, kahvlilt, kahvliks, kahvlini, kahvlina, kahvlita, kahvliga, kahvlid, kahvlite,kahvleid, kahvlitesse, kahvlites, kahvlitest, kahvlitele, kahvlitel, kahvlitelt, kahvliteks,kahvliteni,kahvlitena,kahvliteta, kahvlitega]).
cases(raamat,  [ns, ains, eluta], [raamat, raamatu, raamatut,  raamatusse, raamatus, raamatust, raamatule, raamatul,raamatult, raamatuks, raamatuni, raamatuna, raamatuta, raamatuga, raamatud, raamatute,  raamatuid, raamatutesse, raamatutes, raamatutest, raamatutele, raamatutel, raamatutelt, raamatuteks, raamatuteni, raamatutena, raamatuteta, raamatutega]).
cases(pall,  [ns, ains, eluta], [pall, palli, palli,  palli,  pallis,  pallist,  pallile,  pallil,  pallilt,  palliks,  pallini,  pallina,  pallita, palliga, pallid,  pallide,  pallisid,  pallidesse,  pallides,  pallidest,  pallidele,  pallidel,  pallidelt,  pallideks,  pallideni,  pallidena,  pallideta, pallidega]).
cases(kell,  [ns, ains, eluta], [kell, kella, kella,  kellasse,  kellas,  kellast,  kellale,  kellal,  kellalt,  kellaks,  kellani,  kellana,  kellata, kellaga, kellad,  kellade,  kellasid,  kelladesse,  kellades,  kelladest,  kelladele,  kelladel,  kelladelt,  kelladeks,  kelladeni,  kelladena,  kelladeta, kelladega]).


% RUUMID-------------------
cases(ruum, [ns, ains, eluta], [ruum, ruumi, ruumi, ruumi, ruumis, ruumist, ruumile, ruumil, ruumilt, ruumiks, ruumini, ruumina, ruumita, ruumiga, ruumid, ruumide, ruume, ruumidesse,ruumides, ruumidest, ruumidele, ruumidel, ruumidelt, ruumideks, ruumideni, ruumidena, ruumideta, ruumidega]).
cases(tuba, [ns, ains, eluta], [tuba, toa, tuba, tuppa, toas, toast, toale, toal, toalt, toaks, toani, toana, toata, toaga, toad, tubade, tubasid, tubadesse, tubades, tubadest, tubadele,tubadel, tubadelt, tubadeks, tubadeni, tubadena, tubadeta, tubadega]).
cases(koridor, [ns, ains, eluta], [koridor, koridori, koridori, koridori, koridoris, koridorist, koridorile, koridoril, koridorilt, koridoriks, koridorini, koridorina, koridorita,	koridoriga, koridorid, koridoride, koridore, koridoridesse, koridorides, koridoridest, koridoridele, koridoridel, koridoridelt, koridorideks, koridorideni,	koridoridena, koridorideta, koridoridega]).
cases(kabinet, [ns, ains, eluta], [kabinet, kabineti, kabinetti, kabinetti, kabinetis, kabinetist, kabinetile, kabinetil, kabinetilt, kabinetiks, kabinetini, kabinetina, kabinetita, kabinetiga,kabinetid, kabinettide, kabinette, kabinettidesse, kabinettides, kabinettidest, kabinettidele, kabinettidel, kabinettidelt, kabinettideks, kabinettideni, kabinettidena, kabinettideta,	kabinettidega]).
cases(kohvituba, [ns, ains, eluta], [kohvituba, kohvitoa, kohvituba, kohvituppa, kohvitoas, kohvitoast, kohvitoale, kohvitoal, kohvitoalt, kohvitoaks, kohvitoani, kohvitoana, kohvitoata, kohvitoaga,kohvitoad, kohvitubade, kohvitube, kohvitubadesse, kohvitubades, kohvitubadest, kohvitubadele, kohvitubadel, kohvitubadelt, kohvitubadeks, kohvitubadeni, kohvitubadena, kohvitubadeta,	kohvitubadega]).

% RUUMI SISUSTUS
cases(laud, [ns, ains, eluta], [laud, laua, lauda, lauasse, lauas, lauast, lauale, laual, laualt, lauaks, lauani, lauana, lauata, lauaga, lauad, laudade, laudu, laudadesse,	laudades, laudadest, laudadele, laudadel, laudadelt, laudadeks, laudadeni, laudadena, laudadeta, laudadega]).
cases(tool, [ns, ains, eluta], [tool, tooli, tooli, tooli, toolis, toolist, toolile, toolil, toolilt, tooliks, toolini, toolina, toolita, tooliga, toolid, toolide, toole, toolidesse,toolides, toolidest, toolidele,toolidel,toolidelt, toolideks,toolideni, toolidena, toolideta, toolidega]).
cases(aken, [ns, ains, eluta], [aken, akna, akent, aknasse, aknas, aknast, aknale, aknal, aknalt, aknaks, aknani, aknana, aknata, aknaga, aknad, aknate, aknaid, akendesse, akendes, aknendest, aknendele, aknendel, aknendelt, aknendeks, aknendeni, aknendena, aknendeta, aknendega]).
cases(uks,  [ns, ains, eluta], [uks, ukse, ust, uksesse, ukses, uksest, uksele, uksel, ukselt, ukseks, ukseni, uksena, ukseta, uksega, uksed, uste, uksi, ustesse, ustes, ustest, ustele, ustel, ustelt, usteks, usteni, ustena, usteta, ustega]).
cases(riiul,[ns, ains, eluta], [riiul, riiuli, riiulit, riiulisse, riiulis, riiulist, riiulile, riiulil, riiulilt, riiuliks, riiulini, riiulina, riiulita, riiuliga, riiulid, riiulite, riiuleid, riiulitesse, riiulites, riiulitest, riiulitele, riiulitel,riiulitelt, riiuliteks, riiuliteni, riiulitena, riiuliteta, riiulitega]).
cases(kapp, [ns, ains, eluta], [kapp, kapi, kappi, kappi, kapis, kapist, kapile, kapil, kapilt, kapiks, kapini, kapina,kapita,kapiga, kapid,kappide, kappe, kappidesse, kappides, kappidest, kappidele, kappidel,kappidelt, kappideks, kappideni, kappidena, kappideta, kappidega]).
cases(kohvimasin, [ns, ains, eluta], [kohvimasin,kohvimasina,kohvimasinat,kohvimasinasse, kohvimasinas, kohvimasinast, kohvimasinale,kohvimasinal,kohvimasinalt, kohvimasinaks, kohvimasinani,kohvimasinana,kohvimasinata,kohvimasinaga, kohvimasinad,kohvimasinate, kohvimasinaid, kohvimasinatesse,kohvimasinates, kohvimasinatest, kohvimasinatele, kohvimasinatel, kohvimasinatelt, kohvimasinateks, kohvimasinateni, kohvimasinatena, kohvimasinateta, kohvimasinatega]).
cases(masin, [ns, ains, eluta], [masin,masina,masinat,masinasse, masinas, masinast,masinale,masinal,masinalt, masinaks,masinani,masinana,masinata,masinaga, masinad,masinate, masinaid, masinatesse,masinates, masinatest, masinatele, masinatel, masinatelt, masinateks, masinateni, masinatena,masinateta, masinatega]).
cases(põrand, [ns, ains, eluta], [põrand,põranda,põrandat,põrandasse, põrandas, põrandast,põrandale,põrandal,põrandalt, põrandaks,põrandani,põrandana,põrandata,põrandaga, põrandad,põrandate, põrandaid, põrandatesse,põrandates, põrandatest, põrandatele, põrandatel, põrandatelt, põrandateks, põrandateni, põrandatena,põrandateta, põrandatega]).


% Arvud
cases(teistkümmend,  [arvsonajärk,  ains, _], [teistkümmend, teistkümne, teistkümmet, teistkümnesse, teistkümnes, teistkümnest, teistkümnele,  teistkümnel,  teistkümnelt,  teistkümneks, teistkümneni, teistkümnena, teistkümneta, teistkümnega, teistkümned, teistkümnete, teistkümneid, teistkümnetesse, teistkümnetes, teistkümnetes, teistkümnetele,  teistkümnetel,  teistkümnetelt, teistkümneteks,  teistkümneteni,  teistkümnetena,  teistkümneteta,  teistkümnetega]).
cases(teist,  [arvsonajärk, ains, _], [teist, teistme, teistmet, teistmesse, teistmes, teistmest, teistmele, teistmel, teistmelt, teistmeks, teistmeni, teistmena, teistmeta, teistmega,
teistmed, teistmete, teistmeid, teistmetesse, teistmetes, teistmetest, teistmetele, teistmetel, teistmetelt, teistmeteks, teistmeteni, teistmetena, teistmeteta,teistmetega]).
cases(kümmend, [arvsonajärk, ains, _], [kümmend, kümne, kümmet, kümnesse, kümnes, kümnest, kümnele, kümnel, kümnelt,  kümneks, kümneni, kümnena, kümneta, kümnega]).
cases(sada,  [arvsonajärk, ains, _], [sada, saja, sadat, sajasse, sajas, sajast, sajale, sajal, sajalt, sajaks, sajani, sajana, sajata, sajaga, sajad, sadade, sadu, sadadesse, sadades, sadadest, sadadele, sadadel, sadadelt, sadadeks, sadadeni, sadadena, sadadeta, sadadega]).
cases(tuhat,  [arvsonajärk1, ains, _], [tuhat, tuhande, tuhandet, tuhandesse, tuhandes, tuhandest, tuhandele, tuhandel, tuhandelt, tuhandeks, tuhandeni, tuhandena, tuhandeta, tuhandega, tuhanded, tuhandete, tuhandeid, tuhandetesse, tuhandetes, tuhandetest,  tuhandetele, tuhandetel, tuhandetelt,  tuhandeteks,  tuhandeteni, tuhandetena, tuhandeteta, tuhandetega]).
cases(miljon,  [arvsonajärk1, ains, _], [miljon, miljoni, miljonit, miljonisse, miljonis, miljonist, miljonile, miljonil, miljonilt, miljoniks, miljonini, miljonina, miljonita, miljoniga, miljonid, miljonite, miljoneid, miljonitesse, miljonites, miljonitest, miljonitele, miljonitel, miljonitelt, miljoniteks, miljoniteni, miljonitena, miljoniteta, miljonitega]).
cases(paar,  [arvsonajärk1, ains, _], [paar, paari, paari, paarisse, paaris, paarist, paarile, paaril, paarilt, paariks, paarini, paarina, paarita, paariga, paarid, paaride, paarisid, paaridesse, paarides, paaridest, paarildele, paaridel, paaridelt, paarideks, paarideni, paaridena, paarideta, paaridega]).
cases(tosin,  [arvsonajärk1, ains, _], [tosin, tosina, tosinat, tosinasse, tosinas, tosinast, tosinale, tosinal, tosinalt, tosinaks, tosinani, tosinana, tosinata, tosinaga, tosinad, tosinate, tosinaid, tosinatesse, tosinates, tosinatest, tosinatele, tosinatel, tosinatelt, tosinateks, tosinateni, tosinatena, tosinateta, tosinatega]).

%ARVSÕNA käänded ---------------------------
cases(üks,  [arvsona, ains, _], [üks, ühe, ühte, ühte, ühes, ühest, ühele, ühel, ühelt, üheks, üheni, ühena, üheta, ühega, ühed, ühtede, ühtesid, ühtedesse, ühtedes, ühtedest, ühtedele, ühtedel, ühtedelt, ühtedeks, ühtedeni, ühtedena, ühtedeta, ühtedega]).
cases(kaks,  [arvsona, ains, _], [kaks, kahe, kahte, kahte, kahes, kahest, kahele, kahel, kahelt, kaheks, kaheni, kahena, kaheta, kahega, kahed, kahtede, kahtesid, kahtedesse, kahtedes, kahtedest, kahtedele, kahtedel, kahtedelt, kahtedeks, kahtedeni, kahtedena, kahtedeta, kahtedega]).
cases(kolm,  [arvsona, ains, _], [kolm, kolme, kolme, kolmesse, kolmes, kolmest, kolmele, kolmel, kolmelt, kolmeks, kolmeni, kolmena, kolmeta, kolmega, kolmed, kolmede, kolmesid, kolmedesse, kolmedes, kolmedest, kolmedele, kolmedel, kolmedelt, kolmedeks, kolmedeni, kolmedena, kolmedeta, kolmedega]).
cases(neli,  [arvsona, ains, _], [neli, nelja, nelja, neljasse, neljas, neljast, neljale, neljal, neljalt, neljaks, neljani, neljana, neljata, neljaga, neljad, neljade, neljasid, neljadesse, neljades, neljadest, neljadele, neljadel, neljadelt, neljadeks, neljadeni, neljadena, neljadeta, neljadega]).
cases(viis,  [arvsona, ains, _], [viis, viie, viit, viiesse, viies, viiest, viiele, viiel, viielt, viieks, viieni, viiena, viieta, viiega, viied, viitede, viitesid, viitedesse, viitedes, viitedest, viitedele, viitedel, viitedelt, viitedeks, viitedeni, viitedena, viitedeta, viitedega]).
cases(kuus,  [arvsona, ains, _], [kuus, kuue, kuut, kuuesse, kuues, kuuest, kuuele, kuuel, kuuelt, kuueks, kuueni, kuuena, kuueta, kuuega, kuued, kuutede, kuutesid, kuutedesse, kuutedes, kuutedest, kuutedele, kuutedel, kuutedelt, kuutedeks, kuutedeni, kuutedena, kuutedeta, kuutedega]).
cases(seitse,  [arvsona, ains, _], [seitse, seitsme, seitset, seitsmesse, seitsmes, seitsmest, seitsmele, seitsmel, seitsmelt, seitsmeks, seitsmeni, seitsmena, seitsmeta, seitsmega, seitsmed, seitsmete, seitsmeid, seitsmetesse, seitsmetes, seitsmetest, seitsmetele, seitsmetel, seitsmetelt, seitsmeteks, seitsmeteni, seitsmetena, seitsmeteta, seitsmetega]).
cases(kahkesa,  [arvsona, ains, _], [kaheksa, kaheksa, kaheksat, kaheksasse, kaheksas, kaheksast, kaheksale, kaheksal, kaheksalt, kaheksaks, kaheksani, kaheksana, kaheksata, kaheksaga, kaheksad, kaheksate, kaheksaid, kaheksatesse, kaheksates, kaheksatest, kaheksatele, kaheksatel, kaheksatelt, kaheksateks, kaheksateni, kaheksatena, kaheksateta, kaheksatega]).
cases(üheksa,  [arvsona, ains, _],  [üheksa, üheksa, üheksat, üheksasse, üheksas, üheksast, üheksale, üheksal, üheksalt, üheksaks, üheksani, üheksana, üheksata, üheksaga, üheksad, üheksate, üheksaid, üheksatesse, üheksates, üheksatest, üheksatele, üheksatel, üheksatelt, üheksateks,  üheksateni,  üheksatena, üheksateta, üheksatega]).
cases(kümme,  [arvsona, ains, _], [kümme, kümne, kümmet, kümnesse, kümnes, kümnest, kümnele, kümnel, kümnelt,  kümneks, kümneni, kümnena, kümneta, kümnega, kümned,  kümnete,  kümneid, kümnetesse, kümnetes, kümnetest, kümnetele, kümnetel, kümnetelt, kümneteks, kümneteni, kümnetena, kümneteta, kümnetega]).
%

cases(kohv, [ns, ains, eluta], [kohv, kohvi, kohvi, kohvisse, kohvis, kohvist, kohvile, kohvil, kohvilt, kohviks, kohvini, kohvina, kohvita, kohviga, kohvid, kohvide, kohvisid, kohvidesse,kohvides, kohvidest, kohvidele, kohvidel, kohvidelt, kohvideks, kohvideni, kohvidena, kohvideta, kohvidega]).

