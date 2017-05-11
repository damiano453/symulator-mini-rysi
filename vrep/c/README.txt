Program kompilujemy i uruchamiamy jak zawsze, z zestawem trzech parametrów
ID robota (numer go identyfikuj¹cy)	Xpozycja punktu narodzenia	Ypozycja punktu narodzenia
Czyli dla przyk³adu:
costam.exe 0 1.000 2.000
Tworzy robota o ID 0 w punkcie 1.000 2.000 mapy

Tworzy równie¿ plik Output0.txt, gdzie umieszczany jest stan robota
Komendy wydajemy w pliku Commands0.txt
Numerek to dok³adnie ID robota

Ad. linux
kompilujemy komenda:
gcc -pthread main.c extApiPlatform.c extApi.c -o number -lm

-lm -> potrzebne do math.h
-pthread -> do thread mode
