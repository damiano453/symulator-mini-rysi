�eby uruchomi� ca�y most nale�y:
1. Zainstalowa� vrepa
2. Chyba zbudowa� RemoteAPI do vrepa. Zasadniczo wszystkie pliki, kt�re je tworz� siedz� w folderku vrepPort. Wydaje mi si�, �e ten zestaw plik�w kt�ry tam jest wystarcza do dzia�ania, ale na pewno gdy system jest 32bit, trzeba b�dzie wzi�� sobie now� dllk�, kt�ra jest gdzie� w folderku po instalacji vrepa
3. Trzeba skonfigurowa� pierwsz� sekcj� w pliku test.m, czyli w sumie ustawi� �cie�k� do pliku w kt�rym zapisany jest obiekt typu robot: Robot.ttm. �cie�k� podajemy wzgl�dem katalogu g��wnego vrepa a nie skryptu, ewentualnie bezwzgl�dnie, ale z polskimi znakami i d�ugimi �cie�kami mia�em problem.

I dalej:
4. Uruchamiamy vrepa 
5. W matlabie klikamy f5, �eby program zacz�� si� wykonywa�

Komendy do vrepa przekazujemy przez plik: Commands.txt
W pierwszej linii zapisujemy: Pr�dko�� 1 ko�a (ci�ko powiedzie� czy to prawe czy lewe)(TAB)Pr�dko�� 2 ko�a(TAB)K�t pochylenia korpusu robota (-90 +90) stopni
W drugiej linii: 0, je�li chcemy �eby symulacja si� wykonywa�a; 1, �eby si� nie wykonywa�a

W pliku Output.txt na bie��co mo�na widzie�:
Globalna pozycja robotaX; Globalna pozycja robotaY; Orientacja wzgl�dem XY (a tu niestety w radianach)
Odczyty (chyba w metrach, ale te� nie pami�tam :D ) z trzech czujnik�w odleg�o�ci
I informacj�, czy czujnik co� zmierzy�, czy mierzy powietrze

�eby �adnie wszystko pozamyka�, nale�y wpisa� 1 do drugiej linii Commands, wtedy zatrzymujemy symulator i kombinacj� klawiszy CTRL+C zatrzymujemy program w matlabie 
