¯eby uruchomiæ ca³y most nale¿y:
1. Zainstalowaæ vrepa
2. Chyba zbudowaæ RemoteAPI do vrepa. Zasadniczo wszystkie pliki, które je tworz¹ siedz¹ w folderku vrepPort. Wydaje mi siê, ¿e ten zestaw plików który tam jest wystarcza do dzia³ania, ale na pewno gdy system jest 32bit, trzeba bêdzie wzi¹æ sobie now¹ dllkê, która jest gdzieœ w folderku po instalacji vrepa
3. Trzeba skonfigurowaæ pierwsz¹ sekcjê w pliku test.m, czyli w sumie ustawiæ œcie¿kê do pliku w którym zapisany jest obiekt typu robot: Robot.ttm. Œcie¿kê podajemy wzglêdem katalogu g³ównego vrepa a nie skryptu, ewentualnie bezwzglêdnie, ale z polskimi znakami i d³ugimi œcie¿kami mia³em problem.

I dalej:
4. Uruchamiamy vrepa 
5. W matlabie klikamy f5, ¿eby program zacz¹³ siê wykonywaæ

Komendy do vrepa przekazujemy przez plik: Commands.txt
W pierwszej linii zapisujemy: Prêdkoœæ 1 ko³a (ciê¿ko powiedzieæ czy to prawe czy lewe)(TAB)Prêdkoœæ 2 ko³a(TAB)K¹t pochylenia korpusu robota (-90 +90) stopni
W drugiej linii: 0, jeœli chcemy ¿eby symulacja siê wykonywa³a; 1, ¿eby siê nie wykonywa³a

W pliku Output.txt na bie¿¹co mo¿na widzieæ:
Globalna pozycja robotaX; Globalna pozycja robotaY; Orientacja wzglêdem XY (a tu niestety w radianach)
Odczyty (chyba w metrach, ale te¿ nie pamiêtam :D ) z trzech czujników odleg³oœci
I informacjê, czy czujnik coœ zmierzy³, czy mierzy powietrze

¯eby ³adnie wszystko pozamykaæ, nale¿y wpisaæ 1 do drugiej linii Commands, wtedy zatrzymujemy symulator i kombinacj¹ klawiszy CTRL+C zatrzymujemy program w matlabie 
