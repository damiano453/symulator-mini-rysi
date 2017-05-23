### Symulator robotów miniRYŚ

---
#### Struktura folderów:  
* **`build`** - przekompilowana aplikacja
  * **`bin`** - moduły aplikacji
  * **`comm_robots`** - bufory komunikacji robot-robot
  * **`comm_sym`** - bufory komunikacji robot-symulator
  * **`vrep`** - pliki vrepa
  * `master.exe`
* **`dokumentacja`**  
  * **`instrukcja obslugi`**
  * **`materialy`**
  * **`sprawozdania`**
* **`program`** - miejsce na wklejenie main.c z kodem funkcjonalnym robota pisanym przez programistę miniRysi
* **`visualstudio`** - projekty VS
* **`zrodlowe`** - kod modułów aplikacji
  * **`emulator`** - wyższa warstwa
  * **`komunikator`** - serwer realizujący komunikację robot-robot
  * **`master`** - główny program, który uruchamia poszczególne moduły
  * **`symulator`** - fizyka robota + komunikacja z vrepem
  * **`wizualizator`** - pliki vrepa (zapewne identyczne z build/vrep)

---
#### Projekty:
**komunikator**  
  **`in`** `zrodlowe/komunikator`  
  **`out`** `build/bin`  
  
**master**  
  **`in`** `zrodlowe/master`  
  **`out`** `build`
  
**program_emulator**  
  **`in`** `zrodlowe/emulator`  
  **`in`** `program/main.c`  
  **`in`** `zrodlowe/komunikator/functions.c`  
  **`out`** `build/bin`
  
**symulator**  
  **`in`** `zrodlowe/symulator`  
  **`out`** `build/bin`

---
#### Komendy mastera:
**`set #`** - zainicjuj # pierwszych robotów z listy  
**`vel #`** - ustaw prędkość na #  
**`start`** - uruchom symulację  
**`pause`** - wstrzymaj symulację  
**`stop`** - zatrzymaj i zresetuj symulację

`robots_placement.txt` - położenia robotów w momencie inicjalizacji  
`map.txt` - rozmieszczenie bloków ruchomych i nieruchomych, granice planszy