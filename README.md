# washer_programmer
source code from an automatic washing machine programmer/controller

UNTESTED CODE! COMPILES WITHOUT ERRORS.

Musisz zmienić nazwę pliku:
main_POLSKIE_KOMENTARZE.cpp   ---> main.cpp

OR/LUB:

You must rename:
main_ENGLISH_COMMENTS.cpp     ---> main.cpp

Do odtworzenia kodu z pliku hex użyłem poniższego programu oraz Gemini Argon Extended.
https://github.com/nationalsecurityagency/ghidra

To reverse engineering of code I used belowe program and Gemini Argon Extended.
https://github.com/nationalsecurityagency/ghidra


/*
 * ============================================================================
 * LEGAL DISCLAIMER AND SAFETY WARNING
 * ============================================================================
 * 
 * 1. PURPOSE AND ORIGIN OF THE CODE:
 * This source code is an educational reconstruction of the firmware extracted 
 * from an ATmega644P microcontroller used in an automatic washing machine 
 * main control board (programmer). The reverse engineering and decompilation 
 * process was performed using XXXX software strictly for educational, research, 
 * repair, and hardware interoperability purposes.
 * 
 * 2. NO AFFILIATION:
 * This project is an independent effort and is not affiliated with, authorized, 
 * sponsored, or endorsed by the original appliance manufacturer. Any trademarks 
 * or product names belong to their respective owners.
 * 
 * 3. LETHAL VOLTAGE & MECHANICAL HAZARD WARNING (230V AC MAINS):
 * DANGER! Washing machine control boards typically use NON-ISOLATED power 
 * supplies (capacitive or buck droppers). The digital ground (GND) and 
 * microcontroller pins are directly tied to LIVE 230V AC MAINS potential!
 * - Never connect an ISP programmer, USB-UART adapter, oscilloscope, or PC 
 *   to the board while it is plugged into AC mains without proper galvanic 
 *   isolation. Doing so poses an immediate risk of fatal electrocution and 
 *   equipment destruction.
 * - Faulty phase-angle control of the universal series-wound motor (e.g., 
 *   PID failure or lost tachogenerator feedback) can cause a catastrophic 
 *   motor runaway (>15,000 RPM), resulting in mechanical disintegration of 
 *   the drum, fire, or severe personal injury.
 * - Improper control of the high-power heating element (~2000 W) without water 
 *   level verification or NTC thermal protection creates a severe fire hazard.
 * 
 * 4. LIMITATION OF LIABILITY (USE AT YOUR OWN RISK):
 * This software is provided "AS IS", without warranty of any kind, express or 
 * implied. Under no circumstances shall the author be held liable for any 
 * direct, indirect, or consequential damages (including appliance destruction, 
 * flooding, fire, personal injury, or death) arising from the use, modification, 
 * or flashing of this code onto physical hardware. You use this code entirely 
 * at your own risk and responsibility.
 * ============================================================================
 */





 /*
 * ============================================================================
 * INFORMACJA PRAWNA I OSTRZEŻENIE DOTYCZĄCE BEZPIECZEŃSTWA (DISCLAIMER)
 * ============================================================================
 * 
 * 1. CEL PROJEKTU I POCHODZENIE KODU:
 * Niniejszy kod źródłowy stanowi edukacyjną rekonstrukcję oprogramowania 
 * układowego (firmware) pochodzącego z mikrokontrolera ATmega644P, pracującego 
 * w module wykonawczym (programatorze) pralki automatycznej. Proces inżynierii 
 * wstecznej (Reverse Engineering) oraz dekompilacji wsadu został przeprowadzony 
 * przy użyciu oprogramowania XXXX wyłącznie w celach badawczych, edukacyjnych, 
 * archiwizacyjnych oraz na potrzeby naprawy i analizy interoperacyjności sprzętu.
 * 
 * 2. BRAK POWIĄZAŃ Z PRODUCENTEM:
 * Projekt ten jest całkowicie niezależny i nie jest w żaden sposób powiązany, 
 * autoryzowany, wspierany ani zatwierdzony przez oryginalnego producenta 
 * urządzenia. Wszystkie ewentualne znaki towarowe należą do ich prawowitych 
 * właścicieli.
 * 
 * 3. OSTRZEŻENIE O ZAGROŻENIU ŻYCIA I ZDROWIA (WYSOKIE NAPIĘCIE 230V AC):
 * UWAGA! Płytki drukowane (PCB) sterowników pralek automatycznych zazwyczaj 
 * NIE POSIADAJĄ SEPARACJI GALWANICZNEJ od sieci energetycznej 230V AC 
 * (masa układu cyfrowego GND oraz wyprowadzenia mikrokontrolera mogą znajdować 
 * się bezpośrednio pod potencjałem fazy sieci zasilającej!).
 * - Podłączanie programatora ISP, analizatora stanów logicznych, konwertera 
 *   UART-USB lub komputera PC do płytki podłączonej do sieci 230V AC bez 
 *   odpowiedniej izolacji galwanicznej grozi śmiertelnym porażeniem prądem 
 *   oraz zniszczeniem sprzętu pomiarowego.
 * - Błędne wysterowanie triaka silnika komutatorowego (brak kontroli PID lub 
 *   uszkodzenie pętli sprzężenia zwrotnego z tachogeneratora) prowadzi do 
 *   niekontrolowanego rozbiegania silnika (kilkanaście tysięcy obr./min), co 
 *   grozi rozerwaniem mechanicznym bębna, pożarem lub ciężkim uszkodzeniem ciała.
 * - Nieprawidłowe wysterowanie przekaźnika grzałki (np. ~2000 W) przy braku 
 *   wody w bębnie lub braku nadzoru czujnika NTC grozi bezpośrednim pożarem.
 * 
 * 4. WYŁĄCZENIE ODPOWIEDZIALNOŚCI (UŻYTEK NA WŁASNE RYZYKO):
 * Kod jest udostępniany w postaci „TAK JAK JEST” (AS IS), bez jakichkolwiek 
 * gwarancji – wyraźnych ani dorozumianych. Autor opracowania nie ponosi żadnej 
 * odpowiedzialności za jakiekolwiek szkody materialne (zniszczenie urządzenia, 
 * zalanie, pożar), uszczerbek na zdrowiu ani utratę życia wynikające z użycia, 
 * modyfikacji lub wgrania niniejszego kodu do rzeczywistego urządzenia. 
 * Wszelkie działania wykonujesz wyłącznie na własny użytek i na własną 
 * odpowiedzialność.
 * ============================================================================
 */