# Instrukcja Operatora - System Sterowania Robotem Fabrycznym

## Spis Treści

1. [Podstawowe Funkcjonowanie](#podstawowe-funkcjonowanie)
2. [Tryby Pracy](#tryby-pracy)
3. [Tworzenie i Edycja Tras](#tworzenie-i-edycja-tras)
4. [System Planowania i Wykonywania Akcji](#system-planowania-i-wykonywania-akcji)
5. [Stacje Dokujące](#stacje-dokujące)
6. [Odczyt Statusów i Kontrolek](#odczyt-statusów-i-kontrolek)
7. [Rozwiązywanie Problemów](#rozwiązywanie-problemów)

---

## Podstawowe Funkcjonowanie

### Uruchomienie Aplikacji
System uruchamia się w trybie pełnoekranowym optimalizowanym dla rozdzielczości 1920x1080. Po uruchomieniu operator widzi główny interfejs z mapą fabryki i narzędziami sterowania.

### Stany Systemu
System operuje w trzech podstawowych stanach:

**1. Stan Rozłączony (Disconnected)**
- Robot nie jest połączony z systemem
- Wyświetla się ekran oczekiwania na połączenie
- Niedostępne funkcje nawigacji

**2. Stan Konfiguracji (Configuring)**
- Robot jest połączony, ale w trybie konfiguracyjnym
- Możliwe jest ręczne sterowanie robotem
- Dostępne ustawianie pozycji początkowej robota
- Można wybierać i ładować mapy

**3. Stan Aktywny (Active)**
- Robot jest gotowy do wykonywania zadań
- Dostępne wszystkie funkcje nawigacji i planowania
- System może wykonywać zaprogramowane plany działania

---

## Tryby Pracy

### Tryb Podstawowy
W trybie podstawowym operator może:
- Nawigować robot pojedynczymi trasami
- Ręcznie kontrolować dokowanie/oddokowanie
- Monitorować status robota w czasie rzeczywistym

### Tryb Planowania (Plan System)
Zaawansowany tryb umożliwiający:
- Tworzenie złożonych planów składających się z sekwencji akcji
- Automatyczne wykonywanie ciągów operacji
- Zarządzanie akcjami oczekiwania na sygnały zewnętrzne

---

## Tworzenie i Edycja Tras

### Tworzenie Nowej Trasy

1. **Przejście do Trybu Planowania**
   - Naciśnij przycisk "Planowanie" w interfejsie głównym
   - System przejdzie w tryb rysowania tras

2. **Dodawanie Punktów Trasy**
   - Kliknij lewym przyciskiem myszy na mapie w miejscu pierwszego punktu
   - Każdy kolejny klik dodaje następny punkt trasy
   - Punkty są automatycznie połączone krzywymi Bézier dla płynnej nawigacji

3. **Edycja Punktów Kontrolnych**
   - Każdy punkt trasy ma punkty kontrolne do dostosowania krzywizny
   - Przeciągaj punkty kontrolne aby dostosować kształt trasy
   - System automatycznie generuje płynne przejścia między punktami

4. **Zapisywanie Trasy**
   - Wprowadź nazwę trasy w polu tekstowym
   - Naciśnij "Zapisz Trasę"
   - Trasy są zapisywane per mapa w pliku ~/.robotroutes/routes.json

### Parametry Tras
- **route_name**: Nazwa trasy
- **reverse**: Określa czy trasa ma być wykonana w odwrotnym kierunku
- Każda trasa składa się z węzłów (nodes) zawierających:
  - Pozycję (x, y)
  - Orientację robota
  - Punkty kontrolne krzywych Bézier

### Zarządzanie Trasami
- Trasy są przechowywane w katalogu `~/.robotroutes/`
- Każda mapa ma własny zestaw tras
- System automatycznie ładuje trasy przy zmianie mapy

---

## System Planowania i Wykonywania Akcji

### Typy Akcji

#### 1. Akcja Trasy (ROUTE)
**Parametry:**
- `route_name`: Nazwa trasy do wykonania
- `reverse`: true/false - czy wykonać trasę w odwrotnym kierunku

**Działanie:**
- Robot nawiguje po zaprogramowanej trasie
- Możliwość wykonania trasy do przodu lub do tyłu
- Akcja kończy się po dotarciu do punktu docelowego

#### 2. Akcja Dokowania (DOCK)
**Parametry:**
- `dock_name`: (opcjonalny) Nazwa konkretnej stacji dokującej

**Działanie:**
- Robot dokuje do określonej stacji lub najbliższej dostępnej
- System monitoruje status dokowania
- Akcja kończy się po pomyślnym zadokowaniu

#### 3. Akcja Oddokowania (UNDOCK)
**Działanie:**
- Robot oddokuje od aktualnej stacji
- Przechodzi w tryb gotowości do kolejnych akcji
- Akcja kończy się po pomyślnym oddokowaniu

#### 4. Akcja Oczekiwania na Sygnał (WAIT_FOR_SIGNAL)
**Parametry:**
- `signal_name`: Nazwa sygnału (opis oczekiwanej akcji)

**Działanie:**
- Robot zatrzymuje się i czeka na sygnał zewnętrzny
- Wyświetla się przycisk sygnału na interfejsie
- Sygnał może być wysłany przez:
  - Naciśnięcie przycisku na interfejsie
  - Fizyczny przycisk na robocie
  - Sygnał CAN (ID 0x69) z zewnętrznego systemu

### Tworzenie Planów

1. **Dostęp do Edytora Planów**
   - W trybie aktywnym przejdź do zakładki planowania
   - Wybierz "Nowy Plan" lub edytuj istniejący

2. **Dodawanie Akcji**
   - Wybierz typ akcji z listy rozwijanej
   - Skonfiguruj parametry specyficzne dla akcji
   - Dodaj akcję do sekwencji

3. **Kolejność Wykonania**
   - Akcje wykonują się w kolejności dodania
   - Możliwość zmiany kolejności przez przeciąganie
   - Po wykonaniu wszystkich akcji plan się automatycznie restartuje

4. **Zapisywanie Planów**
   - Plany są zapisywane w ~/.robotroutes/plans.json
   - System przechowuje aktualny indeks akcji
   - Plany można wznowić od miejsca przerwania

### Wykonywanie Planów

**Start Planu:**
- Wybierz plan z listy dostępnych
- Naciśnij "Uruchom Plan"
- System rozpocznie od pierwszej akcji lub od miejsca przerwania

**Kontrola Wykonania:**
- "Zatrzymaj Plan" - przerywa wykonanie z możliwością wznowienia
- "Stop Navigation" - zatrzymuje tylko aktualną nawigację
- "Wykonaj Akcję" - wykonuje pojedynczą akcję z planu

**Automatyczne Przejścia:**
- Po zakończeniu akcji system automatycznie przechodzi do kolejnej
- W przypadku błędu wykonanie się zatrzymuje z komunikatem o błędzie
- Plany wykonują się w pętli - po ostatniej akcji rozpoczynają od pierwszej

---

## Stacje Dokujące

### Konfiguracja Stacji Dokujących

1. **Dodawanie Stacji**
   - W trybie konfiguracji kliknij prawym przyciskiem na mapie
   - Wybierz "Dodaj Stację Dokującą"
   - Wprowadź nazwę stacji

2. **Pozycjonowanie Stacji**
   - Każda stacja ma określoną pozycję (x, y)
   - Orientację robota po zadokowaniu
   - Stacje są zapisywane per mapa w ~/.robotroutes/docks.json

### Użytkowanie Stacji

**Dokowanie Manualne:**
- Przycisk "Dokuj Robot" - dokuje do najbliższej stacji
- Możliwość wyboru konkretnej stacji z listy

**Dokowanie w Planach:**
- Akcje DOCK mogą specyfikować konkretną stację
- Bez parametru dock_name system wybiera najbliższą stację

**Monitorowanie Dokowania:**
- Status dokowania wyświetlany w czasie rzeczywistym
- Możliwe statusy: "Dokuje", "Zadokowany", "Błąd Dokowania"

---

## Odczyt Statusów i Kontrolek

### Panel Statusu Robota

**Status Główny:**
- "Bezczynny" - robot gotowy do akcji
- "Nawiguje" - robot przemieszcza się po trasie
- "Dokuje"/"Zadokowany" - proces dokowania
- "Oczekiwanie na sygnał" - robot czeka na zewnętrzną akcję
- "Failed"/"Error" - wystąpił błąd wymagający interwencji

**Status Baterii:**
- Procent naładowania baterii
- Tekst statusu (np. "Charging", "Discharging")
- Kolorowe wskaźniki ostrzeżeń o niskim poziomie baterii

### Kontrolki Nawigacji

**Ręczne Sterowanie:**
- Klawisze WASD do sterowania kierunkowego
- Przycisk "Start Manual Control" / "Stop Manual Control"
- Regulacja prędkości w zależności od trybu

**Kontrola Pozycji:**
- Kliknięcie na mapie ustawia pozycję początkową robota
- Automatyczne zapisywanie pozycji co 10 sekund
- Ładowanie ostatniej pozycji przy zmianie map

### Mapa i Wizualizacja

**Elementy Mapy:**
- Mapa fabryki w skali szarości
- Pozycja robota jako niebieska ikona ze strzałką orientacji
- Trasy wyświetlane jako krzywe Bézier
- Stacje dokujące jako zielone ikony

**Interakcje:**
- Zoom i przewijanie mapy
- Kliknięcie ustawia pozycję robota
- Rysowanie tras w trybie planowania

### Sygnały i Ostrzeżenia

**System Sygnałów CAN:**
- Monitorowanie sygnałów na magistrali CAN
- Sygnał ID 0x69 obsługiwany jako sygnał zewnętrzny
- Wyświetlanie statusu oczekiwania na sygnał

**Przycisk Sygnału:**
- Pojawia się podczas akcji WAIT_FOR_SIGNAL
- Opis oczekiwanego sygnału
- Możliwość ręcznego potwierdzenia przez operatora

**Ostrzeżenia:**
- Komunikaty o błędach nawigacji
- Ostrzeżenia o niskim poziomie baterii
- Status połączenia z robotem

---

## Rozwiązywanie Problemów

### Problemy z Połączeniem

**Robot nie łączy się:**
1. Sprawdź status połączenia sieciowego
2. Upewnij się, że wszystkie usługi ROS2 są uruchomione
3. Sprawdź dostępność usługi `/trigger_service`

**Utrata połączenia w trakcie pracy:**
1. System automatycznie przejdzie w stan rozłączony
2. Robot zatrzyma aktualną akcję
3. Po przywróceniu połączenia można wznowić pracę

### Problemy z Nawigacją

**Robot nie wykonuje trasy:**
1. Sprawdź czy mapa jest prawidłowo załadowana
2. Upewnij się, że trasa istnieje i ma prawidłową strukturę
3. Sprawdź pozycję początkową robota na mapie

**Nieprawidłowa ścieżka:**
1. Sprawdź punkty kontrolne krzywych Bézier
2. Upewnij się, że trasa nie przecina przeszkód
3. Przetestuj trasę w trybie pojedynczej akcji

### Problemy z Planami

**Plan się nie wykonuje:**
1. Sprawdź czy wszystkie trasy w planie istnieją
2. Upewnij się, że stacje dokujące są prawidłowo zdefiniowane
3. Sprawdź parametry każdej akcji w planie

**Plan zatrzymuje się na akcji oczekiwania:**
1. Sprawdź czy wyświetla się przycisk sygnału
2. Upewnij się, że sygnały CAN są prawidłowo konfigurowane
3. Sprawdź logi systemu pod kątem błędów komunikacji

### Problemy z Mapami

**Mapa się nie ładuje:**
1. Sprawdź czy pliki .pgm i .yaml istnieją w ~/.robotroutes/maps/
2. Sprawdź poprawność struktury pliku YAML
3. Upewnij się, że są odpowiednie uprawnienia do plików

**Robot jest w złej pozycji:**
1. Ręcznie ustaw pozycję kliknięciem na mapie
2. Sprawdź czy plik pozycji nie jest uszkodzony
3. Użyj funkcji "Set Position" w trybie konfiguracji

### Systemy Pomocnicze

**Logi Systemowe:**
- Sprawdź terminal pod kątem komunikatów błędów
- Logi ROS2 mogą zawierać dodatkowe informacje diagnostyczne
- Komunikaty debugowania CAN są widoczne w konsoli

**Pliki Konfiguracyjne:**
- ~/.robotroutes/routes.json - trasy
- ~/.robotroutes/plans.json - plany
- ~/.robotroutes/docks.json - stacje dokujące
- ~/.robotroutes/positions/ - zapisane pozycje robota

**Kopie Zapasowe:**
Regularne wykonywanie kopii zapasowych katalogu ~/.robotroutes/ zapewnia możliwość odtworzenia konfiguracji w przypadku problemów.

---

*Instrukcja została przygotowana dla operatorów systemu sterowania robotem fabrycznym w wersji roboto_viz.*