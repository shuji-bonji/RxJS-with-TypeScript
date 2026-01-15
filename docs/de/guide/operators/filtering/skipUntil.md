---
description: Der skipUntil-Operator überspringt alle Werte des ursprünglichen Observable, bis ein anderes Observable einen Wert ausgibt, danach werden Werte normal ausgegeben. Praktisch für zeitbasierte verzögerte Starts oder Verarbeitung nach bestimmten Ereignissen.
---

# skipUntil - Überspringen Bis Trigger

Der `skipUntil`-Operator **überspringt alle Werte vom ursprünglichen Observable, bis das angegebene Observable (Benachrichtigungs-Trigger) den ersten Wert ausgibt**. Nach Auslösung des Benachrichtigungs-Triggers werden Werte normal ausgegeben.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // Wert alle 0,5 Sekunden ausgeben
const notifier$ = timer(2000); // Wert nach 2 Sekunden ausgeben

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Ausgabe: 4, 5, 6, 7, 8, ...
// (Werte 0, 1, 2, 3 der ersten 2 Sekunden werden übersprungen)
```

**Ablauf**:
1. `source$` gibt 0, 1, 2, 3 aus → Alle übersprungen
2. Nach 2 Sekunden gibt `notifier$` Wert aus
3. Danach werden Werte von `source$` (4, 5, 6, ...) normal ausgegeben

[🌐 RxJS Offizielle Dokumentation - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)


## 🆚 Vergleich mit takeUntil

`skipUntil` und `takeUntil` verhalten sich gegensätzlich.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // Wert alle 0,5 Sekunden ausgeben
const notifier$ = timer(2000); // Wert nach 2 Sekunden ausgeben

// takeUntil: Werte bis zur Benachrichtigung abrufen
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3 (stoppt nach 2 Sekunden)

// skipUntil: Werte bis zur Benachrichtigung überspringen
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Ausgabe: 4, 5, 6, 7, ... (startet nach 2 Sekunden)
```

| Operator | Verhalten | Abschlusszeitpunkt |
|---|---|---|
| `takeUntil(notifier$)` | Werte bis zur Benachrichtigung **abrufen** | Automatischer Abschluss bei Benachrichtigung |
| `skipUntil(notifier$)` | Werte bis zur Benachrichtigung **überspringen** | Bei Abschluss des ursprünglichen Streams |


## 💡 Typische Anwendungsmuster

[Codebeispiele für: Verarbeitung nach Benutzerauthentifizierung, Ereignisverarbeitung nach Initialladen, Timerbasierte verzögerte Starts]

## 🎓 Zusammenfassung

### Wann skipUntil verwenden
- ✅ Wenn Verarbeitung nach bestimmtem Ereignis starten soll
- ✅ Wenn Benutzeroperationen nach Initialisierung aktiviert werden sollen
- ✅ Wenn zeitbasierter verzögerter Start benötigt wird
- ✅ Wenn Datenverarbeitung nach Authentifizierung starten soll

### Kombination mit takeUntil
- ✅ Wenn Werte nur in bestimmtem Zeitraum abgerufen werden sollen (skipUntil + takeUntil)

### Hinweise
- ⚠️ Nur erste Auslösung des Benachrichtigungs-Observable ist wirksam
- ⚠️ Wenn Benachrichtigungs-Observable nicht ausgelöst wird, werden alle Werte weiter übersprungen
- ⚠️ Subscription wird aufrechterhalten bis ursprünglicher Stream abschließt


## 🚀 Nächste Schritte

- **[skip](./skip)** - Methode zum Überspringen der ersten N Werte lernen
- **[take](./take)** - Methode zum Abrufen der ersten N Werte lernen
- **[takeUntil](../utility/takeUntil)** - Methode zum Abrufen von Werten bis zur Auslösung eines anderen Observable lernen
- **[filter](./filter)** - Methode zum Filtern basierend auf Bedingungen lernen
- **[Praktische Beispiele für Filteroperatoren](./practical-use-cases)** - Reale Anwendungsfälle lernen
