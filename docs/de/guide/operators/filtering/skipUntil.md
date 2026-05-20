---
description: "Der skipUntil-Operator überspringt alle Werte des ursprünglichen Observable, bis ein anderes Observable einen Wert ausgibt; danach wird der Wert wie üblich ausgegeben. Dies ist nützlich für zeitlich verzögerte Starts oder nachdem ein bestimmtes Ereignis eingetreten ist."
---

# skipUntil - bis zur Zündung überspringen

Der Operator "skipUntil" **überspringt alle Werte des ursprünglichen Observable**, bis der erste Wert vom angegebenen Observable ausgegeben wird (Notification Trigger). Nach dem Zeitpunkt, an dem der Notification Trigger ausgegeben wird, werden die Werte wie gewohnt ausgegeben.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // 0.5Wert jede Sekunde ausgeben
const notifier$ = timer(2000); // 2Wert nach jeder Sekunde ausgeben

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Ausgabe: 4, 5, 6, 7, 8, ...
// (erster2zweiter Wert 0, 1, 2, 3 werden übersprungen)
```

**Ablauf der Operation**:.
1. `source$` gibt 0, 1, 2, 3 aus → alles überspringen
2. 2 Sekunden später gibt `notifier$` einen Wert aus
3. die folgenden `Quelle$`-Werte (4, 5, 6, ...) werden wie üblich ausgegeben.

[🌐 Offizielle RxJS Dokumentation - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)

## 🆚 Gegensatz zu takeUntil

skipUntil" und "takeUntil" haben ein gegensätzliches Verhalten.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // 0.5Wert jede Sekunde ausgeben
const notifier$ = timer(2000); // 2Wert nach jeder Sekunde ausgeben

// takeUntil: Abrufen des Wertes bis zur Benachrichtigung
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3(Stoppt nach2(hält nach 1,5 Sekunden an)

// skipUntil: Überspringen von Werten bis zur Benachrichtigung
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Ausgabe: 4, 5, 6, 7, ...(Stoppt nach2(Beginnt nach 1,5 Sekunden)
```

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // 0.5Wert jede Sekunde ausgeben
const notifier$ = timer(2000); // 2Wert nach jeder Sekunde ausgeben

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Ausgabe: 4, 5, 6, 7, 8, ...
// (erster2zweiter Wert 0, 1, 2, 3 werden übersprungen)
```

## 💡 Typisches Nutzungsmuster

1. **Start der Datenverarbeitung nach Benutzerauthentifizierung**.


```ts
   import { interval, Subject } from 'rxjs';
   import { skipUntil } from 'rxjs';

   const authenticated$ = new Subject<void>();
   const dataStream$ = interval(1000);

   // Daten überspringen, bis die Authentifizierung abgeschlossen ist
   dataStream$.pipe(
     skipUntil(authenticated$)
   ).subscribe(data => {
     console.log(`Verarbeitung der Daten: ${data}`);
   });

   // 3Authentifizierung abgeschlossen nach 2 Sekunden
   setTimeout(() => {
     console.log('Authentifizierung abgeschlossen！');
     authenticated$.next();
   }, 3000);
   // 3(Beginn nach einer Sekunde) "Datenverarbeitung: 3Datenverarbeitung: 4', 'Datenverarbeitung'...und Ausgabe
   ```

2. **Ereignisverarbeitung beginnt nach Abschluss des ersten Ladens**
   ```ts
   import { fromEvent, BehaviorSubject } from 'rxjs';
   import { filter, skipUntil } from 'rxjs';

   const appReady$ = new BehaviorSubject<boolean>(false);
   const button = document.createElement('button');
   button.textContent = 'Klicken.';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');

   // Klicks ignorieren, bis die App fertig ist
   clicks$.pipe(
     skipUntil(appReady$.pipe(filter(ready => ready)))
   ).subscribe(() => {
     console.log('Klick verarbeitet');
   });

   // 2App bereit in Sekunden
   setTimeout(() => {
     console.log('App ist bereit');
     appReady$.next(true);
   }, 2000);
   ```

3. **Timer-basierte Verzögerung gestartet**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { skipUntil, scan } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Zählung';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');
   const startTime$ = timer(3000); // 3Sekunden später

   // 3Klicks werden erst nach Ablauf der Sekunden gezählt
   clicks$.pipe(
     skipUntil(startTime$),
     scan(count => count + 1, 0)
   ).subscribe(count => {
     console.log(`Zählung: ${count}`);
   });

   console.log('3Zählung beginnt nach Sekunden...');
   ```

## 🧠 Praktisches Code-Beispiel (Spiel-Countdown)

Dies ist ein Beispiel für das Ignorieren von Klicks während des Countdowns vor Beginn des Spiels und das Aktivieren von Klicks nach Ende des Countdowns.

```

ts.
import { fromEvent, timer, interval } from 'rxjs';
importieren { skipUntil, take, scan } from 'rxjs';

// Erstellen von UI-Elementen
const container = document.createElement('div');.
document.body.appendChild(container);

const count = document.createElement('div');
countdown.style.fontSize = '24px';
countdown.style.marginBottom = '10px';
countdown.textContent = 'Countdown läuft...' ;
container.appendChild(countdown);

const button = document.createElement('button');
button.textContent = 'Klick!' ;
button.disabled = true;
container.appendChild(button);

const scoreDisplay = document.createElement('div');
scoreDisplay.style.marginTop = '10px';
scoreDisplay.textContent = 'score: 0';
container.appendChild(scoreDisplay);

// Countdown (3 Sekunden)
const countdownTimer$ = interval(1000).pipe(take(3));
countdownTimer$.subscribe({
  next: (n) => {
    countdown.textContent = `${3 - n} Sekunden bis zum Start... `;
  },.
  complete: () => {
    countdown.textContent = 'Das Spiel beginnt!' ;
    button.disabled = false;
  }
});

// Spielstart-Benachrichtigung
const gameStart$ = timer(3000);.

// Klick-Ereignis (springt zum Spielstart)
const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  skipUntil(gameStart$),.
  scan(Spielstand => Spielstand + 10, 0)
).subscribe(Spielstand => {
  scoreDisplay.textContent = `Punktestand: ${Punktestand}`;
});

```

In diesem Code wird der Countdown3Sekunden, Klicks werden während des Countdowns ignoriert, und nur Klicks nach dem Ende des Countdowns werden im Ergebnis berücksichtigt.

## 🎯 skip Der Unterschied zwischen skipUntil Unterschied zwischen

```

ts.
import { interval, timer } from 'rxjs';
importieren { skip, skipUntil } from 'rxjs';

const source$ = interval(500);.

// skip: Überspringen des ersten N nach Nummer
source$.pipe(
  skip(3)
).subscribe(console.log);
// Ausgabe: 3, 4, 5, 6, ...

// skipUntil: Überspringen, bis ein anderes Observable ausgelöst wird
source$.pipe(
  skipUntil(timer(1500))
).subscribe(console.log);.
// Ausgabe: 3, 4, 5, 6, ... (gleiches Ergebnis, aber andere Kontrollmethode)

```

| Bediener | Bedingungen überspringen | Anwendungsfall |
|---|---|---|
| `skip(n)` | ErstenÜberspringen einer bestimmten Anzahl von Teilen | Überspringen einer festen Anzahl |
| `skipWhile(predicate)` | Überspringen, wenn Bedingungen erfüllt sind | Bedingungsbasiertes Überspringen |
| `skipUntil(notifier$)` | Überspringen bis zum nächstenObservableÜberspringen, bis ein | Ereignis/Zeitbasiertes Überspringen |

## 📋 Typsichere Verwendung

TypeScript Dies ist ein Beispiel für eine typsichere Implementierung, bei der Generika verwendet werden

```

ts.
import { Observable, Subject, fromEvent } from 'rxjs';
importieren { skipUntil, map } from 'rxjs';

interface GameState {
  status: 'waiting' | 'ready' | 'playing' | 'finished';
}

interface ClickEvent {
  timestamp: Zahl; }
  x: Zahl;
  y: Zahl;
}

Klasse Game {
  private gameReady$ = new Subject();
  private state: GameState = { status: 'waiting' };.

  startGame(element: HTMLElement): Observable {
    const clicks$ = fromEvent\<MouseEvent>(element, 'click').pipe(
      map(event => ({
        timestamp: Date.now(),.
        x: event.clientX, event.
        y: event.clientY
      } as ClickEvent))),.
      skipUntil(this.gameReady$)
    );

    // Benachrichtigung über die Bereitschaft
    setTimeout(() => {
      this.state = { status: 'ready' };
      this.gameReady$.next();
      console.log('Spiel bereit!') ;
    }, 2000);

    return clicks$;
  }
}

// Beispiel für die Verwendung
const game = new Game();
const canvas = document.createElement('div');
canvas.style.width = '300px';
canvas.style.height = '200px';
canvas.style.border = '1px solid black';
canvas.textContent = 'Hier klicken';
document.body.appendChild(canvas);

game.startGame(canvas).subscribe(click => {
  console.log(`Klickposition: (${click.x}, ${click.y})`);
});

```

## 🔄 skipUntil Der Unterschied zwischen takeUntil Kombination von

Kombinieren Sie beides, wenn Sie nur Werte für einen bestimmten Zeitraum erhalten möchten.

```

ts.
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500);
const start$ = timer(2000); // Start nach 2 Sekunden
const stop$ = timer(5000); // stoppt nach 5 Sekunden

source$.pipe(
  skipUntil(start$), // Überspringen bis nach 2 Sekunden
  takeUntil(stop$); // nach 5 Sekunden anhalten
).subscribe({
  next: console.log,.
  complete: () => console.log('complete')
});
// Ausgabe: 4, 5, 6, 7, 8, 9, complete.
// (es werden nur Werte zwischen 2 und 5 Sekunden abgerufen)

```

**Zeitleisten**:
```

0s 1s 2s 3s 4s 5s

___TABELLE_11___

0 1 2 3 4 5 6 7 8 9 10
      ↑ hoch hoch hoch hoch hoch hoch
   SKIP Anfang TAKE Ende
   (von 4) (bis 9)

```

## ⚠️ Ein häufiger Fehler

> [!IMPORTANT]
> `skipUntil` sind Benachrichtigungen Observable der**Nur das erste Feuern**ist gültig.2Der zweite und die folgenden Abbrände werden ignoriert.

### Falsch: BenachrichtigungObservablewird mehr als einmal abgefeuert.

```

ts
import { interval, Subject } from 'rxjs';
importieren { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ❌ Schlechtes Beispiel: next mehrfach aufrufen, aber nur das erste Mal hat eine Wirkung
setTimeout(() => notifier$.next(), 1000);
setTimeout(() => notifier$.next(), 2000); // dies ist sinnlos

```

### Richtig.: Nur der erste Abschuss ist gültig.

```

ts.
import { interval, Subject } from 'rxjs';
importieren { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ✅ Gutes Beispiel: next nur einmal aufrufen
setTimeout(() => {
  console.log('Ende des Überspringens');
  notifier$.next();
  notifier$.complete(); // best practice to complete.
}, 1000);
```

## 🎓 Zusammenfassung

### Wann sollte skipUntil verwendet werden.
- ✅ Wenn Sie die Verarbeitung nach Eintreten eines bestimmten Ereignisses starten wollen
- ✅ Wenn Sie Benutzeroperationen nach Abschluss der Initialisierung ermöglichen wollen
- ✅ Wenn Sie einen zeitlich verzögerten Start benötigen
- ✅ Wenn Sie die Datenverarbeitung erst nach Abschluss der Authentifizierung starten wollen

### In Kombination mit takeUntil.
- ✅ Wenn Sie Werte nur für einen bestimmten Zeitraum abrufen wollen (skipUntil + takeUntil)

### Hinweise.
- ⚠️ Nur das erste Feuern des Observable ist gültig
- ⚠️ Wenn das Observable nicht ausgelöst wird, werden weiterhin alle Werte übersprungen
- ⚠️ Das Abonnement wird aufrechterhalten, bis der ursprüngliche Stream abgeschlossen ist.

## 🚀 Nächste Schritte.

- **[skip](./skip)** - lernen Sie, wie man die ersten N Werte überspringt.
- **[take](./take)** - lerne, wie man die ersten N Werte erhält.
- **[takeUntil](../utility/takeUntil)** - lerne, wie man Werte nimmt, bis ein anderes Observable feuert
- **[filter](./filter)** - lernen Sie, wie man auf der Grundlage von Bedingungen filtert
- **[filtering-operator-practical-use-cases](./practical-use-cases)** - lernen Sie echte Anwendungsfälle
