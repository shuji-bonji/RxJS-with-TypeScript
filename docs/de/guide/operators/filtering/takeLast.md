---
description: "takeLast ist ein RxJS-Filteroperator, der nur die letzten N Werte ausgibt, wenn ein Observable-Stream abgeschlossen ist. Er ist ideal für Situationen, in denen nur der letzte Wert des gesamten Streams benötigt wird, wie z. B. das Abrufen der letzten Zählung im Protokoll, das Anzeigen der obersten N Werte im Leaderboard oder die endgültige Datenzusammenfassung nach Abschluss. Kann nicht mit unendlichen Streams verwendet werden, da der Wert bis zum Abschluss in einem Puffer gehalten wird."
---

# takeLast - liefert die letzten N Werte

Der Operator `takeLast` gibt nur die letzten N Werte zu dem Zeitpunkt aus, an dem der Stream **vollständig** ist. Er behält die Werte in einem Puffer, bis der Stream abgeschlossen ist, und gibt sie nach Abschluss zusammen aus.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9
```

**Ablauf der Operation**:.
1. Stream gibt 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 aus
2. intern die letzten 3 im Puffer halten
3. Stream abgeschlossen 4. Pufferwerte 7, 8, 9
4. gibt die Pufferwerte 7, 8, 9 nacheinander aus

[🌐 Offizielle RxJS Dokumentation - `takeLast`](https://rxjs.dev/api/operators/takeLast)

## 🆚 Im Gegensatz zu take.

`take` und `takeLast` haben ein gegensätzliches Verhalten.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

// take: Die ersteNHolt den ersten
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2(sofort ausgeben)

// takeLast: Holt die letzteNHolt den ersten
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9(vor der Ausgabe auf die Fertigstellung warten)
```

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9
```

## 💡 Typisches Nutzungsmuster

1. **Die letzten N Protokolleinträge abrufen**.


```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'warn' as const, message: 'Slow query detected' },
     { timestamp: 4, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 5, level: 'info' as const, message: 'Retry successful' },
   ] as LogEntry[]);

   // Abrufen der letzten3Abrufen von Protokollen der
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Ausgabe:
   // [warn] Slow query detected
   // [error] Connection failed
   // [info] Retry successful
   ```

2. **Spitze der RanglisteNAbrufen der Spitze**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]).pipe(
     // Sortierung nach Punktzahl vorausgesetzt
   );

   // Abrufen der Spitze3Abrufen der
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Ausgabe: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Endgültige Zusammenfassung nach Abschluss der DatenverarbeitungNZusammenfassung der Fälle**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Simulation der Sensordaten
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // Holt die letzte5Berechnung der durchschnittlichen Temperatur des Gehäuses
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`Daten${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Letzte5Datenerfassung des Falles abgeschlossen');
     }
   });
   ```

## 🧠 Praktisches Code-Beispiel (Eingabe-Historie)

Beispiel für die Anzeige der letzten Werte3Dies ist ein Beispiel für die Anzeige der letzten vom Benutzer eingegebenen Werte.

```

ts.
import { fromEvent, Subject } from 'rxjs';
importiere { takeLast } von 'rxjs';

// Erstellen von UI-Elementen
const container = document.createElement('div');
document.body.appendChild(container);

const input = document.createElement('input');
input.placeholder = 'Geben Sie einen Wert ein und bestätigen Sie';
container.appendChild(input);

const submitButton = document.createElement('button');
submitButton.textContent = 'Verlauf anzeigen (letzte 3)';
container.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
container.appendChild(historyDisplay);

// Subject zur Aufnahme von Eingabewerten
const inputs$ = new Subject();.

// **WICHTIG**: takeLast Abonnement zuerst setzen
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (Wert) => {
    const item = document.createElement('div');
    item.textContent = `- ${Wert}`;
    historyDisplay.appendChild(item);
  },.
  complete: () => {
    const note = document.createElement('div');
    note.style.marginTop = '5px';
    note.style.color = 'grau';
    note.textContent = '(Laden Sie die Seite neu, um erneut zu tippen)';
    historyDisplay.appendChild(note);

    // Eingabefelder und Schaltflächen deaktivieren
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Eingabe mit der Eingabetaste hinzufügen
fromEvent\<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`Hinzufügen: ${input.value}`);
    input.value = '';
  }
});

// Vervollständigen Sie mit dem Klick auf die Schaltfläche und zeigen Sie den Verlauf an
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>History (latest 3):</strong><br>';
  inputs$.complete(); // Stream complete → takeLast feuert
});

```

> [!IMPORTANT]
> **Wichtige Punkte**:
> - `takeLast(3)` Abonnieren Sie die**zuerst.**muss zuerst eingerichtet werden
> - wenn die Schaltfläche angeklickt wird. `complete()` wird der letzte der bis dahin empfangenen Werte ausgegeben.3Der letzte bis zu diesem Zeitpunkt empfangene Wert wird ausgegeben.
> - `complete()` Nach dem Aufruf**Nach dem Aufruf von**an `subscribe` fließen keine Werte, wenn Sie

## ⚠️ Ein wichtiger Punkt, der zu beachten ist

> [!WARNING]
> `takeLast` ist, dass Sie warten müssen, bis der Stream**Warten bis zum Abschluss**Daher funktioniert es nicht mit unendlichen Streams. Außerdem muss die`takeLast(n)` desngroß ist, verbraucht es viel Speicher.

### 1. Kann nicht mit unendlichen Streams verwendet werden.

`takeLast` funktioniert nicht mit unendlichen Streams, da es wartet, bis der Stream abgeschlossen ist.

```

ts.
import { interval } from 'rxjs';
importieren { takeLast } from 'rxjs';

// ❌ Schlechtes Beispiel: Verwendung von takeLast mit unendlichen Streams
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);.
// Es wird nichts ausgegeben (weil der Stream nie abgeschlossen wird)

```

**Lösung.**: `take` Verwenden Sie einen endlichen Stream in Kombination mit

```

ts.
import { interval } from 'rxjs';
importiere { take, takeLast } von 'rxjs';

// ✅ Gutes Beispiel: endlicher Stream, dann takeLast verwenden
interval(1000).pipe(
  take(10), // Komplett mit den ersten 10
  takeLast(3), // take die letzten 3 davon
).subscribe(console.log);.
// Ausgabe: 7, 8, 9

```

### 2. Achten Sie auf die Speichernutzung

`takeLast(n)` funktioniert nicht mit endlichen Streams, da es das letzte StücknStück im Puffer gehalten werden soll,ngroß ist, verbraucht er mehr Speicher.

```

ts.
import { range } from 'rxjs';
importieren { takeLast } from 'rxjs';

// ⚠️ Hinweis: Große Datenmengen werden in einem Puffer gehalten
range(0, 1000000).pipe(
  takeLast(100000) // 100.000 Datensätze werden im Speicher gehalten
).subscribe(console.log);.

```

## 🎯 last Der Unterschied zwischen

```

ts.
import { range } from 'rxjs';
importieren { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: nur die letzte Zahl
numbers$.pipe(
  last()
).subscribe(console.log);
// Ausgabe: 9

// takeLast(1): letzter Wert (Ausgabe als Einzelwert, nicht als Array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);.
// Ausgabe: 9

// takeLast(3): letzte 3
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9

```

| Betreiber | Anzahl der Erfassungen | Spezifikation der Bedingung | Anwendungsfall |
|---|---|---|---|
| `last()` | 1Anzahl der | Mögliche | Holt die letzte1Stücke oder das letzte Stück, das die Bedingung erfüllt1Anzahl der |
| `takeLast(n)` | nAnzahl der | Unmöglich | Holt die letztenEinfach das letzte Stück ermitteln, das die Bedingung erfüllt |

## 📋 Typsichere Verwendung

TypeScript Dies ist ein Beispiel für eine typsichere Implementierung, die die Generika in

```

ts.
import { Observable, from } from 'rxjs';
importiere { takeLast } von 'rxjs';

Schnittstelle Transaction {
  id: string;
  amount: Zahl;
  timestamp: Datum;
  status: 'pending' | 'completed' | 'failed'; }
}

function getRecentTransactions(
  transactions$: Observable,.
  count: Zahl
): Observable {
  return transactions$.pipe(
    takeLast(count)
  );
}

// Beispiel für die Verwendung
const transactions$ = from([.
  { id: '1', amount: 100, timestamp: new Date('2025-01-01'), status: 'completed' as const }
  { id: '2', Betrag: 200, Zeitstempel: new Date('2025-01-02'), Status: 'completed' as const }
  { id: '3', Betrag: 150, Zeitstempel: new Date('2025-01-03'), Status: 'pending' as const }
  { id: '4', Betrag: 300, Zeitstempel: new Date('2025-01-04'), Status: 'completed' as const }
  { id: '5', Betrag: 250, Zeitstempel: new Date('2025-01-05'), Status: 'failed' as const }
] as Transaction[]);.

// Abrufen der drei jüngsten Transaktionen
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount} yen (${tx.status})`);
});
// Ausgabe:.
// 3: 150 yen (ausstehend)
// 4: 300 Yen (abgeschlossen)
// 5: ¥250 (fehlgeschlagen)

```

## 🔄 skip und takeLast Kombination von

Der mittlere Teil des Wertes wird ausgeschlossen und nur der letzteNNur der letzte Wert kann abgerufen werden.

```

ts
import { range } from 'rxjs';
importieren { skip, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

// Überspringe die ersten 5 und nimm die verbleibenden letzten 3
numbers$.pipe(
  skip(5), // skip 0, 1, 2, 3, 4
  takeLast(3) // nimmt die letzten 3 der verbleibenden 5, 6, 7, 8, 9
).subscribe(console.log);.
// Ausgabe: 7, 8, 9
```

## 🎓 Zusammenfassung

### Wann sollte takeLast verwendet werden.
- ✅ Wenn Sie die letzten N Daten in einem Stream benötigen
- ✅ Wenn Sie die letzten N Logs oder Transaktionen abrufen wollen
- ✅ Wenn der Stream garantiert abgeschlossen ist
- ✅ Wenn Sie eine Zusammenfassung oder die ersten N Datensätze anzeigen wollen

### Wann Sie take verwenden sollten.
- ✅ Wenn Sie die ersten N Daten des Datenstroms benötigen
- ✅ Wenn Sie die Ergebnisse sofort erhalten wollen
- ✅ Wenn Sie einen Teil eines unendlichen Datenstroms abrufen wollen

### Hinweise.
- ⚠️ Kann nicht mit unendlichen Datenströmen verwendet werden (da sie nicht vollständig sind)
- ⚠️ Großes n in `takeLast(n)` verbraucht Speicher
- ⚠️ Die Ausgabe wird nach Abschluss kompiliert (nicht sofort)
- ⚠️ Muss oft mit `take(n)` kombiniert werden, um einen endlichen Stream zu erzeugen

## 🚀 Nächster Schritt.

- **[take](./take)** - lernen, wie man die ersten n Werte erhält.
- **[last](./last)** - lerne, wie man den letzten 1 Wert erhält
- **[skip](./skip)** - lerne, wie man die ersten N Werte überspringt
- **[filter](./filter)** - lerne, wie man anhand von Bedingungen filtert
- **[filtering-operator-practical-use-cases](./practical-use-cases)** - lernen Sie, wie man echte Anwendungsfälle verwendet
