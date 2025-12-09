---
description: takeLast ist ein RxJS-Filteroperator, der nur die letzten N Werte ausgibt, wenn ein Observable-Stream abgeschlossen wird. Ideal für Szenarien wie Abrufen der neuesten Protokolleinträge, Anzeige der Top-N-Einträge einer Rangliste oder abschließende Datenzusammenfassungen, bei denen nur die letzten Werte aus dem gesamten Stream benötigt werden. Da Werte bis zum Abschluss im Puffer gehalten werden, kann es nicht mit unendlichen Streams verwendet werden.
---

# takeLast - Die letzten N Werte abrufen

Der `takeLast`-Operator gibt nur die letzten N Werte aus, **wenn der Stream abgeschlossen wird**. Werte werden im Puffer gehalten, bis der Stream abgeschlossen wird, und dann zusammen ausgegeben.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9
```

**Ablauf**:
1. Stream gibt 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 aus
2. Intern werden die letzten 3 Werte im Puffer gehalten
3. Stream wird abgeschlossen
4. Gepufferte Werte 7, 8, 9 werden nacheinander ausgegeben

[🌐 Offizielle RxJS-Dokumentation - `takeLast`](https://rxjs.dev/api/operators/takeLast)


## 🆚 Vergleich mit take

`take` und `takeLast` verhalten sich gegensätzlich.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

// take: Die ersten N Werte abrufen
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2 (sofort ausgegeben)

// takeLast: Die letzten N Werte abrufen
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9 (nach Abschluss ausgegeben)
```

| Operator | Abrufposition | Ausgabe-Timing | Verhalten vor Abschluss |
|---|---|---|---|
| `take(n)` | Erste n Werte | Sofort ausgegeben | Automatischer Abschluss nach n Werten |
| `takeLast(n)` | Letzte n Werte | Zusammen nach Abschluss ausgegeben | Im Puffer gehalten |


## 💡 Typische Anwendungsmuster

1. **Neueste N Protokolleinträge abrufen**
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

   // Neueste 3 Protokolleinträge abrufen
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

2. **Top-N-Einträge einer Rangliste abrufen**
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
     // Angenommen, nach Score sortiert
   );

   // Top 3 abrufen
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Ausgabe: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Abschließende N Einträge nach Datenverarbeitung**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Sensordaten-Simulation
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // Durchschnittstemperatur der letzten 5 Einträge berechnen
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`Daten ${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Abruf der neuesten 5 Daten abgeschlossen');
     }
   });
   ```


## 🧠 Praxisbeispiel (Eingabeverlauf)

Beispiel, das die neuesten 3 vom Benutzer eingegebenen Werte anzeigt.

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeLast } from 'rxjs';

// UI-Elemente erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const input = document.createElement('input');
input.placeholder = 'Wert eingeben und Enter drücken';
container.appendChild(input);

const submitButton = document.createElement('button');
submitButton.textContent = 'Verlauf anzeigen (neueste 3)';
container.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
container.appendChild(historyDisplay);

// Subject zum Halten der Eingabewerte
const inputs$ = new Subject<string>();

// **Wichtig**: takeLast-Abonnement zuerst einrichten
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `・ ${value}`;
    historyDisplay.appendChild(item);
  },
  complete: () => {
    const note = document.createElement('div');
    note.style.marginTop = '5px';
    note.style.color = 'gray';
    note.textContent = '(Seite neu laden, um erneut einzugeben)';
    historyDisplay.appendChild(note);

    // Eingabefeld und Button deaktivieren
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Enter-Taste zum Hinzufügen von Eingaben
fromEvent<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`Hinzugefügt: ${input.value}`);
    input.value = '';
  }
});

// Bei Buttonklick abschließen und Verlauf anzeigen
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>Verlauf (neueste 3):</strong><br>';
  inputs$.complete(); // Stream abschließen → takeLast wird ausgelöst
});
```

> [!IMPORTANT]
> **Wichtiger Punkt**:
> - Das `takeLast(3)`-Abonnement muss **zuerst** eingerichtet werden
> - Wenn Sie `complete()` aufrufen, werden die letzten 3 der bis dahin empfangenen Werte ausgegeben
> - Wenn Sie **nach** `complete()` ein `subscribe` durchführen, fließen keine Werte mehr


## ⚠️ Wichtige Hinweise

> [!WARNING]
> Da `takeLast` **bis zum Abschluss des Streams wartet**, funktioniert es nicht mit unendlichen Streams. Außerdem verbraucht `takeLast(n)` viel Speicher, wenn n groß ist.

### 1. Nicht mit unendlichen Streams verwendbar

Da `takeLast` bis zum Abschluss des Streams wartet, funktioniert es nicht mit unendlichen Streams.

```ts
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Schlechtes Beispiel: takeLast mit unendlichem Stream verwenden
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);
// Keine Ausgabe (Stream wird nicht abgeschlossen)
```

**Lösung**: Mit `take` zu einem endlichen Stream machen

```ts
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Gutes Beispiel: Endlichen Stream erstellen, dann takeLast verwenden
interval(1000).pipe(
  take(10),      // Mit ersten 10 Werten abschließen
  takeLast(3)    // Davon die letzten 3 abrufen
).subscribe(console.log);
// Ausgabe: 7, 8, 9
```

### 2. Auf Speicherverbrauch achten

`takeLast(n)` hält die letzten n Werte im Puffer, daher wird bei großem n Speicher verbraucht.

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Vorsicht: Große Datenmenge im Puffer halten
range(0, 1000000).pipe(
  takeLast(100000) // 100.000 Einträge im Speicher halten
).subscribe(console.log);
```


## 🎯 Unterschied zu last

```ts
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: Nur der letzte eine Wert
numbers$.pipe(
  last()
).subscribe(console.log);
// Ausgabe: 9

// takeLast(1): Der letzte eine Wert (als einzelner Wert ausgegeben, nicht als Array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);
// Ausgabe: 9

// takeLast(3): Die letzten 3 Werte
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9
```

| Operator | Anzahl | Bedingungsangabe | Anwendungsfall |
|---|---|---|---|
| `last()` | 1 Wert | Möglich | Letzter Wert oder letzter Wert, der Bedingung erfüllt |
| `takeLast(n)` | n Werte | Nicht möglich | Einfaches Abrufen der letzten n Werte |


## 📋 Typsichere Verwendung

Beispiel für typsichere Implementierung mit TypeScript-Generics.

```ts
import { Observable, from } from 'rxjs';
import { takeLast } from 'rxjs';

interface Transaction {
  id: string;
  amount: number;
  timestamp: Date;
  status: 'pending' | 'completed' | 'failed';
}

function getRecentTransactions(
  transactions$: Observable<Transaction>,
  count: number
): Observable<Transaction> {
  return transactions$.pipe(
    takeLast(count)
  );
}

// Verwendungsbeispiel
const transactions$ = from([
  { id: '1', amount: 100, timestamp: new Date('2025-01-01'), status: 'completed' as const },
  { id: '2', amount: 200, timestamp: new Date('2025-01-02'), status: 'completed' as const },
  { id: '3', amount: 150, timestamp: new Date('2025-01-03'), status: 'pending' as const },
  { id: '4', amount: 300, timestamp: new Date('2025-01-04'), status: 'completed' as const },
  { id: '5', amount: 250, timestamp: new Date('2025-01-05'), status: 'failed' as const },
] as Transaction[]);

// Neueste 3 Transaktionen abrufen
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount}円 (${tx.status})`);
});
// Ausgabe:
// 3: 150円 (pending)
// 4: 300円 (completed)
// 5: 250円 (failed)
```


## 🔄 Kombination von skip und takeLast

Mittlere Werte ausschließen und nur die letzten N abrufen.

```ts
import { range } from 'rxjs';
import { skip, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

// Erste 5 überspringen und von den verbleibenden die letzten 3 abrufen
numbers$.pipe(
  skip(5),      // 0, 1, 2, 3, 4 überspringen
  takeLast(3)   // Von den verbleibenden 5, 6, 7, 8, 9 die letzten 3
).subscribe(console.log);
// Ausgabe: 7, 8, 9
```


## 🎓 Zusammenfassung

### Wann takeLast verwenden
- ✅ Wenn die letzten N Daten eines Streams benötigt werden
- ✅ Wenn die neuesten N Protokoll- oder Transaktionseinträge abgerufen werden sollen
- ✅ Wenn sichergestellt ist, dass der Stream abgeschlossen wird
- ✅ Wenn Datenzusammenfassungen oder Top-N-Einträge angezeigt werden sollen

### Wann take verwenden
- ✅ Wenn die ersten N Daten eines Streams benötigt werden
- ✅ Wenn Ergebnisse sofort abgerufen werden sollen
- ✅ Wenn ein Teil von einem unendlichen Stream abgerufen werden soll

### Vorsicht
- ⚠️ Nicht mit unendlichen Streams verwendbar (wird nicht abgeschlossen)
- ⚠️ `takeLast(n)` verbraucht Speicher bei großem n
- ⚠️ Ausgabe erfolgt zusammen nach Abschluss (nicht sofort)
- ⚠️ Oft muss mit `take(n)` ein endlicher Stream erstellt werden


## 🚀 Nächste Schritte

- **[take](./take)** - Lernen Sie, wie man die ersten N Werte abruft
- **[last](./last)** - Lernen Sie, wie man den letzten Wert abruft
- **[skip](./skip)** - Lernen Sie, wie man die ersten N Werte überspringt
- **[filter](./filter)** - Lernen Sie, wie man basierend auf Bedingungen filtert
- **[Praktische Beispiele für Filteroperatoren](./practical-use-cases)** - Lernen Sie reale Anwendungsfälle
