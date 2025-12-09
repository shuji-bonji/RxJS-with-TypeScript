---
description: zipWith ist ein RxJS-Kombinationsoperator, der entsprechende Werte in der Reihenfolge vom ursprünglichen Observable und anderen Observables paart. Ideal für Stream-Kombinationen, bei denen die Reihenfolge wichtig ist, wie Verbindung von Ergebnissen paralleler Verarbeitung mit Reihenfolgegarantie, Zuordnung von IDs zu Daten und Synchronisierung von verwandten Daten, die zu unterschiedlichen Zeiten ausgegeben werden. Als Pipeable Operator bequem für die Verwendung in Pipelines.
---

# zipWith - Entsprechende Werte in der Pipeline paaren

Der `zipWith`-Operator fasst **entsprechende Werte in der Reihenfolge** zusammen, die vom ursprünglichen Observable und anderen angegebenen Observables ausgegeben werden.
Er wartet, bis von allen Observables jeweils ein Wert angekommen ist, und erstellt dann Paare, wenn sie ausgerichtet sind.
Dies ist die Pipeable Operator-Version der Creation Function `zip`.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const letters$ = of('A', 'B', 'C', 'D');
const numbers$ = interval(1000).pipe(
  map(val => val * 10),
  take(3)
);

letters$
  .pipe(zipWith(numbers$))
  .subscribe(([letter, number]) => {
    console.log(`${letter} - ${number}`);
  });

// Ausgabe:
// A - 0
// B - 10
// C - 20
// (D wird nicht ausgegeben, da kein entsprechender Wert vorhanden ist)
```

- Paare werden ausgegeben, wenn **jeweils ein Wert von jedem Observable vorhanden ist**.
- Wenn eines der Observables abgeschlossen wird, werden die verbleibenden Werte verworfen.

[🌐 RxJS Official Documentation - `zipWith`](https://rxjs.dev/api/operators/zipWith)


## 💡 Typische Anwendungsmuster

- **Verbindung von Ergebnissen paralleler Verarbeitung mit Reihenfolgegarantie**: Paarung von Ergebnissen mehrerer API-Aufrufe
- **Zuordnung von IDs zu Daten**: Verbindung von Benutzer-IDs mit entsprechenden Profildaten
- **Stream-Synchronisierung**: Synchronisierung von verwandten Daten, die zu unterschiedlichen Zeiten ausgegeben werden


## 🧠 Praktisches Codebeispiel (mit UI)

Ein Beispiel, das Benutzer-IDs und entsprechende Benutzernamen nacheinander paart und anzeigt.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Benutzer-ID-Stream (sofort ausgegeben)
const userIds$ = from([101, 102, 103, 104]);

// Benutzernamen-Stream (jede Sekunde ausgegeben)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// Mit zip anzeigen
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 Benutzer-ID ${id}: ${name}`;
    output.appendChild(item);
  });

// Ausgabe:
// 👤 Benutzer-ID 101: Alice
// 👤 Benutzer-ID 102: Bob
// 👤 Benutzer-ID 103: Carol
// (104 wird nicht angezeigt, da kein entsprechender Name vorhanden ist)
```

- IDs und Namen werden **1:1 entsprechend** gepaart.
- Wenn eines abgeschlossen wird, werden die verbleibenden Werte verworfen.


## 🔄 Unterschied zur Creation Function `zip`

### Grundlegender Unterschied

| | `zip` (Creation Function) | `zipWith` (Pipeable Operator) |
|:---|:---|:---|
| **Verwendungsort** | Als unabhängige Funktion | Innerhalb einer `.pipe()`-Kette |
| **Schreibweise** | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Erster Stream** | Alle gleichwertig behandelt | Als Hauptstream behandelt |
| **Vorteil** | Einfach und lesbar | Leicht mit anderen Operatoren kombinierbar |

### Konkrete Beispiele für die Auswahl

**Wenn nur einfache Paarung → Creation Function empfohlen**

```ts
import { zip, of } from 'rxjs';

const questions$ = of('Name?', 'Alter?', 'Adresse?');
const answers$ = of('Taro', '30', 'Tokyo');
const scores$ = of(10, 20, 30);

// Einfach und lesbar
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`F: ${q}, A: ${a}, Punkte: ${s}`);
});
// Ausgabe:
// F: Name?, A: Taro, Punkte: 10
// F: Alter?, A: 30, Punkte: 20
// F: Adresse?, A: Tokyo, Punkte: 30
```

**Wenn Transformation zum Hauptstream hinzugefügt werden soll → Pipeable Operator empfohlen**

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Aufgabenliste
const tasks$ = from([
  { id: 1, name: 'Bericht erstellen', priority: 'high' },
  { id: 2, name: 'E-Mail beantworten', priority: 'low' },
  { id: 3, name: 'Meeting vorbereiten', priority: 'high' },
  { id: 4, name: 'Dokumente sortieren', priority: 'medium' }
]);

// Zuständigenliste (jede Sekunde zuweisen)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable Operator-Version - vollständig in einer Pipeline
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // Nur hohe Priorität
    map(task => task.name),                     // Nur Aufgabennamen extrahieren
    zipWith(assignees$),                        // Zuständigen zuweisen
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Zuständig: ${assignment.assignee}`);
  });
// Ausgabe:
// [Zeit] Bericht erstellen → Zuständig: Alice
// [Zeit] Meeting vorbereiten → Zuständig: Bob

// ❌ Creation Function-Version - wird umständlich
import { zip } from 'rxjs';
zip(
  tasks$.pipe(
    filter(task => task.priority === 'high'),
    map(task => task.name)
  ),
  assignees$
).pipe(
  map(([taskName, assignee]) => ({
    task: taskName,
    assignee,
    assignedAt: new Date().toLocaleTimeString()
  }))
).subscribe(assignment => {
  console.log(`[${assignment.assignedAt}] ${assignment.task} → Zuständig: ${assignment.assignee}`);
});
```

**Synchronisierung von Daten, bei denen die Reihenfolge wichtig ist**

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UI erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Quiz-Spiel</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// Fragenliste (sofort vorbereitet)
const questions$ = from([
  'Was ist die Hauptstadt von Japan?',
  'Was ist 1+1?',
  'Die Erde ist der wievielte Planet?'
]);

// Antwortenliste (Simulation der Benutzereingabe: alle 2 Sekunden)
const answers$ = from(['Tokyo', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// Richtige Antworten
const correctAnswers$ = from(['Tokyo', '2', '3']);

// ✅ Pipeable Operator-Version - Fragen als Hauptstream verarbeiten
questions$
  .pipe(
    zipWith(answers$, correctAnswers$),
    map(([question, answer, correct], index) => ({
      no: index + 1,
      question,
      answer,
      correct,
      isCorrect: answer === correct
    }))
  )
  .subscribe(result => {
    const div = document.createElement('div');
    div.style.marginTop = '10px';
    div.style.padding = '10px';
    div.style.border = '1px solid #ccc';
    div.style.backgroundColor = result.isCorrect ? '#e8f5e9' : '#ffebee';
    div.innerHTML = `
      <strong>Frage ${result.no}:</strong> ${result.question}<br>
      <strong>Antwort:</strong> ${result.answer}<br>
      <strong>Ergebnis:</strong> ${result.isCorrect ? '✅ Richtig!' : '❌ Falsch'}
    `;
    questionArea.appendChild(div);
  });
```

### Zusammenfassung

- **`zip`**: Optimal, wenn mehrere Streams einfach nach Reihenfolge zugeordnet werden sollen
- **`zipWith`**: Optimal, wenn dem Hauptstream Transformationen oder Verarbeitungen hinzugefügt werden sollen, während mit anderen Streams mit Reihenfolgegarantie verbunden wird


## ⚠️ Wichtige Hinweise

### Bei unterschiedlicher Länge

Wenn das kürzere Observable abgeschlossen wird, werden die verbleibenden Werte des längeren verworfen.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// Ausgabe: [1, 'A'], [2, 'B'], [3, 'C']
// 'D' und 'E' werden verworfen
```

### Speicheransammlung

Wenn eines der Observables weiterhin Werte ausgibt, werden die Werte im Speicher angesammelt, bis das andere aufholt.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Schneller Stream (alle 100ms)
const fast$ = interval(100).pipe(take(10));

// Langsamer Stream (jede Sekunde)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// Ausgabe: [0, 0] (nach 1 Sekunde), [1, 1] (nach 2 Sekunden), [2, 2] (nach 3 Sekunden)
// Die Werte von fast$ werden im Speicher angesammelt und warten
```

### Unterschied zu combineLatestWith

`zipWith` paart nach entsprechender Reihenfolge, während `combineLatestWith` die neuesten Werte kombiniert.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Paarung nach entsprechender Reihenfolge
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Ausgabe: [0, 0], [1, 1]

// combineLatestWith: Kombination der neuesten Werte
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Ausgabe: [0, 0], [1, 0], [2, 0], [2, 1]
```


## 📚 Verwandte Operatoren

- **[zip](/de/guide/creation-functions/combination/zip)** - Creation Function-Version
- **[combineLatestWith](/de/guide/operators/combination/combineLatestWith)** - Neueste Werte kombinieren
- **[withLatestFrom](/de/guide/operators/combination/withLatestFrom)** - Nur Hauptstream als Trigger
