---
description: "zipWith ist ein RxJS-Verknüpfungsoperator, der das ursprüngliche Observable mit den entsprechenden geordneten Werten aus anderen Observables abgleicht und paart. Er eignet sich ideal für ordnungskritische Stream-Joins, wie z. B. die Kombination der Ergebnisse paralleler Verarbeitung mit Ordnungsgarantien, die Zuordnung von IDs zu Daten, die Synchronisierung verwandter Daten, die zu unterschiedlichen Zeiten veröffentlicht wurden, usw. Die pipe-fähige Operatorversion ist für die Verwendung in Pipelines geeignet."
---

# zipWith - Paar entsprechender Werte

Der Operator "zipWith" fasst die **entsprechenden geordneten Werte** zusammen, die von der ursprünglichen Observable und den anderen angegebenen Observables ausgegeben werden.
Er wartet, bis die Werte einer nach dem anderen von allen Observable eintreffen und bildet Paare, wenn sie fertig sind.
Dies ist die Pipeable Operator-Version von "zip" der Creation Function.

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
// (Dwird nicht ausgegeben, da es keinen entsprechenden Wert gibt)
```

- Von jedem Observable wird ein Paar ausgegeben, **wenn einer der Werte vollständig ist**.
- Wenn ein Observable abgeschlossen ist, werden die übrigen Werte verworfen.

[🌐 Offizielle RxJS Dokumentation - `zipWith`](https://rxjs.dev/api/operators/zipWith)

## 💡 Typisches Nutzungsmuster.

- **Parallele Verarbeitungsergebnisse in garantierter Reihenfolge kombinieren**: Paarung der Ergebnisse mehrerer API-Aufrufe.
- **Zuordnung von IDs zu Daten**: Kombination von Benutzer-IDs mit entsprechenden Profildaten
- **Synchronisierung von Datenströmen**: Synchronisierung zusammengehöriger Daten, die zu unterschiedlichen Zeitpunkten ausgegeben werden

## 🧠 Praktische Code-Beispiele (mit UI)

Beispiel für die Verknüpfung einer Liste von Benutzerkennungen mit den entsprechenden Benutzernamen in Folge.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Erstellung eines Ausgabebereichs
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith Praktische Beispiele für:</h3>';
document.body.appendChild(output);

// BenutzerIDStream (sofort veröffentlicht)
const userIds$ = from([101, 102, 103, 104]);

// User Name Stream (ausgegeben alle1(wird jede Sekunde ausgegeben)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// zipund angezeigt
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 BenutzerID ${id}: ${name}`;
    output.appendChild(item);
  });

// Ausgabe:
// 👤 BenutzerID 101: Alice
// 👤 BenutzerID 102: Bob
// 👤 BenutzerID 103: Carol
// (104wird nicht angezeigt, da es keinen entsprechenden Namen gibt)
```

- IDs und Namen werden in einer **Eins-zu-Eins-Entsprechung** gepaart.
- Wenn einer der Werte vollständig ist, werden die übrigen verworfen.

## 🔄 Unterschied zur Creation Function `zip`.

### Grundlegende Unterschiede.

|  | zip" (Creation Function). | zipWith" (Pipeable Operator) |
|---|---|---|
| **Wo verwendet** | Wird als eigenständige Funktion verwendet | Verwendet in der `.pipe()` Kette. |
| **Wie man schreibt**. | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Erster Stream**. | Alle als gleich behandeln | Als Hauptstrom behandeln |
| **Vorteile**. | Einfach und leicht zu lesen | Leicht mit anderen Operatoren zu kombinieren |

### Konkrete Beispiele für die Verwendung

**Für einfache Paarungen wird die Creation Function empfohlen**.

```ts
import { zip, of } from 'rxjs';

const questions$ = of('Der Name ist？', 'Das Alter ist？', 'Die Adresse lautet？');
const answers$ = of('Taro', '30', 'Tokio');
const scores$ = of(10, 20, 30);

// Einfach und lesbar
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, A: ${a}, Punktzahl: ${s}Spielstand`);
});
// Ausgabe:
// Q: Der Name ist？, A: Taro, Punktzahl: 10Spielstand
// Q: Das Alter ist？, A: 30, Punktzahl: 20Spielstand
// Q: Die Adresse lautet？, A: Tokio, Punktzahl: 30Spielstand
```

**Wenn Sie dem Hauptstrom einen Transformationsprozess hinzufügen möchten, wird der Pipeable Operator empfohlen**.

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Aufgabenliste
const tasks$ = from([
  { id: 1, name: 'Berichterstattung', priority: 'high' },
  { id: 2, name: 'Beantwortung von Emails', priority: 'low' },
  { id: 3, name: 'Vorbereitung von Besprechungen', priority: 'high' },
  { id: 4, name: 'Organisieren von Dokumenten', priority: 'medium' }
]);

// Liste der verantwortlichen Personen (1Zuweisung im Sekundentakt)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable OperatorAusgaben - Komplett in einer Pipeline
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // Nur hohe Priorität
    map(task => task.name),                     // Aufgabennamen extrahieren
    zipWith(assignees$),                        // Verantwortliche Person zuweisen
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Zugewiesen an: ${assignment.assignee}`);
  });
// Ausgabe:
// [Zeit] Berichterstattung → Zugewiesen an: Alice
// [Zeit] Vorbereitung von Besprechungen → Zugewiesen an: Bob

// ❌ Creation FunctionAusgaben - Redundant
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
  console.log(`[${assignment.assignedAt}] ${assignment.task} → Zugewiesen an: ${assignment.assignee}`);
});
```

**Synchronisation von auftragskritischen Daten**.

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UIerstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Quizspiel</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// Liste der Fragen (sofort vorbereitet)
const questions$ = from([
  'Die Hauptstadt von Japan ist？',
  '1+1ist？',
  'Welcher Planet ist die Erde?？'
]);

// Antwortliste (simuliert Benutzereingaben)：2(jede Sekunde)
const answers$ = from(['Tokio', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// Richtige Antwortliste
const correctAnswers$ = from(['Tokio', '2', '3']);

// ✅ Pipeable OperatorAusgaben - Fragen als Hauptstrom verarbeiten
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
      <strong>Frage${result.no}:</strong> ${result.question}<br>
      <strong>Antworten:</strong> ${result.answer}<br>
      <strong>Ergebnis:</strong> ${result.isCorrect ? '✅ Richtige Antwort！' : '❌ Falsche Antwort'}
    `;
    questionArea.appendChild(div);
  });
```

### Zusammenfassung.

- **`zip`**: am besten, wenn Sie nur mehrere Streams in der richtigen Reihenfolge abbilden wollen
- **`zipWith`**: ideal, wenn Sie den Hauptstrom mit anderen Strömen in garantierter Reihenfolge zusammenführen wollen, während Sie den Hauptstrom transformieren oder verarbeiten

## ⚠️ Hinweise.

### Für unterschiedliche Längen.

Wenn das kürzere Observable abgeschlossen ist, werden die verbleibenden Werte des längeren Observables verworfen.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// Ausgabe: [1, 'A'], [2, 'B'], [3, 'C']
// 'D'und'E'werden verworfen
```

### Speicherakkumulation.

Wenn ein Observable weiterhin Werte ausgibt, sammeln sich die Werte im Speicher an, bis das andere Observable aufholt.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Schnelle Ströme (100mspro Datenstrom)
const fast$ = interval(100).pipe(take(10));

// Langsame Streams (1(jede Sekunde)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// Ausgabe: [0, 0] (1Sekunden später), [1, 1] (2Sekunden später), [2, 2] (3Sekunden später)
// fast$Werte werden im Speicher abgelegt und abgewartet
```

### Unterschied zu combineLatestWith.

`zipWith` paart die Werte in der entsprechenden Reihenfolge, während `combineLatestWith` die neuesten Werte kombiniert.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Gepaart in entsprechender Reihenfolge
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Ausgabe: [0, 0], [1, 1]

// combineLatestWith: Kombinieren der letzten Werte
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Ausgabe: [0, 0], [1, 0], [2, 0], [2, 1]
```

## 📚 Verwandte Operatoren.

- **[zip](/de/guide/creation-functions/combination/zip)** - Creation Function Version.
- **[combineLatestWith](/de/guide/operators/combination/combineLatestWith)** - Neuesten Wert kombinieren
- **[withLatestFrom](/de/guide/operators/combination/withLatestFrom)** - nur Mainstream-Auslöser
