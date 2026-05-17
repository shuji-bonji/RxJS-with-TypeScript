---
description: concatWith ist ein RxJS-Kombinationsoperator, der andere Observables nacheinander verbindet, nachdem das ursprüngliche Observable abgeschlossen ist. Ideal für sequentielle Verarbeitung in Pipelines, Folgeaktionen nach Abschluss und schrittweises Laden von Daten, wenn Sie nachfolgende Prozesse als Erweiterung des Hauptstreams hinzufügen möchten. Als Pipeable Operator bequem für die Verwendung in Pipelines.
---

# concatWith - Streams nacheinander in der Pipeline verbinden

Der `concatWith`-Operator verbindet **nacheinander** andere angegebene Observables, nachdem das ursprüngliche Observable `complete` wurde.
Dies ist die Pipeable Operator-Version der Creation Function `concat`.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of, delay } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));
const obs3$ = of('E', 'F').pipe(delay(100));

obs1$
  .pipe(concatWith(obs2$, obs3$))
  .subscribe(console.log);

// Ausgabe: A → B → C → D → E → F
```

- `obs2$` startet erst, nachdem `obs1$` abgeschlossen ist, und `obs3$` startet erst, nachdem `obs2$` abgeschlossen ist.
- Da es innerhalb einer `.pipe()`-Kette verwendet werden kann, lässt es sich leicht mit anderen Operatoren kombinieren.

[🌐 RxJS Official Documentation - `concatWith`](https://rxjs.dev/api/operators/concatWith)


## 💡 Typische Anwendungsmuster

- **Sequentielle Verarbeitung in Pipelines**: Nacheinander zusätzliche Daten an einen transformierten Stream anhängen
- **Folgeaktionen nach Abschluss**: Bereinigung oder Benachrichtigungen nach Abschluss der Hauptverarbeitung hinzufügen
- **Schrittweises Laden von Daten**: Nach dem Abrufen der Anfangsdaten sequentiell zusätzliche Daten abrufen


## 🧠 Praktisches Codebeispiel (mit UI)

Ein Beispiel, das zuerst die Hauptsuchergebnisse anzeigt und dann nacheinander verwandte empfohlene Artikel anzeigt.

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>concatWith Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Hauptsuchergebnisse
const searchResults$ = of('🔍 Suchergebnis 1', '🔍 Suchergebnis 2', '🔍 Suchergebnis 3').pipe(
  delay(500)
);

// Empfohlene Artikel 1
const recommendations1$ = of('💡 Empfohlenes Produkt A', '💡 Empfohlenes Produkt B').pipe(
  delay(300)
);

// Empfohlene Artikel 2
const recommendations2$ = of('⭐ Beliebtes Produkt X', '⭐ Beliebtes Produkt Y').pipe(
  delay(300)
);

// Nacheinander verbinden und anzeigen
searchResults$
  .pipe(
    concatWith(recommendations1$, recommendations2$),
    map((value, index) => `${index + 1}. ${value}`)
  )
  .subscribe((value) => {
    const item = document.createElement('div');
    item.textContent = value;
    output.appendChild(item);
  });
```

- Zuerst werden die Suchergebnisse angezeigt,
- Dann werden die empfohlenen Produkte nacheinander angezeigt.
- Kann in der Pipeline mit anderen Operatoren wie `map` kombiniert werden.


## 🔄 Unterschied zur Creation Function `concat`

### Grundlegender Unterschied

| | `concat` (Creation Function) | `concatWith` (Pipeable Operator) |
|:---|:---|:---|
| **Verwendungsort** | Als unabhängige Funktion | Innerhalb einer `.pipe()`-Kette |
| **Schreibweise** | `concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **Erster Stream** | Alle gleichwertig behandelt | Als Hauptstream behandelt |
| **Vorteil** | Einfach und lesbar | Leicht mit anderen Operatoren kombinierbar |

### Konkrete Beispiele für die Auswahl

**Wenn nur einfache Verbindung → Creation Function empfohlen**

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// Einfach und lesbar
concat(part1$, part2$, part3$).subscribe(console.log);
// Ausgabe: A → B → C → D → E → F
```

**Wenn Transformation erforderlich → Pipeable Operator empfohlen**

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ❌ Creation Function-Version - wird umständlich
import { concat } from 'rxjs';
concat(
  userData$.pipe(
    filter(user => user.age >= 30),
    map(user => user.name)
  ),
  additionalData$.pipe(map(user => user.name))
).subscribe(console.log);

// ✅ Pipeable Operator-Version - vollständig in einer Pipeline
userData$
  .pipe(
    filter(user => user.age >= 30),  // Nur 30 Jahre und älter
    map(user => user.name),          // Nur Namen extrahieren
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// Ausgabe: Alice → Charlie
```

**Wenn Folgeaktionen zum Hauptstream hinzugefügt werden**

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, map } from 'rxjs';

// Button und Ausgabebereich erstellen
const button = document.createElement('button');
button.textContent = 'Bitte 3 Mal klicken';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Pipeable Operator-Version - natürlich als Erweiterung des Hauptstreams
clicks$
  .pipe(
    take(3),                          // Erste 3 Klicks erhalten
    map(() => 'Wurde geklickt'),
    concatWith(of('Abgeschlossen'))    // Nach Abschluss zusätzliche Nachricht
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });

// Dasselbe Verhalten mit Creation Function-Version...
// ❌ Creation Function-Version - Hauptstream muss separat geschrieben werden
import { concat } from 'rxjs';
concat(
  clicks$.pipe(
    take(3),
    map(() => 'Wurde geklickt')
  ),
  of('Abgeschlossen')
).subscribe(console.log);
```

### Zusammenfassung

- **`concat`**: Optimal, wenn mehrere Streams einfach verbunden werden sollen
- **`concatWith`**: Optimal, wenn dem Hauptstream Transformationen oder Verarbeitungen hinzugefügt werden sollen, während Folgeaktionen hinzugefügt werden


## ⚠️ Wichtige Hinweise

### Verzögerung durch Warten auf Abschluss

Die nächsten Observables starten nicht, bevor das ursprüngliche Observable abgeschlossen ist.

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // Mit 3 abschließen
  concatWith(of('Abgeschlossen'))
).subscribe(console.log);
// Ausgabe: 0 → 1 → 2 → Abgeschlossen
```

### Fehlerbehandlung

Wenn im vorherigen Observable ein Fehler auftritt, werden die nachfolgenden Observables nicht ausgeführt.

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('Fehler aufgetreten'))
  .pipe(
    catchError((err: unknown) => of('Fehler behoben')),
    concatWith(of('Nächste Verarbeitung'))
  )
  .subscribe(console.log);
// Ausgabe: Fehler behoben → Nächste Verarbeitung
```


## 📚 Verwandte Operatoren

- **[concat](/de/guide/creation-functions/combination/concat)** - Creation Function-Version
- **[mergeWith](/de/guide/operators/combination/mergeWith)** - Parallele Verbindung, Pipeable-Version
- **[concatMap](/de/guide/operators/transformation/concatMap)** - Sequentielles Mapping jedes Werts
